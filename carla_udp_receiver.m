function carla_udp_receiver()
% CARLA_UDP_RECEIVER - The definitive, feature-complete dashboard for CARLA,
% with a robust, pixel-perfect layout, 3D plots, dynamic graphs, user controls,
% and an integrated system log for MATLAB errors and warnings.

% ==============================================================================
% -- Configuration -------------------------------------------------------------
% ==============================================================================
UDP_PORT = 10000;
DATA_FOLDER = 'matlab_data';
LIDAR_PLOT_RANGE = 50;
GRAPH_HISTORY_LENGTH = 100;
TRAIL_LENGTH = 200;
MAX_LOG_LINES = 100; % Max number of lines to keep in the UI log

% ==============================================================================
% -- Initialization & Main Loop ------------------------------------------------
% ==============================================================================
udpPort = []; uiHandles = [];
if ~exist(DATA_FOLDER, 'dir'), mkdir(DATA_FOLDER); end

latestData = []; 
newDataAvailable = false;
chunkBuffer = struct('frame', -1, 'chunks', {{}}, 'total_chunks', 0);

try
    [uiHandles] = setupDashboard(GRAPH_HISTORY_LENGTH);
catch ME
    fprintf(2, 'FATAL: Error setting up the dashboard: %s\n', ME.message);
    fprintf(2, 'In file: %s, at line: %d\n', ME.stack(1).file, ME.stack(1).line);
    return;
end

fprintf('Starting UDP receiver on port %d...\n', UDP_PORT);
try
    udpPort = udpport("LocalPort", UDP_PORT, "Timeout", 1);
    configureCallback(udpPort, "terminator", @readUDPCb);
    logMessage(uiHandles, 'UDP receiver started successfully.', 'INFO', MAX_LOG_LINES);
catch ME
    logMessage(uiHandles, ['UDP port initialization failed: ' ME.message], 'ERROR', MAX_LOG_LINES);
    cleanup(udpPort, uiHandles); return;
end

cleanupObj = onCleanup(@() cleanup(udpPort, uiHandles));

% --- Main Loop ---
isPaused = false;
lastWarningMessage = '';
while ishandle(uiHandles.fig)
    if newDataAvailable
        if ~isPaused
            try
                processedData = preprocessData(latestData);
                updateDashboard(uiHandles, processedData, GRAPH_HISTORY_LENGTH, LIDAR_PLOT_RANGE, TRAIL_LENGTH);
                
                fileName = fullfile(DATA_FOLDER, sprintf('frame_%08d.mat', processedData.frame));
                save(fileName, 'processedData');
            catch ME
                logMessage(uiHandles, ['Error during main update: ' ME.message], 'ERROR', MAX_LOG_LINES);
            end
        end
        newDataAvailable = false;
    end
    
    [msg, ~] = lastwarn;
    if ~isempty(msg) && ~strcmp(msg, lastWarningMessage)
        logMessage(uiHandles, msg, 'WARNING', MAX_LOG_LINES);
        lastWarningMessage = msg;
    end

    uiHandles.isPaused = isPaused;
    pause(0.01);
end
disp('Dashboard closed. Shutting down...');

% ==============================================================================
% -- NESTED CALLBACK FUNCTION --------------------------------------------------
% ==============================================================================
    function readUDPCb(src, ~)
        try
            if src.NumBytesAvailable > 0
                rawData = read(src, src.NumBytesAvailable, "uint8");
                packet = jsondecode(char(rawData'));
                
                if packet.frame ~= chunkBuffer.frame
                    chunkBuffer.frame = packet.frame;
                    chunkBuffer.total_chunks = packet.total_chunks;
                    chunkBuffer.chunks = cell(1, packet.total_chunks);
                end
                
                try
                    chunk_index = packet.chunk + 1;
                    chunkBuffer.chunks{chunk_index} = base64decode(packet.data);
                catch
                    logMessage(uiHandles, 'Base64 decoding failed for a chunk.', 'WARNING', MAX_LOG_LINES); return;
                end
                
                if nnz(~cellfun('isempty', chunkBuffer.chunks)) == chunkBuffer.total_chunks
                    try
                        fullPayload = [chunkBuffer.chunks{:}];
                        
                        % --- MODIFIED: Decompression is no longer needed ---
                        % The fullPayload is now the raw JSON bytes.
                        finalData = jsondecode(char(fullPayload')); % Directly decode the payload
                        
                        latestData = finalData;
                        newDataAvailable = true;
                        chunkBuffer = struct('frame', -1, 'chunks', {{}}, 'total_chunks', 0);
                    catch ME
                        logMessage(uiHandles, ['Frame ' num2str(chunkBuffer.frame) ' Error: ' ME.message], 'ERROR', MAX_LOG_LINES);
                        chunkBuffer = struct('frame', -1, 'chunks', {{}}, 'total_chunks', 0);
                    end
                end
            end
        catch ME
            logMessage(uiHandles, ['UDP Read Error: ' ME.message], 'ERROR', MAX_LOG_LINES);
        end
    end
end

% ==============================================================================
% -- Data Pre-processing Function ----------------------------------------------
% ==============================================================================
function processedData = preprocessData(rawData)
persistent lastTimestamp lastDriverInputTime;
if isempty(lastTimestamp), lastTimestamp = rawData.timestamp; lastDriverInputTime = rawData.timestamp; end
INATTENTION_THRESHOLD_S=3.0; CONTROL_INPUT_THRESHOLD=0.05; MIN_SPEED_FOR_CHECK_KPH=5;

timeSinceLastPacket = rawData.timestamp - lastTimestamp;
if timeSinceLastPacket>0.5, networkStatus=0; elseif timeSinceLastPacket>0.1, networkStatus=0.5; else, networkStatus=1; end
lastTimestamp = rawData.timestamp;

if isfield(rawData, 'fused_state') && isstruct(rawData.fused_state), sensorFusionStatus=1; else, sensorFusionStatus=0; end

fallbackInitiation=1;
if isfield(rawData, 'sensor_health')
    sensorNames=fieldnames(rawData.sensor_health);
    numWarnings=0; numDead=0;
    for i=1:length(sensorNames), statuses=rawData.sensor_health.(sensorNames{i}); numWarnings=numWarnings+sum(strcmp(statuses,'NO_DATA')); numDead=numDead+sum(strcmp(statuses,'DEAD')); end
    if numDead>0, fallbackInitiation=0; elseif numWarnings>0, fallbackInitiation=0.5; end
else
    fallbackInitiation = 0;
end

isDriving=abs(rawData.control.steer)>CONTROL_INPUT_THRESHOLD || rawData.control.throttle>CONTROL_INPUT_THRESHOLD || rawData.control.brake>CONTROL_INPUT_THRESHOLD;
if isDriving, lastDriverInputTime=rawData.timestamp; end
driverReadiness=1.0; timeSinceInput=rawData.timestamp-lastDriverInputTime;
if rawData.speed>MIN_SPEED_FOR_CHECK_KPH && timeSinceInput>INATTENTION_THRESHOLD_S, driverReadiness=0.0; end

processedData.frame = rawData.frame;
processedData.networkLatency = timeSinceLastPacket;
processedData.networkStatus = networkStatus;
processedData.sensorFusionStatus = sensorFusionStatus;
processedData.fallbackInitiation = fallbackInitiation;
processedData.driverReadiness = driverReadiness;
processedData.position = rawData.position_raw;
processedData.speed = rawData.speed;
processedData.gnss = rawData.gnss;
processedData.imu = rawData.imu;
if isfield(rawData, 'cam_front'), processedData.cam_front = rawData.cam_front; end
if isfield(rawData, 'cam_rear'), processedData.cam_rear = rawData.cam_rear; end
if isfield(rawData, 'cam_left'), processedData.cam_left = rawData.cam_left; end
if isfield(rawData, 'cam_right'), processedData.cam_right = rawData.cam_right; end
if isfield(rawData, 'cam_interior'), processedData.cam_interior = rawData.cam_interior; end
if isfield(rawData, 'lidar'), processedData.lidar = rawData.lidar; end
end

% ==============================================================================
% -- PERFECTED DASHBOARD SETUP -------------------------------------------------
% ==============================================================================
function [uiHandles] = setupDashboard(historyLength)
fig = uifigure('Name', 'CARLA Final Diagnostics Dashboard', 'Color', [0.94 0.94 0.94], 'Position', [50 50, 1600, 950]);
uiHandles.fig = fig;
uiHandles.isPaused = false;

gl = uigridlayout(fig, [3, 4]);
gl.RowHeight = {250, '2x', 200};
gl.ColumnWidth = {'1x', '1x', '1x', '1x'};

p_speed=uipanel(gl,'Title','Speed (km/h)','FontWeight','bold'); p_speed.Layout.Row=1; p_speed.Layout.Column=1;
g_speed=uigridlayout(p_speed,[1 1]);
uiHandles.speedGauge=uigauge(g_speed,'ScaleColors',{[0 .8 .4],[.9 .8 0],[1 .2 .2]},'ScaleColorLimits',[0 60;60 100;100 160]);

p_imu=uipanel(gl,'Title','IMU - Accelerometer','FontWeight','bold'); p_imu.Layout.Row=1; p_imu.Layout.Column=2;
uiHandles.imuAx=uiaxes(p_imu);
hold(uiHandles.imuAx,'on'); grid(uiHandles.imuAx,'on');
uiHandles.accelXPlot=plot(uiHandles.imuAx,NaN(1,historyLength),'r-','DisplayName','X');
uiHandles.accelYPlot=plot(uiHandles.imuAx,NaN(1,historyLength),'g-','DisplayName','Y');
uiHandles.accelZPlot=plot(uiHandles.imuAx,NaN(1,historyLength),'b-','DisplayName','Z');
xlabel(uiHandles.imuAx,'Frames Ago'); ylabel(uiHandles.imuAx,'m/s^2'); legend(uiHandles.imuAx);

p_gnss=uipanel(gl,'Title','GNSS Trajectory','FontWeight','bold'); p_gnss.Layout.Row=1; p_gnss.Layout.Column=3;
uiHandles.gnssAx=uiaxes(p_gnss);
hold(uiHandles.gnssAx,'on'); grid(uiHandles.gnssAx,'on'); axis(uiHandles.gnssAx,'equal');
xlabel(uiHandles.gnssAx,'Longitude'); ylabel(uiHandles.gnssAx,'Latitude');
uiHandles.gnssPlot=plot(uiHandles.gnssAx,NaN,NaN,'-m','LineWidth',1.5);

p_diag=uipanel(gl,'Title','Diagnostics & Controls','FontWeight','bold'); p_diag.Layout.Row=1; p_diag.Layout.Column=4;
g_diag=uigridlayout(p_diag,[2 1],'RowHeight',{'2x','fit'});
p_diag_metrics=uipanel(g_diag,'Title',''); p_diag_metrics.Layout.Row=1;
g_diag_metrics=uigridlayout(p_diag_metrics,[5,2],'ColumnWidth',{'fit','1x'},'Padding',5);
uilabel(g_diag_metrics,'Text','Frame:'); uiHandles.frameLabel=uilabel(g_diag_metrics,'Text','N/A','FontWeight','bold');
uilabel(g_diag_metrics,'Text','Net Latency:'); uiHandles.latencyLabel=uilabel(g_diag_metrics,'Text','N/A','FontWeight','bold');
uilabel(g_diag_metrics,'Text','Sensor Fusion:'); uiHandles.fuzzyFusionLabel=uilabel(g_diag_metrics,'Text','N/A','FontWeight','bold');
uilabel(g_diag_metrics,'Text','Fallback:'); uiHandles.fuzzyFallbackLabel=uilabel(g_diag_metrics,'Text','N/A','FontWeight','bold');
uilabel(g_diag_metrics,'Text','Driver Ready:'); uiHandles.fuzzyReadinessLabel=uilabel(g_diag_metrics,'Text','N/A','FontWeight','bold');
p_ctrl=uipanel(g_diag,'Title',''); p_ctrl.Layout.Row=2;
g_ctrl=uigridlayout(p_ctrl,[1 2],'Padding',5);
uiHandles.pauseButton=uibutton(g_ctrl,'Text','Pause');
uiHandles.saveButton=uibutton(g_ctrl,'Text','Save Snapshot');
uiHandles.pauseButton.ButtonPushedFcn=@(src,~)pauseCallback(src,uiHandles);
uiHandles.saveButton.ButtonPushedFcn=@(src,~)saveCallback(uiHandles);

p_cameras=uipanel(gl,'Title','Camera Feeds','FontWeight','bold'); p_cameras.Layout.Row=2; p_cameras.Layout.Column=[1 2];
g_cameras=uigridlayout(p_cameras,[2 3],'ColumnWidth',{'1x','1x','1x'},'RowHeight',{'1x','1x'});
uiHandles.camLeftAx=uiaxes(g_cameras,'XTick',[],'YTick',[]); uiHandles.camLeftAx.Layout.Row=1;uiHandles.camLeftAx.Layout.Column=1; title(uiHandles.camLeftAx,'Left');
uiHandles.camFrontAx=uiaxes(g_cameras,'XTick',[],'YTick',[]); uiHandles.camFrontAx.Layout.Row=1;uiHandles.camFrontAx.Layout.Column=2; title(uiHandles.camFrontAx,'Front');
uiHandles.camRightAx=uiaxes(g_cameras,'XTick',[],'YTick',[]); uiHandles.camRightAx.Layout.Row=1;uiHandles.camRightAx.Layout.Column=3; title(uiHandles.camRightAx,'Right');
uiHandles.camInteriorAx=uiaxes(g_cameras,'XTick',[],'YTick',[]); uiHandles.camInteriorAx.Layout.Row=2;uiHandles.camInteriorAx.Layout.Column=1; title(uiHandles.camInteriorAx,'Interior');
uiHandles.camRearAx=uiaxes(g_cameras,'XTick',[],'YTick',[]); uiHandles.camRearAx.Layout.Row=2;uiHandles.camRearAx.Layout.Column=3; title(uiHandles.camRearAx,'Rear');

p_traj=uipanel(gl,'Title','Vehicle Trajectory','FontWeight','bold'); p_traj.Layout.Row=2; p_traj.Layout.Column=3;
uiHandles.trajAx=uiaxes(p_traj);
hold(uiHandles.trajAx,'on'); grid(uiHandles.trajAx,'on'); axis(uiHandles.trajAx,'equal');
xlabel(uiHandles.trajAx,'X (m)'); ylabel(uiHandles.trajAx,'Y (m)');
uiHandles.trajPlot=plot(uiHandles.trajAx,NaN,NaN,'-c','LineWidth',2);
uiHandles.currentPosPlot=plot(uiHandles.trajAx,NaN,NaN,'bo','MarkerFaceColor','b','MarkerSize',8);
legend(uiHandles.trajAx,'Path','Current','Location','northwest');

p_lidar=uipanel(gl,'Title','3D LiDAR Point Cloud','FontWeight','bold'); p_lidar.Layout.Row=2; p_lidar.Layout.Column=4;
uiHandles.lidarAx=uiaxes(p_lidar);
hold(uiHandles.lidarAx,'on'); grid(uiHandles.lidarAx,'on'); axis(uiHandles.lidarAx,'equal');
xlabel(uiHandles.lidarAx,'X (m)'); ylabel(uiHandles.lidarAx,'Y (m)'); zlabel(uiHandles.lidarAx,'Z (m)');
view(uiHandles.lidarAx, -45, 30);
uiHandles.lidarPlot = scatter3(uiHandles.lidarAx, NaN, NaN, NaN, 10, NaN, 'filled');

p_log=uipanel(gl,'Title','System Log','FontWeight','bold'); p_log.Layout.Row=3; p_log.Layout.Column=[1 4];
g_log=uigridlayout(p_log,[1 1]);
uiHandles.logArea = uitextarea(g_log, 'Value',{''}, 'Editable','off', 'FontName', 'Monospaced');
end

% ==============================================================================
% -- DASHBOARD UPDATE FUNCTION -------------------------------------------------
% ==============================================================================
function updateDashboard(uiHandles, data, historyLength, lidarRange, trailLength)
    persistent accelHistory gnssHistory trajHistory;
    if isempty(accelHistory)
        accelHistory=nan(historyLength,3); gnssHistory=nan(historyLength,2); trajHistory=nan(trailLength,2);
    end

    uiHandles.speedGauge.Value = data.speed;
    try_imshow(uiHandles.camFrontAx,data,'cam_front'); try_imshow(uiHandles.camRearAx,data,'cam_rear');
    try_imshow(uiHandles.camLeftAx,data,'cam_left'); try_imshow(uiHandles.camRightAx,data,'cam_right');
    try_imshow(uiHandles.camInteriorAx,data,'cam_interior');
    accelHistory=[accelHistory(2:end,:); [data.imu.accelerometer.x,data.imu.accelerometer.y,data.imu.accelerometer.z]];
    set(uiHandles.accelXPlot,'YData',accelHistory(:,1)); set(uiHandles.accelYPlot,'YData',accelHistory(:,2)); set(uiHandles.accelZPlot,'YData',accelHistory(:,3));
    set(uiHandles.imuAx,'XLim',[0 historyLength]);
    gnssHistory=[gnssHistory(2:end,:); [data.gnss.longitude, data.gnss.latitude]];
    set(uiHandles.gnssPlot,'XData',gnssHistory(:,1),'YData',gnssHistory(:,2));
    uiHandles.frameLabel.Text=num2str(data.frame);
    uiHandles.latencyLabel.Text=sprintf('%.1f ms',data.networkLatency*1000);
    uiHandles.fuzzyFusionLabel.Text=num2str(data.sensorFusionStatus);
    uiHandles.fuzzyFallbackLabel.Text=num2str(data.fallbackInitiation);
    uiHandles.fuzzyReadinessLabel.Text=num2str(data.driverReadiness);

    try
        if isfield(data,'lidar') && ~isempty(data.lidar)
            raw_points=typecast(base64decode(data.lidar),'single');
            points=reshape(raw_points,3,[])';
            set(uiHandles.lidarPlot,'XData',points(:,1),'YData',points(:,2),'ZData',points(:,3),'CData',points(:,3));
            axis(uiHandles.lidarAx,[-lidarRange lidarRange -lidarRange lidarRange -5 lidarRange/2]);
            colormap(uiHandles.lidarAx,'jet'); clim(uiHandles.lidarAx,[-2, 5]);
        end
    catch ME, logMessage(uiHandles,['LiDAR plot error: ' ME.message],'ERROR',evalin('base','MAX_LOG_LINES')); 
    end

    newPos=[data.position.x,data.position.y];
    trajHistory=[trajHistory(2:end,:); newPos];
    set(uiHandles.trajPlot,'XData',trajHistory(:,1),'YData',trajHistory(:,2));
    set(uiHandles.currentPosPlot,'XData',newPos(1),'YData',newPos(2));
    if ~any(isnan(trajHistory(:))), axis(uiHandles.trajAx,[min(trajHistory(:,1))-10,max(trajHistory(:,1))+10,min(trajHistory(:,2))-10,max(trajHistory(:,2))+10]); end

    drawnow limitrate;
end

% ==============================================================================
% -- LOGGING, UI CONTROL, & UTILITIES ------------------------------------------
% ==============================================================================
function logMessage(uiHandles, message, level, maxLogLines)
    timestamp = string(datetime('now'), 'HH:mm:ss.SSS');
    formattedMsg = sprintf('[%s] [%s] %s', timestamp, upper(level), message);
    if strcmpi(level, 'ERROR'), fprintf(2, '%s\n', formattedMsg); else, fprintf('%s\n', formattedMsg); end
    if isfield(uiHandles, 'logArea') && ishandle(uiHandles.logArea)
        currentLog = uiHandles.logArea.Value;
        newLog = [currentLog; {formattedMsg}];
        if numel(newLog) > maxLogLines, newLog = newLog(end-maxLogLines+1:end); end
        uiHandles.logArea.Value = newLog;
        scroll(uiHandles.logArea, 'bottom');
    end
end

function pauseCallback(src, uiHandles)
    if uiHandles.isPaused, src.Text='Pause'; else, src.Text='Resume'; end
    evalin('base', 'isPaused = ~isPaused;');
end

function saveCallback(uiHandles)
    maxLogLines = evalin('base', 'MAX_LOG_LINES');
    logMessage(uiHandles, 'Saving dashboard snapshot...', 'INFO', maxLogLines);
    d=uiprogressdlg(uiHandles.fig,'Title','Saving Snapshot','Indeterminate','on');
    drawnow;
    try
        filename=sprintf('dashboard_snapshot_%s.png',string(datetime('now'),'yyyyMMdd_HHmmss'));
        exportgraphics(uiHandles.fig, filename, 'Resolution', 150);
        logMessage(uiHandles, ['Dashboard saved to ' filename], 'INFO', maxLogLines);
    catch ME, logMessage(uiHandles,['Save failed: ' ME.message],'ERROR', maxLogLines); 
    end
    close(d);
end

function try_imshow(ax, data, field)
    if isfield(data,field) && ~isempty(data.(field))
        try imshow(imdecode(base64decode(data.(field)),'jpg'),'Parent',ax); catch, cla(ax); text(ax,0.5,0.5,'Img Error','Color','r','HA','center','FontSize',14); end
    else, cla(ax); text(ax,0.5,0.5,'No Img Data','Color','y','HA','center','FontSize',14); 
    end
end

% --- REMOVED: gunzip_matlab is no longer needed ---

function cleanup(udpPort, uiHandles)
disp('Cleaning up resources...');
if ~isempty(udpPort)&&isvalid(udpPort), delete(udpPort); disp('UDP port closed.'); end
if ~isempty(uiHandles)&&isfield(uiHandles,'fig')&&ishandle(uiHandles.fig), delete(uiHandles.fig); end
disp('Cleanup complete.');
end

function decoded=base64decode(encoded)
decoded=typecast(org.apache.commons.codec.binary.Base64.decodeBase64(encoded),'uint8');
end