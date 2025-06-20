# ==============================================================================
# -- Imports -------------------------------------------------------------------
# ==============================================================================
import glob
import os
import sys
import random
import time
import threading
import traceback
import socket
import json
import base64
import csv
import struct
from queue import Queue, Empty
from typing import Dict, Any, List, Tuple, Optional

import pygame
from pygame.locals import *
import numpy as np
import carla
import cv2

from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
from scipy.linalg import block_diag
from sklearn.cluster import DBSCAN

# ==============================================================================
# -- Constants -----------------------------------------------------------------
# ==============================================================================
IMAGE_WIDTH, IMAGE_HEIGHT = 320, 240
GRID_COLS, GRID_ROWS = 3, 2
WINDOW_WIDTH, WINDOW_HEIGHT = IMAGE_WIDTH * GRID_COLS, IMAGE_HEIGHT * GRID_ROWS
TICK_RATE = 20
VEHICLE_MODEL = 'vehicle.tesla.model3'
NUM_BACKUPS = 2
NUM_NPC_VEHICLES = 20 # Number of NPCs for V2V
DATA_DIR = 'data'
LOG_DIR = os.path.join(DATA_DIR, 'logs')
CAMERA_DIRS = {
    'front': os.path.join(DATA_DIR, 'images/front'), 'back': os.path.join(DATA_DIR, 'images/back'),
    'left': os.path.join(DATA_DIR, 'images/left'), 'right': os.path.join(DATA_DIR, 'images/right'),
    'interior': os.path.join(DATA_DIR, 'images/interior')
}
WEATHER_PRESETS = {
    'ClearNoon': carla.WeatherParameters.ClearNoon,
    'CloudyNoon': carla.WeatherParameters.CloudyNoon,
    'WetNoon': carla.WeatherParameters.WetNoon,
    'HardRainNoon': carla.WeatherParameters.HardRainNoon
}
MAP_LAYERS = [carla.MapLayer.NONE, carla.MapLayer.Buildings, carla.MapLayer.Decals, carla.MapLayer.Foliage, carla.MapLayer.Ground, carla.MapLayer.ParkedVehicles, carla.MapLayer.Particles, carla.MapLayer.Props, carla.MapLayer.StreetLights, carla.MapLayer.Walls, carla.MapLayer.All]
COMMAND_LISTEN_IP = '127.0.0.1'
COMMAND_LISTEN_PORT = 10001


# ==============================================================================
# -- Helper Classes ------------------------------------------------------------
# ==============================================================================
class CommandReceiver(threading.Thread):
    """
    A thread that listens for UDP commands from the FDIR system in MATLAB.
    """
    def __init__(self, command_queue: Queue, ip: str, port: int):
        super().__init__()
        self.daemon = True
        self.queue = command_queue
        self.ip = ip
        self.port = port
        self.running = True
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(1.0) # Timeout to allow checking self.running
        print(f"Command receiver thread will listen on {ip}:{port}")

    def run(self):
        try:
            self.sock.bind((self.ip, self.port))
            print("Command receiver bound to socket and listening.")
        except Exception as e:
            print(f"[ERROR] Could not bind command receiver socket: {e}")
            self.running = False
            return

        while self.running:
            try:
                data, _ = self.sock.recvfrom(1024)
                if data:
                    command_str = data.decode('utf-8')
                    try:
                        command_obj = json.loads(command_str)
                        # Don't print control commands to avoid flooding the console
                        if command_obj.get('command') != 'SET_VEHICLE_CONTROL':
                             print(f"[CMD RECEIVED]: {command_obj}")
                        self.queue.put(command_obj)
                    except json.JSONDecodeError:
                        print(f"[WARN] Received invalid JSON command: {command_str}")
            except socket.timeout:
                continue # Allows the loop to check self.running status
            except Exception as e:
                if self.running: # Don't print error if we are intentionally stopping
                    print(f"Error in command receiver thread: {e}")
                    time.sleep(1) # Avoid spamming errors

    def stop(self):
        print("Stopping command receiver thread...")
        self.running = False
        self.sock.close()

class ImageSaver(threading.Thread):
    def __init__(self): super().__init__(); self.daemon=True; self.queue=Queue(); self.running=True
    def run(self):
        while self.running:
            try: (filepath, surface) = self.queue.get(timeout=1); pygame.image.save(surface, filepath); self.queue.task_done()
            except Empty: continue
    def save_image(self, fp, surf): self.queue.put((fp, surf))
    def stop(self): self.running=False; self.queue.join()

class KalmanFilterSystem:
    def __init__(self, dt):
        self.dt=dt; self.kf=KalmanFilter(dim_x=6, dim_z=2); self.kf.F=np.array([[1,0,dt,0,0.5*dt**2,0],[0,1,0,dt,0,0.5*dt**2],[0,0,1,0,dt,0],[0,0,0,1,0,dt],[0,0,0,0,1,0],[0,0,0,0,0,1]]); self.kf.H=np.array([[1,0,0,0,0,0],[0,1,0,0,0,0]]); self.kf.P*=1000.; self.kf.R=np.diag([0.5**2, 0.5**2]); q_3d=Q_discrete_white_noise(dim=3,dt=dt,var=0.1); self.kf.Q=block_diag(q_3d,q_3d); self.initialized=False; print("Kalman Filter (Ego-Motion) initialized.")
    def initialize(self, pos): self.kf.x[0,0]=pos[0]; self.kf.x[1,0]=pos[1]; self.initialized=True
    def update(self, gps, imu):
        if not self.initialized and gps is not None: self.initialize(gps)
        if imu is not None: self.kf.x[4,0]=imu[0]; self.kf.x[5,0]=imu[1]
        self.kf.predict()
        if gps is not None: self.kf.update(gps)
    def get_fused_state(self):
        if not self.initialized: return None
        return {'x':float(self.kf.x[0,0]), 'y':float(self.kf.x[1,0]), 'vx':float(self.kf.x[2,0]), 'vy':float(self.kf.x[3,0])}

class ObjectTracker:
    def __init__(self):
        self.detected_obstacles = []
        print("Object Tracker initialized.")
    def process_lidar_to_obstacles(self, point_cloud, vehicle_transform):
        try:
            points = np.frombuffer(point_cloud.raw_data, dtype=np.float32).reshape(-1, 4)
            points_xyz = points[:, :3]
            ground_height_threshold = -1.4 
            non_ground_indices = points_xyz[:, 2] > ground_height_threshold
            points_xyz = points_xyz[non_ground_indices]
            if len(points_xyz) < 10: return []
            db = DBSCAN(eps=0.8, min_samples=15).fit(points_xyz[:, :2])
            labels = db.labels_
            obstacles = []
            unique_labels = set(labels)
            for label in unique_labels:
                if label == -1: continue
                cluster_points = points_xyz[labels == label]
                centroid = np.mean(cluster_points, axis=0)
                min_coords = np.min(cluster_points, axis=0); max_coords = np.max(cluster_points, axis=0)
                dimensions = max_coords - min_coords
                relative_location = carla.Location(x=float(centroid[0]), y=float(centroid[1]), z=float(centroid[2]))
                world_location = vehicle_transform.transform(relative_location)
                obstacles.append({'id': int(label), 'position': {'x': world_location.x, 'y': world_location.y, 'z': world_location.z}, 'dimensions': {'length': float(dimensions[0]), 'width': float(dimensions[1]), 'height': float(dimensions[2])}})
            return obstacles
        except Exception as e: print(f"Error in LiDAR processing: {e}"); return []
    def update(self, lidar_point_cloud, vehicle_transform):
        if lidar_point_cloud: self.detected_obstacles = self.process_lidar_to_obstacles(lidar_point_cloud, vehicle_transform)
        else: self.detected_obstacles = []
    def get_obstacles(self): return self.detected_obstacles

class HUD:
    def __init__(self, width: int, height: int):
        self.dim = (width, height); self.font = pygame.font.SysFont('Arial', 14); self.help_font = pygame.font.SysFont('Monospace', 14)
        self.large_font = pygame.font.SysFont('Arial', 48, bold=True)
        self.show_help = False
        self.HEALTH_COLORS = {"OK": (0, 255, 0), "NO_DATA": (255, 255, 0), "DEAD": (255, 0, 0)}

    def render(self, display: pygame.Surface, sim_state: Dict[str, Any]):
        if display is None: return
        self._render_hud_info(display, sim_state)
        self._render_sensor_health(display, sim_state.get('sensor_health', {}), sim_state.get('active_sensors', {}))
        if sim_state.get('safe_mode_locked', False): self._render_safe_mode_warning(display)
        if self.show_help: self._render_help_text(display)

    def _render_hud_info(self, display: pygame.Surface, sim_state: Dict[str, Any]):
        mode_text = sim_state.get('matlab_mode', 'MANUAL')
        if sim_state.get('safe_mode_locked', False):
            mode_text = 'LOCKED'
        info_text = [
            f"Speed: {sim_state['speed']:.2f} km/h", f"Map: {sim_state['map_name']}",
            f"Fused Pos: {sim_state.get('fused_pos', 'N/A')}", f"Collisions: {sim_state['collision_count']}",
            f"Lane Invasions: {sim_state['lane_invasion_count']}", "-----------",
            f"Throttle: {sim_state['control']['throttle']:.2f}", f"Steer: {sim_state['control']['steer']:.2f}",
            f"Brake: {sim_state['control']['brake']:.2f}", "-----------",
            f"Control Mode: {mode_text}", f"Img Record: {'ON' if sim_state['recording_images'] else 'OFF'}",
            f"Sim Record: {'ON' if sim_state['recording_sim'] else 'OFF'}"
        ]
        for i, text in enumerate(info_text): display.blit(self.font.render(text, True, (255, 255, 255)), (10, 10 + i * 20))
    
    def _render_sensor_health(self, display: pygame.Surface, health_data: Dict[str, List[str]], active_indices: Dict[str, int]):
        x_offset = WINDOW_WIDTH - 280; y_offset = 10
        display.blit(self.font.render("--- Sensor Health (Active*) ---", True, (255, 255, 255)), (x_offset, y_offset)); y_offset += 20
        for name in sorted(health_data.keys()):
            statuses = health_data[name]
            active_idx = active_indices.get(name, 0)
            name_surface = self.font.render(f"{name.replace('_', ' ').title()}: ", True, (255, 255, 255)); display.blit(name_surface, (x_offset, y_offset))
            current_x = x_offset + name_surface.get_width()
            for i, status in enumerate(statuses):
                color = self.HEALTH_COLORS.get(status, (200, 200, 200))
                status_text = f"*{status}*" if i == active_idx else status
                status_surface = self.font.render(status_text, True, color); display.blit(status_surface, (current_x, y_offset)); current_x += status_surface.get_width()
                if i < len(statuses) - 1:
                    sep_surface = self.font.render(" | ", True, (255, 255, 255)); display.blit(sep_surface, (current_x, y_offset)); current_x += sep_surface.get_width()
            y_offset += 20
            
    def _render_safe_mode_warning(self, display: pygame.Surface):
        s = pygame.Surface((WINDOW_WIDTH, 80)); s.set_alpha(180); s.fill((255, 0, 0))
        display.blit(s, (0, (WINDOW_HEIGHT / 2) - 40))
        text_surf = self.large_font.render('SAFE MODE LOCKED', True, (255, 255, 255))
        text_rect = text_surf.get_rect(center=(WINDOW_WIDTH/2, WINDOW_HEIGHT/2))
        display.blit(text_surf, text_rect)
        sub_text_surf = self.font.render('Press L to unlock', True, (255, 255, 255))
        sub_text_rect = sub_text_surf.get_rect(center=(WINDOW_WIDTH/2, WINDOW_HEIGHT/2 + 30))
        display.blit(sub_text_surf, sub_text_rect)

    def _render_help_text(self, display: pygame.Surface):
        help_text = ["W/S: Throttle/Brake", "A/D: Steer Left/Right", "Q: Toggle Reverse", "Space: Hand Brake", "P: Control now managed by MATLAB", "L: Unlock Safe Mode", "C: Next Weather (Shift+C: Prev)", "V: Next Map Layer (Shift+V: Prev)", "B: Load Layer (Shift+B: Unload)", "R: Toggle Image Recording", "Ctrl+R: Toggle Sim Recording", "3: Switch to Town03", "0: Switch to Town10HD", "F1: Toggle HUD", "H: Toggle Help", "ESC: Quit"]
        s = pygame.Surface((300, len(help_text) * 22 + 20)); s.set_alpha(200); s.fill((0, 0, 0)); display.blit(s, (50, 50))
        for i, text in enumerate(help_text): display.blit(self.help_font.render(text, True, (255, 255, 255)), (60, 60 + i * 22))

class KeyboardController:
    def __init__(self, sim):
        self.simulation = sim
        self._control_state = {'throttle': 0.0, 'steer': 0.0, 'brake': 0.0, 'hand_brake': False, 'reverse': False}
    def parse_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT: return False
            elif event.type in (pygame.KEYDOWN, pygame.KEYUP): self._parse_key_event(event)
        return True
    def get_control_state(self): return self._control_state
    def _parse_key_event(self, event):
        is_keydown = (event.type == pygame.KEYDOWN); key = event.key; mods = pygame.key.get_mods()
        is_ctrl = (mods & pygame.KMOD_CTRL); is_shift = (mods & pygame.KMOD_SHIFT)
        if key == K_w: self._control_state['throttle'] = 1.0 if is_keydown else 0.0
        elif key == K_s: self._control_state['brake'] = 1.0 if is_keydown else 0.0
        elif key == K_a: self._control_state['steer'] = -0.7 if is_keydown else 0.0
        elif key == K_d: self._control_state['steer'] = 0.7 if is_keydown else 0.0
        elif key == K_q and is_keydown: self._control_state['reverse'] = not self._control_state['reverse']
        elif key == K_SPACE: self._control_state['hand_brake'] = is_keydown
        if not is_keydown: return
        if key == K_p and not is_ctrl: 
            print("[INFO] Autopilot is now fully managed by the MATLAB controller.")
        elif key == K_l: self.simulation.reset_safe_mode_lock()
        elif key == K_r and not is_ctrl: self.simulation.toggle_image_recording()
        elif key == K_r and is_ctrl: self.simulation.toggle_sim_recording()
        elif key == K_c: self.simulation.change_weather(-1 if is_shift else 1)
        elif key == K_v: self.simulation.change_map_layer(-1 if is_shift else 1)
        elif key == K_b: self.simulation.toggle_current_map_layer(unload=is_shift)
        elif key == K_F1: self.simulation.toggle_hud()
        elif key == K_h: self.simulation.toggle_help()
        elif key == K_3: self.simulation.change_map('Town03')
        elif key == K_0: self.simulation.change_map('Town10HD')
        elif key == K_ESCAPE: self.simulation.running = False

class SensorManager:
    def __init__(self, world, display_manager, sensor_type, transform, attached_vehicle, attributes=None, display_pos=None, camera_name=''):
        self.stype=sensor_type; self.cname=camera_name; self.surface=None; self.data=None; self.event_list=[]; self.lock=threading.Lock(); self.last_updated_frame=0
        bp_map={'RGB':'sensor.camera.rgb','LIDAR':'sensor.lidar.ray_cast','IMU':'sensor.other.imu','GNSS':'sensor.other.gnss','RADAR':'sensor.other.radar','ULTRASONIC':'sensor.other.obstacle','COLLISION':'sensor.other.collision','LANE_INVASION':'sensor.other.lane_invasion'}
        bp = world.get_blueprint_library().find(bp_map[sensor_type])
        if sensor_type == 'RGB': bp.set_attribute('image_size_x', str(IMAGE_WIDTH)); bp.set_attribute('image_size_y', str(IMAGE_HEIGHT))
        if attributes:
            for key, value in attributes.items():
                if bp.has_attribute(key): bp.set_attribute(key, str(value))
        self.sensor = world.spawn_actor(bp, transform, attach_to=attached_vehicle)
        callbacks = {'RGB':self._parse_image, 'COLLISION':self._parse_event, 'LANE_INVASION':self._parse_event}
        self.sensor.listen(callbacks.get(sensor_type, self._update_data))
        if display_manager and display_pos: display_manager.attach_sensor(self, display_pos)
    def get_health_status(self, current_frame):
        if not self.sensor or not self.sensor.is_alive: return "DEAD"
        if self.stype in ['COLLISION', 'LANE_INVASION']: return "OK"
        if current_frame - self.last_updated_frame > (TICK_RATE * 2): return "NO_DATA"
        return "OK"
    def _update_data(self, data): self.data = data; self.last_updated_frame=data.frame
    def _parse_image(self, image):
        self.last_updated_frame=image.frame; array = np.frombuffer(image.raw_data,dtype=np.uint8).reshape((image.height,image.width,4))[:,:,:3][:,:,::-1]; self.surface = pygame.surfarray.make_surface(array.swapaxes(0,1)); self.data = array
    def _parse_event(self, event):
        with self.lock: self.event_list.append(event)
    def get_events(self):
        with self.lock: events=self.event_list[:]; self.event_list.clear(); return events
    def destroy(self):
        if self.sensor and self.sensor.is_alive: self.sensor.destroy()

class DisplayManager:
    def __init__(self, grid_size):
        self.display=pygame.display.set_mode((grid_size[0]*IMAGE_WIDTH, grid_size[1]*IMAGE_HEIGHT), pygame.RESIZABLE)
        self.sensors=[]
    def attach_sensor(self, sensor, pos):
        self.sensors.append((sensor, pos))
    def render(self):
        self.display.fill((0,0,0))
        for sensor, pos in self.sensors:
            if sensor.surface: self.display.blit(sensor.surface, (pos[0]*IMAGE_WIDTH, pos[1]*IMAGE_HEIGHT))
        return self.display

class UDPDataSender:
    def __init__(self, ip='127.0.0.1', port=10000, chunk_size=4096):
        self.ip=ip; self.port=port; self.sock=socket.socket(socket.AF_INET, socket.SOCK_DGRAM); self.chunk_size=chunk_size; print(f"UDP sender initialized for MATLAB. Target: {ip}:{port}.")
    def send_string_chunks(self, payload, frame_id):
        try:
            chunks = [payload[i:i+self.chunk_size] for i in range(0, len(payload), self.chunk_size)]
            if not chunks: chunks.append('')
            for i, chunk in enumerate(chunks):
                packet = {'frame':frame_id, 'chunk':i, 'total_chunks':len(chunks), 'data':chunk}
                self.sock.sendto(json.dumps(packet).encode('utf-8'), (self.ip, self.port))
        except Exception as e: print(f"UDP send error: {e}")
    def close(self): self.sock.close()

# ==============================================================================
# -- CarlaSimulation Class -----------------------------------------------------
# ==============================================================================
class CarlaSimulation:
    def __init__(self):
        self.client=None; self.world=None; self.vehicle=None; self.display_manager=None; self.hud=None
        self.controller=None; self.udp_sender=None; self.image_saver=None; self.sensors={}; self.log_files={}; self.csv_writers={}
        self.kalman_filter=None; self.fused_state=None; self.running=True; self.matlab_mode = 'MANUAL'
        self.recording_images=False; self.recording_sim=False; self.show_hud=True; self.collision_count=0; self.lane_invasion_count=0
        self.weather_names = list(WEATHER_PRESETS.keys()); self.current_weather_index = 0; self.current_layer_index = 0
        self.object_tracker = None; self.detected_obstacles = []
        # V2V and FDIR state variables
        self.npc_vehicles = []
        self.command_queue = Queue()
        self.command_receiver = None
        self.active_sensor_indices = {}
        self.safe_mode_locked = False
        self.was_autopilot_on_lock = False

    def run(self):
        try:
            self._initialize()
            clock=pygame.time.Clock()
            while self.running:
                clock.tick(TICK_RATE)
                if not self.controller.parse_events(): break
                self._tick_simulation()
        finally: self._cleanup()

    def _initialize(self):
        pygame.init(); pygame.font.init(); self._setup_logging()
        self.client=carla.Client('localhost',2000); self.client.set_timeout(20.0)
        self.world=self.client.load_world('Town03')
        settings=self.world.get_settings(); settings.synchronous_mode=True; settings.fixed_delta_seconds=1.0/TICK_RATE
        self.world.apply_settings(settings)
        self.display_manager=DisplayManager((GRID_COLS,GRID_ROWS)); self.hud=HUD(WINDOW_WIDTH,WINDOW_HEIGHT); self.controller=KeyboardController(self)
        self.udp_sender=UDPDataSender(); self.image_saver=ImageSaver(); self.image_saver.start()
        self.command_receiver = CommandReceiver(self.command_queue, COMMAND_LISTEN_IP, COMMAND_LISTEN_PORT)
        self.command_receiver.start()
        self._setup_actors_and_sensors()
        self.kalman_filter=KalmanFilterSystem(1.0/TICK_RATE)
        self.object_tracker = ObjectTracker()

    def _setup_logging(self):
        os.makedirs(LOG_DIR, exist_ok=True); [os.makedirs(d, exist_ok=True) for d in CAMERA_DIRS.values()]
        log_paths={'collision':os.path.join(LOG_DIR,'collision_log.csv'), 'lane':os.path.join(LOG_DIR,'lane_invasion_log.csv')}
        headers={'collision':['ts','frame','intensity','actor_type'], 'lane':['ts','frame','lane_types']}
        for name, path in log_paths.items():
            self.log_files[name]=open(path, 'w', newline=''); self.csv_writers[name]=csv.writer(self.log_files[name]); self.csv_writers[name].writerow(headers[name])

    def _spawn_sensor_with_backups(self, name, s_type, transform, attributes=None, display_pos=None, camera_name=''):
        sensor_list = []
        primary_sensor = SensorManager(self.world, self.display_manager, s_type, transform, self.vehicle, attributes, display_pos, camera_name)
        sensor_list.append(primary_sensor)
        for i in range(NUM_BACKUPS):
            loc = transform.location; offset = (i + 1) * 0.05
            perturbed_transform = carla.Transform(carla.Location(loc.x+offset, loc.y+offset, loc.z), transform.rotation)
            backup_sensor = SensorManager(self.world, None, s_type, perturbed_transform, self.vehicle, attributes)
            sensor_list.append(backup_sensor)
        self.sensors[name] = sensor_list
        self.active_sensor_indices[name] = 0
        print(f"  - Spawned {name} with {NUM_BACKUPS} backups.")

    def _setup_actors_and_sensors(self):
        print("Spawning vehicle and full sensor suite with backups...")
        vehicle_bp = self.world.get_blueprint_library().find(VEHICLE_MODEL)
        spawn_point = random.choice(self.world.get_map().get_spawn_points())
        self.vehicle = self.world.spawn_actor(vehicle_bp, spawn_point)
        
        # This is the fully RESTORED sensor list
        self._spawn_sensor_with_backups('cam_front', 'RGB', carla.Transform(carla.Location(x=1.5,z=2.4)), display_pos=(1,0), camera_name='front')
        self._spawn_sensor_with_backups('cam_back', 'RGB', carla.Transform(carla.Location(x=-2.0,z=0.8),carla.Rotation(yaw=180)), display_pos=(1,2), camera_name='back') 
        self._spawn_sensor_with_backups('cam_left', 'RGB', carla.Transform(carla.Location(x=1.3,y=-0.9,z=1.2),carla.Rotation(yaw=-110)), display_pos=(0,0), camera_name='left')
        self._spawn_sensor_with_backups('cam_right', 'RGB', carla.Transform(carla.Location(x=1.3,y=0.9,z=1.2),carla.Rotation(yaw=110)), display_pos=(2,0), camera_name='right') 
        self._spawn_sensor_with_backups('cam_interior', 'RGB', carla.Transform(carla.Location(x=0.5,y=-0.3,z=1.4),carla.Rotation(pitch=-15,yaw=180)), display_pos=(1,1), camera_name='interior')
        self._spawn_sensor_with_backups('gnss', 'GNSS', carla.Transform(carla.Location(z=2.6)))
        self._spawn_sensor_with_backups('imu', 'IMU', carla.Transform(carla.Location(x=0,z=0.5)))
        self._spawn_sensor_with_backups('lidar_roof', 'LIDAR', carla.Transform(carla.Location(z=2.5)), {'range':'100', 'points_per_second':'100000'})
        self._spawn_sensor_with_backups('lidar_front', 'LIDAR', carla.Transform(carla.Location(x=2.5, z=0.5)), {'range': '50', 'points_per_second':'60000'})
        self._spawn_sensor_with_backups('lidar_back', 'LIDAR', carla.Transform(carla.Location(x=-2.5, z=0.5), carla.Rotation(yaw=180)), {'range': '50', 'points_per_second':'60000'})
        self._spawn_sensor_with_backups('radar_front', 'RADAR', carla.Transform(carla.Location(x=2.5,z=0.7)), {'range':'150'})
        self._spawn_sensor_with_backups('radar_back', 'RADAR', carla.Transform(carla.Location(x=-2.5,z=0.7), carla.Rotation(yaw=180)), {'range':'150'})
        self._spawn_sensor_with_backups('ultrasonic_front', 'ULTRASONIC', carla.Transform(carla.Location(x=2.2,z=0.5)))
        self._spawn_sensor_with_backups('ultrasonic_back', 'ULTRASONIC', carla.Transform(carla.Location(x=-2.2,z=0.5), carla.Rotation(yaw=180)))
        self._spawn_sensor_with_backups('radar_front_left', 'RADAR', carla.Transform(carla.Location(x=2.0, y=-0.9, z=0.7), carla.Rotation(yaw=-45)), {'range':'75'})
        self._spawn_sensor_with_backups('radar_front_right', 'RADAR', carla.Transform(carla.Location(x=2.0, y=0.9, z=0.7), carla.Rotation(yaw=45)), {'range':'75'})
        self._spawn_sensor_with_backups('radar_rear_left', 'RADAR', carla.Transform(carla.Location(x=-2.0, y=-0.9, z=0.7), carla.Rotation(yaw=-135)), {'range':'75'})
        self._spawn_sensor_with_backups('radar_rear_right', 'RADAR', carla.Transform(carla.Location(x=-2.0, y=0.9, z=0.7), carla.Rotation(yaw=135)), {'range':'75'})
        self._spawn_sensor_with_backups('collision', 'COLLISION', carla.Transform())
        self._spawn_sensor_with_backups('lane_invasion', 'LANE_INVASION', carla.Transform())
        print("All ego sensors spawned.")

        print("Spawning NPC vehicles...")
        if self.npc_vehicles:
            self.client.apply_batch([carla.command.DestroyActor(x) for x in self.npc_vehicles])
            self.npc_vehicles.clear()
        vehicle_blueprints = self.world.get_blueprint_library().filter('vehicle.*')
        spawn_points = self.world.get_map().get_spawn_points()
        for _ in range(min(NUM_NPC_VEHICLES, len(spawn_points) - 1)):
            if not spawn_points: break
            spawn_point = random.choice(spawn_points)
            spawn_points.remove(spawn_point)
            bp = random.choice(vehicle_blueprints)
            try:
                npc = self.world.try_spawn_actor(bp, spawn_point)
                if npc is not None: npc.set_autopilot(True); self.npc_vehicles.append(npc)
            except Exception: continue
        print(f"  - Spawned {len(self.npc_vehicles)} NPC vehicles for V2V.")

    def _tick_simulation(self):
        self.world.tick()
        snapshot = self.world.get_snapshot()
        self._process_fdir_commands()

        if self.safe_mode_locked and self.was_autopilot_on_lock:
            v = self.vehicle.get_velocity(); speed = np.linalg.norm([v.x, v.y, v.z])
            if speed > 0.1: self.apply_vehicle_control({'brake': 0.5})
            else: self.apply_vehicle_control({'hand_brake': True})

        self._process_sensor_data(snapshot)
        self._run_sensor_fusion()
        self._run_object_detection()
        self._send_udp_data(snapshot)
        display = self.display_manager.render()
        if self.show_hud: self.hud.render(display, self._get_simulation_state(snapshot))
        pygame.display.flip()

    def _process_fdir_commands(self):
        try:
            command = self.command_queue.get_nowait()
            cmd_type = command.get('command')

            if cmd_type == 'SET_VEHICLE_CONTROL':
                control_data = command.get('control', None)
                if control_data:
                    self.matlab_mode = 'AUTOPILOT' if control_data.get('throttle',0) > 0 or control_data.get('steer',0) != 0 else 'MANUAL'
                    self.apply_vehicle_control(control_data)
            
            elif cmd_type == 'ACTIVATE_BACKUP':
                sensor_group = command.get('sensor_group'); backup_index = command.get('backup_index')
                if sensor_group in self.active_sensor_indices:
                    self.active_sensor_indices[sensor_group] = backup_index
                    print(f"Activated backup #{backup_index} for sensor '{sensor_group}'")
                else: print(f"[WARN] Tried to activate backup for unknown sensor group: {sensor_group}")
            elif cmd_type == 'ENTER_SAFE_MODE':
                reason = command.get('reason', 'No reason specified')
                print(f"!!! ENTERING LOCKED SAFE MODE. Reason: {reason} !!!")
                self.was_autopilot_on_lock = (self.matlab_mode == 'AUTOPILOT')
                self.matlab_mode = 'LOCKED'
                self.safe_mode_locked = True
        except Empty: 
            return

    def _process_sensor_data(self, snapshot):
        if self.recording_images:
            for name, sensor_list in self.sensors.items():
                if name.startswith('cam_'):
                    active_idx = self.active_sensor_indices.get(name, 0)
                    if active_idx < len(sensor_list):
                        active_sensor = sensor_list[active_idx]
                        if active_sensor.surface: self.image_saver.save_image(os.path.join(CAMERA_DIRS[active_sensor.cname], f'frame_{snapshot.frame:06d}.png'), active_sensor.surface)
        for sensor_group in self.sensors.get('collision',[]):
            for event in sensor_group.get_events(): self.collision_count+=1; impulse=event.normal_impulse; intensity=np.linalg.norm([impulse.x,impulse.y,impulse.z]); self.csv_writers['collision'].writerow([time.time(), event.frame, intensity, event.other_actor.type_id])
        for sensor_group in self.sensors.get('lane_invasion',[]):
            for event in sensor_group.get_events(): self.lane_invasion_count+=1; lane_types=','.join([str(m.type) for m in event.crossed_lane_markings]); self.csv_writers['lane'].writerow([time.time(), event.frame, lane_types])

    def _run_sensor_fusion(self):
        gnss_sensor = self.sensors.get('gnss',[None])[self.active_sensor_indices.get('gnss', 0)]
        imu_sensor = self.sensors.get('imu',[None])[self.active_sensor_indices.get('imu', 0)]
        loc = self.vehicle.get_transform().location
        gps_pos = np.array([loc.x, loc.y]) if gnss_sensor and gnss_sensor.data else None
        imu_accel = np.array([imu_sensor.data.accelerometer.x, imu_sensor.data.accelerometer.y]) if imu_sensor and imu_sensor.data else None
        self.kalman_filter.update(gps_pos, imu_accel)
        self.fused_state = self.kalman_filter.get_fused_state()

    def _run_object_detection(self):
        lidar_sensor = self.sensors.get('lidar_roof', [None])[self.active_sensor_indices.get('lidar_roof', 0)]
        if lidar_sensor and lidar_sensor.data:
            self.object_tracker.update(lidar_sensor.data, self.vehicle.get_transform())
            self.detected_obstacles = self.object_tracker.get_obstacles()
        else: self.detected_obstacles = []
    
    def _gather_v2v_data(self) -> List[Dict[str, Any]]:
        v2v_list = []
        for actor in self.npc_vehicles:
            if actor and actor.is_alive:
                t = actor.get_transform(); v = actor.get_velocity()
                v2v_list.append({'id': actor.id, 'position': {'x': t.location.x, 'y': t.location.y, 'z': t.location.z}, 'velocity': {'x': v.x, 'y': v.y, 'z': v.z}})
        return v2v_list

    def _gather_v2i_data(self) -> Dict[str, Any]:
        v2i_data = {}
        if self.vehicle and self.vehicle.is_at_traffic_light():
            traffic_light = self.vehicle.get_traffic_light()
            if traffic_light: v2i_data['traffic_light_state'] = str(traffic_light.get_state()); v2i_data['traffic_light_id'] = traffic_light.id
        return v2i_data

    def _send_udp_data(self, snapshot):
        t=self.vehicle.get_transform(); v=self.vehicle.get_velocity()
        health_data = {name: [s.get_health_status(snapshot.frame) for s in s_list] for name, s_list in self.sensors.items()}
        data_packet = {
            'timestamp':snapshot.timestamp.elapsed_seconds, 'frame':snapshot.frame,
            'mode': self.matlab_mode,
            'map': self.world.get_map().name.split('/')[-1],
            'weather': self.weather_names[self.current_weather_index],
            'speed':np.linalg.norm([v.x,v.y,v.z])*3.6, 'position':{'x':t.location.x,'y':t.location.y,'z':t.location.z},
            'rotation':{'pitch':t.rotation.pitch,'yaw':t.rotation.yaw,'roll':t.rotation.roll},
            'control':self.controller.get_control_state(), 'fused_state':self.fused_state,
            'ekf_covariance':self.kalman_filter.kf.P.tolist() if self.kalman_filter else None,
            'sensor_health': health_data, 'collisions': self.collision_count, 'lane_invasions': self.lane_invasion_count,
            'Detected_Obstacles': self.detected_obstacles, 'V2V_Data': self._gather_v2v_data(), 'V2I_Data': self._gather_v2i_data(),
        }
        for name, sensor_list in self.sensors.items():
            active_sensor = sensor_list[self.active_sensor_indices.get(name, 0)]
            if active_sensor.data is None or active_sensor.stype in ['COLLISION','LANE_INVASION']: continue
            data=active_sensor.data; key_name=name; processed_data=None
            if active_sensor.stype == 'RGB': key_name=f"image_{active_sensor.cname}"; _,jpeg=cv2.imencode('.jpg',data); processed_data=base64.b64encode(jpeg).decode('utf-8')
            elif active_sensor.stype == 'LIDAR': processed_data=base64.b64encode(data.raw_data).decode('utf-8')
            elif active_sensor.stype == 'IMU': processed_data=base64.b64encode(struct.pack('<ffffff',data.accelerometer.x,data.accelerometer.y,data.accelerometer.z,data.gyroscope.x,data.gyroscope.y,data.gyroscope.z)).decode('utf-8')
            elif active_sensor.stype == 'GNSS': processed_data={'lat':data.latitude,'lon':data.longitude,'alt':data.altitude}
            elif active_sensor.stype == 'RADAR': processed_data=[{'alt':d.altitude,'az':d.azimuth,'depth':d.depth,'vel':d.velocity} for d in data]
            elif active_sensor.stype == 'ULTRASONIC': processed_data=data.distance
            if processed_data is not None: data_packet[key_name] = processed_data
        self.udp_sender.send_string_chunks(json.dumps(data_packet,separators=(',',':')), snapshot.frame)

    def _cleanup(self):
        print("Cleaning up resources...")
        if self.command_receiver: self.command_receiver.stop(); self.command_receiver.join()
        if self.image_saver: self.image_saver.stop()
        if self.udp_sender: self.udp_sender.close()
        for f in self.log_files.values(): f.close()

        if self.world:
            print("Resetting CARLA server settings...")
            settings = self.world.get_settings(); settings.synchronous_mode = False; settings.fixed_delta_seconds = None
            try: self.world.apply_settings(settings)
            except RuntimeError as e: print(f"[WARN] Could not apply settings during cleanup: {e}")

        if self.client:
            print("Destroying actors...")
            actor_ids = []
            if self.npc_vehicles: actor_ids.extend([x.id for x in self.npc_vehicles if x.is_alive])
            for sensor_list in self.sensors.values(): actor_ids.extend([s.sensor.id for s in sensor_list if s.sensor and s.sensor.is_alive])
            if self.vehicle and self.vehicle.is_alive: actor_ids.append(self.vehicle.id)
            if actor_ids:
                try: self.client.apply_batch_sync([carla.command.DestroyActor(aid) for aid in actor_ids], True); print(f"  - Destroyed {len(actor_ids)} actors.")
                except RuntimeError as e: print(f"[WARN] Could not destroy all actors during cleanup: {e}")
        pygame.quit()
        print("Simulation ended.")

    def apply_vehicle_control(self, control: dict):
        if self.vehicle and self.vehicle.is_alive:
            if self.safe_mode_locked and self.was_autopilot_on_lock:
                safe_control = carla.VehicleControl(throttle=0.0, brake=float(control.get('brake', 0.5)), steer=0.0)
                self.vehicle.apply_control(safe_control)
                return
            
            vehicle_control = carla.VehicleControl(
                throttle=float(control.get('throttle', 0.0)),
                steer=float(control.get('steer', 0.0)),
                brake=float(control.get('brake', 0.0)),
                hand_brake=bool(control.get('hand_brake', False)),
                reverse=bool(control.get('reverse', False))
            )
            self.vehicle.apply_control(vehicle_control)

    def toggle_image_recording(self): self.recording_images = not self.recording_images
    def toggle_sim_recording(self):
        self.recording_sim=not self.recording_sim
        if self.recording_sim: self.client.start_recorder(os.path.join(LOG_DIR,f"sim_rec_{time.time()}.log"))
        else: self.client.stop_recorder()
    
    def change_weather(self, d):
        self.current_weather_index = (self.current_weather_index + d) % len(self.weather_names)
        self.world.set_weather(WEATHER_PRESETS[self.weather_names[self.current_weather_index]])

    def change_map_layer(self, d): self.current_layer_index=(self.current_layer_index+d)%len(MAP_LAYERS)
    def toggle_current_map_layer(self, unload):
        if unload: self.world.unload_map_layer(MAP_LAYERS[self.current_layer_index])
        else: self.world.load_map_layer(MAP_LAYERS[self.current_layer_index])
    def toggle_hud(self): self.show_hud = not self.show_hud
    def toggle_help(self): self.hud.show_help = not self.hud.show_help
    
    def change_map(self, map_name):
        if self.world.get_map().name.endswith(map_name): return
        if self.client and self.npc_vehicles: self.client.apply_batch_sync([carla.command.DestroyActor(x) for x in self.npc_vehicles if x.is_alive], True); self.npc_vehicles.clear()
        [s.destroy() for s_list in self.sensors.values() for s in s_list]; self.sensors.clear()
        if self.vehicle: self.vehicle.destroy(); self.vehicle=None
        self.world = self.client.load_world(map_name)
        settings=self.world.get_settings(); settings.synchronous_mode=True; settings.fixed_delta_seconds=1.0/TICK_RATE
        self.world.apply_settings(settings); self._setup_actors_and_sensors()
    
    def reset_safe_mode_lock(self):
        if self.safe_mode_locked:
            print("[INFO] User override: Safe Mode unlocked. All sensors reset to primary.")
            self.safe_mode_locked = False
            self.was_autopilot_on_lock = False
            for key in self.active_sensor_indices: self.active_sensor_indices[key] = 0
        else: print("[INFO] Safe mode is not currently locked.")

    def _get_simulation_state(self, snapshot):
        v = self.vehicle.get_velocity()
        health_data = {name: [s.get_health_status(snapshot.frame) for s in s_list] for name, s_list in self.sensors.items()}
        return {
            'speed':np.linalg.norm([v.x,v.y,v.z])*3.6, 'map_name':self.world.get_map().name.split('/')[-1],
            'collision_count':self.collision_count, 'lane_invasion_count':self.lane_invasion_count,
            'control':self.controller.get_control_state(), 'matlab_mode': self.matlab_mode,
            'recording_images':self.recording_images, 'recording_sim':self.recording_sim,
            'fused_pos':f"({self.fused_state['x']:.1f},{self.fused_state['y']:.1f})" if self.fused_state else "N/A",
            'sensor_health': health_data,
            'safe_mode_locked': self.safe_mode_locked,
            'active_sensors': self.active_sensor_indices
        }

if __name__ == '__main__':
    try:
        CarlaSimulation().run()
    except Exception as e:
        print(f"\nAn unhandled error occurred: {e}\n{traceback.format_exc()}")