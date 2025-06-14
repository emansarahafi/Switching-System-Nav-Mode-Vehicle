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
import binascii
from queue import Queue, Empty
from typing import Dict, Any, List, Tuple, Optional

import pygame
from pygame.locals import *
import numpy as np
import carla
import cv2

# Sensor Fusion imports
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise

# ==============================================================================
# -- Constants -----------------------------------------------------------------
# ==============================================================================
# Display and Camera Settings
IMAGE_WIDTH = 320
IMAGE_HEIGHT = 240
GRID_COLS = 3
GRID_ROWS = 2
WINDOW_WIDTH = IMAGE_WIDTH * GRID_COLS
WINDOW_HEIGHT = IMAGE_HEIGHT * GRID_ROWS
TICK_RATE = 20  # Hz

# Data and Logging
DATA_DIR = 'data'
IMAGE_DIR = os.path.join(DATA_DIR, 'images')
LOG_DIR = os.path.join(DATA_DIR, 'logs')
CAMERA_DIRS = {
    'front': os.path.join(IMAGE_DIR, 'front'), 'back': os.path.join(IMAGE_DIR, 'back'),
    'left': os.path.join(IMAGE_DIR, 'left'), 'right': os.path.join(IMAGE_DIR, 'right'),
    'interior': os.path.join(IMAGE_DIR, 'interior')
}

# Vehicle and Sensor Settings
VEHICLE_MODEL = 'vehicle.tesla.model3'
NUM_BACKUPS = 2 # Each primary sensor will have 2 backups

# Weather and Map Presets
WEATHER_PRESETS = [carla.WeatherParameters.ClearNoon, carla.WeatherParameters.CloudyNoon, carla.WeatherParameters.WetNoon, carla.WeatherParameters.HardRainNoon, carla.WeatherParameters.ClearSunset, carla.WeatherParameters.WetSunset, carla.WeatherParameters.HardRainSunset]
MAP_LAYERS = [carla.MapLayer.NONE, carla.MapLayer.Buildings, carla.MapLayer.Decals, carla.MapLayer.Foliage, carla.MapLayer.Ground, carla.MapLayer.ParkedVehicles, carla.MapLayer.Particles, carla.MapLayer.Props, carla.MapLayer.StreetLights, carla.MapLayer.Walls, carla.MapLayer.All]

# ==============================================================================
# -- ImageSaver Class (Performance Optimization) -------------------------------
# ==============================================================================
class ImageSaver(threading.Thread):
    """A threaded class to save images to disk without blocking the main loop."""
    def __init__(self):
        super().__init__()
        self.daemon = True
        self.queue = Queue()
        self.running = True

    def run(self):
        while self.running:
            try:
                filepath, surface = self.queue.get(timeout=1)
                pygame.image.save(surface, filepath)
                self.queue.task_done()
            except Empty:
                continue

    def save_image(self, filepath: str, surface: pygame.Surface):
        self.queue.put((filepath, surface))

    def stop(self):
        print("Image saver: Waiting for queue to empty...")
        self.queue.join()
        self.running = False
        print("Image saver: Stopped.")

# ==============================================================================
# -- KalmanFilterSystem Class --------------------------------------------------
# ==============================================================================
class KalmanFilterSystem:
    """Fuses GPS and IMU data to provide a robust estimate of the vehicle's state."""
    def __init__(self, dt):
        self.dt = dt
        self.kf = KalmanFilter(dim_x=6, dim_z=2)
        self.kf.F = np.array([[1,0,dt,0,0.5*dt**2,0],[0,1,0,dt,0,0.5*dt**2],[0,0,1,0,dt,0],[0,0,0,1,0,dt],[0,0,0,0,1,0],[0,0,0,0,0,1]])
        self.kf.H = np.array([[1,0,0,0,0,0],[0,1,0,0,0,0]])
        self.kf.P *= 1000.
        self.kf.R = np.diag([0.5**2, 0.5**2])
        accel_noise_var = 0.1
        q_entry_1, q_entry_2, q_entry_3 = (dt**4)/4, (dt**3)/2, (dt**2)
        self.kf.Q = np.zeros((6, 6))
        self.kf.Q[0,0], self.kf.Q[1,1] = q_entry_1, q_entry_1
        self.kf.Q[0,2], self.kf.Q[2,0] = q_entry_2, q_entry_2
        self.kf.Q[1,3], self.kf.Q[3,1] = q_entry_2, q_entry_2
        self.kf.Q[2,2], self.kf.Q[3,3] = q_entry_3, q_entry_3
        self.kf.Q[4,4], self.kf.Q[5,5] = accel_noise_var*dt, accel_noise_var*dt
        self.kf.Q *= accel_noise_var
        self.initialized = False
        print("Kalman Filter system initialized.")

    def initialize(self, initial_pos):
        self.kf.x[:2, 0] = initial_pos
        self.initialized = True

    def update(self, gps_pos, imu_accel):
        if not self.initialized and gps_pos is not None:
            self.initialize(gps_pos)
            return
        self.kf.predict()
        if gps_pos is not None: self.kf.update(gps_pos)
        if imu_accel is not None: self.kf.x[4,0], self.kf.x[5,0] = imu_accel[0], imu_accel[1]

    def get_fused_state(self) -> Optional[Dict[str, float]]:
        if not self.initialized: return None
        return {'x': float(self.kf.x[0,0]), 'y': float(self.kf.x[1,0]), 'vx': float(self.kf.x[2,0]), 'vy': float(self.kf.x[3,0])}

# ==============================================================================
# -- HUD Class -----------------------------------------------------------------
# ==============================================================================
class HUD:
    """Renders the Heads-Up Display, including sensor health."""
    def __init__(self, width: int, height: int):
        self.dim = (width, height)
        self.font = pygame.font.SysFont('Arial', 14)
        self.help_font = pygame.font.SysFont('Monospace', 14)
        self.show_help = False
        self.HEALTH_COLORS = {"OK": (0, 255, 0), "NO_DATA": (255, 255, 0), "DEAD": (255, 0, 0)}

    def render(self, display: pygame.Surface, sim_state: Dict[str, Any]):
        self._render_hud_info(display, sim_state)
        self._render_sensor_health(display, sim_state.get('sensor_health', {}))
        if self.show_help:
            self._render_help_text(display)

    def _render_hud_info(self, display: pygame.Surface, sim_state: Dict[str, Any]):
        info_text = [
            f"Speed: {sim_state['speed']:.2f} km/h", f"Map: {sim_state['map_name']}",
            f"Fused Pos: {sim_state.get('fused_pos', 'N/A')}",
            f"Collisions: {sim_state['collision_count']}", f"Lane Invasions: {sim_state['lane_invasion_count']}",
            "-----------", f"Throttle: {sim_state['control']['throttle']:.2f}",
            f"Steer: {sim_state['control']['steer']:.2f}", f"Brake: {sim_state['control']['brake']:.2f}",
            "-----------", f"Autopilot: {'ON' if sim_state['autopilot'] else 'OFF'}",
            f"Img Record: {'ON' if sim_state['recording_images'] else 'OFF'}",
            f"Sim Record: {'ON' if sim_state['recording_sim'] else 'OFF'}"
        ]
        y_offset = 10
        for text in info_text:
            text_surface = self.font.render(text, True, (255, 255, 255))
            display.blit(text_surface, (10, y_offset))
            y_offset += 20
    
    def _render_sensor_health(self, display: pygame.Surface, health_data: Dict[str, List[str]]):
        x_offset = WINDOW_WIDTH - 280
        y_offset = 10
        title_surface = self.font.render("--- Sensor Health ---", True, (255, 255, 255))
        display.blit(title_surface, (x_offset, y_offset))
        y_offset += 20

        for name, statuses in health_data.items():
            name_surface = self.font.render(f"{name}: ", True, (255, 255, 255))
            display.blit(name_surface, (x_offset, y_offset))
            current_x = x_offset + name_surface.get_width()

            for i, status in enumerate(statuses):
                color = self.HEALTH_COLORS.get(status, (200, 200, 200))
                status_surface = self.font.render(status, True, color)
                display.blit(status_surface, (current_x, y_offset))
                current_x += status_surface.get_width()

                if i < len(statuses) - 1:
                    sep_surface = self.font.render(" | ", True, (255, 255, 255))
                    display.blit(sep_surface, (current_x, y_offset))
                    current_x += sep_surface.get_width()
            y_offset += 20

    def _render_help_text(self, display: pygame.Surface):
        help_text = [
            "W/S: Throttle/Brake", "A/D: Steer Left/Right", "Q: Toggle Reverse", "Space: Hand Brake",
            "P: Toggle Autopilot", "C: Next Weather (Shift+C: Prev)", "V: Next Map Layer (Shift+V: Prev)",
            "B: Load Layer (Shift+B: Unload)", "R: Toggle Image Recording", "Ctrl+R: Toggle Sim Recording",
            "3: Switch to Town03", "0: Switch to Town10HD", "F1: Toggle HUD", "H: Toggle Help", "ESC: Quit"
        ]
        s = pygame.Surface((300, len(help_text) * 22 + 20)); s.set_alpha(200); s.fill((0, 0, 0))
        display.blit(s, (50, 50))
        y_offset = 60
        for text in help_text:
            text_surface = self.help_font.render(text, True, (255, 255, 255))
            display.blit(text_surface, (60, y_offset))
            y_offset += 22

# ==============================================================================
# -- Controller, Sensor, Display, and Networking Classes -----------------------
# ==============================================================================
class KeyboardController:
    """Handles keyboard input for vehicle control and simulation commands."""
    def __init__(self, simulation):
        self.simulation = simulation
        self._control_state = {'throttle': 0.0, 'steer': 0.0, 'brake': 0.0, 'hand_brake': False, 'reverse': False}

    def parse_events(self) -> bool:
        for event in pygame.event.get():
            if event.type == pygame.QUIT: return False
            elif event.type in (pygame.KEYDOWN, pygame.KEYUP): self._parse_key_event(event)
        if not self.simulation.autopilot:
            self.simulation.apply_vehicle_control(self._control_state)
        return True

    def get_control_state(self) -> Dict[str, Any]:
        return self._control_state

    def _parse_key_event(self, event: pygame.event.Event):
        is_keydown = (event.type == pygame.KEYDOWN)
        key = event.key
        mods = pygame.key.get_mods()
        is_ctrl = (mods & pygame.KMOD_CTRL)
        is_shift = (mods & pygame.KMOD_SHIFT)

        if key == K_w: self._control_state['throttle'] = 1.0 if is_keydown else 0.0
        elif key == K_s: self._control_state['brake'] = 1.0 if is_keydown else 0.0
        elif key == K_a: self._control_state['steer'] = -0.7 if is_keydown else 0.0
        elif key == K_d: self._control_state['steer'] = 0.7 if is_keydown else 0.0
        elif key == K_q and is_keydown: self._control_state['reverse'] = not self._control_state['reverse']
        elif key == K_SPACE: self._control_state['hand_brake'] = is_keydown
        if not is_keydown: return
        if key == K_p and not is_ctrl: self.simulation.toggle_autopilot()
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
    """Manages a CARLA sensor, its data, visualization, and health status."""
    def __init__(self, world, display_manager, sensor_type, transform, attached_vehicle, attributes=None, display_pos=None, camera_name=''):
        self.sensor_type = sensor_type; self.camera_name = camera_name
        self.surface = None; self.data = None
        self.event_list = []; self.lock = threading.Lock()
        self.last_updated_frame = 0
        self.sensor = self._create_sensor(world, sensor_type, transform, attached_vehicle, attributes)
        self._setup_callback()
        if display_manager and display_pos:
            display_manager.attach_sensor(self, display_pos)

    def get_health_status(self, current_frame: int) -> str:
        if not self.sensor or not self.sensor.is_alive:
            return "DEAD"
        if self.sensor_type in ['COLLISION', 'LANE_INVASION']:
            return "OK"
        if current_frame - self.last_updated_frame > TICK_RATE:
            return "NO_DATA"
        return "OK"

    def _update_timestamp(self, data):
        self.last_updated_frame = data.frame
        self.data = data

    def _create_sensor(self, world, sensor_type, transform, attached_vehicle, attributes):
        bp_map={'RGB':'sensor.camera.rgb','LIDAR':'sensor.lidar.ray_cast','RADAR':'sensor.other.radar','ULTRASONIC':'sensor.other.obstacle','IMU':'sensor.other.imu','GNSS':'sensor.other.gnss','COLLISION':'sensor.other.collision','LANE_INVASION':'sensor.other.lane_invasion'}
        bp = world.get_blueprint_library().find(bp_map[sensor_type])
        if sensor_type == 'RGB':
            bp.set_attribute('image_size_x', str(IMAGE_WIDTH)); bp.set_attribute('image_size_y', str(IMAGE_HEIGHT))
        if attributes:
            for key, value in attributes.items():
                if bp.has_attribute(key): bp.set_attribute(key, str(value))
        return world.spawn_actor(bp, transform, attach_to=attached_vehicle)

    def _setup_callback(self):
        callbacks = {'RGB': self._parse_image, 'RADAR': self._parse_radar, 'ULTRASONIC': self._parse_obstacle, 'COLLISION': self._parse_event, 'LANE_INVASION': self._parse_event}
        self.sensor.listen(callbacks.get(self.sensor_type, self._update_timestamp))

    def _parse_image(self, image: carla.Image):
        self.last_updated_frame = image.frame
        array = np.frombuffer(image.raw_data, dtype=np.uint8).reshape((image.height, image.width, 4))[:, :, :3][:, :, ::-1]
        self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        self.data = array

    def _parse_radar(self, radar_data: carla.RadarMeasurement):
        self.last_updated_frame = radar_data.frame
        self.data = [{'alt':d.altitude,'az':d.azimuth,'depth':d.depth,'vel':d.velocity} for d in radar_data]

    def _parse_obstacle(self, event: carla.ObstacleDetectionEvent):
        self.last_updated_frame = event.frame
        self.data = event.distance

    def _parse_event(self, event):
        with self.lock:
            self.event_list.append(event)

    def get_events(self) -> list:
        with self.lock:
            events = self.event_list[:]; self.event_list.clear()
            return events

    def destroy(self):
        if self.sensor and self.sensor.is_alive:
            self.sensor.destroy()

class DisplayManager:
    """Manages the Pygame display and sensor surface rendering."""
    def __init__(self, grid_size: Tuple[int, int]):
        self.display = pygame.display.set_mode((grid_size[0] * IMAGE_WIDTH, grid_size[1] * IMAGE_HEIGHT), pygame.RESIZABLE)
        pygame.display.set_caption("CARLA Sensor Dashboard")
        self.sensors: List[Tuple[SensorManager, Tuple[int, int]]] = []

    def attach_sensor(self, sensor_manager: SensorManager, position: Tuple[int, int]):
        self.sensors.append((sensor_manager, position))

    def render(self):
        self.display.fill((0, 0, 0))
        for sensor, pos in self.sensors:
            if sensor.surface:
                self.display.blit(sensor.surface, (pos[0] * IMAGE_WIDTH, pos[1] * IMAGE_HEIGHT))
        return self.display

class UDPDataSender:
    """Handles data chunking and sending over UDP without compression."""
    def __init__(self, ip: str = '127.0.0.1', port: int = 10000, chunk_size: int = 60000):
        self.ip = ip; self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.max_packet_size = chunk_size
        json_wrapper_overhead = 200
        # The chunk size for the data must account for base64's overhead (approx 4/3 size)
        self.max_data_chunk_size = int((self.max_packet_size - json_wrapper_overhead) * 3 / 4)
        print(f"UDP sender initialized. Target packet size: {self.max_packet_size}. Max raw data per chunk: {self.max_data_chunk_size} bytes.")

    def send(self, data: Dict[str, Any]):
        try:
            json_str = json.dumps(data, ensure_ascii=False, separators=(',', ':'))
            data['_crc32'] = binascii.crc32(json_str.encode('utf-8'))
            
            final_json_str = json.dumps(data, ensure_ascii=False, separators=(',', ':'))
            # The payload is now just the encoded JSON string. No compression.
            full_payload_bytes = final_json_str.encode('utf-8')
            
            frame_id = data.get('frame', -1)
            chunks = [full_payload_bytes[i:i + self.max_data_chunk_size] for i in range(0, len(full_payload_bytes), self.max_data_chunk_size)]
            
            if not chunks: chunks.append(b'')
            
            for i, chunk in enumerate(chunks):
                # Data is still base64 encoded for safe transport within the JSON wrapper.
                packet = {'frame': frame_id, 'chunk': i, 'total_chunks': len(chunks), 'data': base64.b64encode(chunk).decode('utf-8')}
                self.sock.sendto(json.dumps(packet).encode('utf-8'), (self.ip, self.port))
        except Exception as e:
            print(f"UDP send error: {e}\n{traceback.format_exc()}")

    def close(self): self.sock.close()

# ==============================================================================
# -- CarlaSimulation Class -----------------------------------------------------
# ==============================================================================
class CarlaSimulation:
    """The main class that orchestrates the entire simulation."""
    def __init__(self):
        self.client = None; self.world = None; self.vehicle = None
        self.display_manager = None; self.hud = None; self.controller = None
        self.udp_sender = None; self.image_saver = None
        self.log_files = {}; self.csv_writers = {}
        self.sensors: Dict[str, List[SensorManager]] = {}
        self.kalman_filter: Optional[KalmanFilterSystem] = None
        self.fused_state: Optional[Dict[str, float]] = None
        self.autopilot = False; self.recording_images = False; self.recording_sim = False
        self.show_hud = True; self.collision_count = 0; self.lane_invasion_count = 0
        self.current_weather_index = 0; self.current_layer_index = 0
        self.running = True

    def run(self):
        try:
            self._initialize()
            clock = pygame.time.Clock()
            while self.running:
                clock.tick(TICK_RATE)
                if not self.controller.parse_events(): break
                self._tick_simulation()
        except Exception as e:
            print(f"\nAn error occurred: {e}", file=sys.stderr)
            traceback.print_exc()
        finally:
            self._cleanup()

    def _initialize(self):
        pygame.init(); pygame.font.init()
        self._setup_directories_and_logging()
        self.client = carla.Client('localhost', 2000); self.client.set_timeout(20.0)
        self.world = self.client.load_world('Town03')
        settings = self.world.get_settings(); settings.synchronous_mode = True; settings.fixed_delta_seconds = 1.0 / TICK_RATE
        self.world.apply_settings(settings)
        self.display_manager = DisplayManager(grid_size=(GRID_COLS, GRID_ROWS))
        self.hud = HUD(WINDOW_WIDTH, WINDOW_HEIGHT)
        self.controller = KeyboardController(self)
        # --- THE FIX: Use a smaller, safer chunk size to avoid OS buffer errors ---
        self.udp_sender = UDPDataSender(chunk_size=32768)  # Using 32KB packets
        self.image_saver = ImageSaver(); self.image_saver.start()
        self._setup_actors_and_sensors()
        self.kalman_filter = KalmanFilterSystem(dt=1.0/TICK_RATE)
        self.change_weather(0)

    def _setup_directories_and_logging(self):
        os.makedirs(DATA_DIR, exist_ok=True); os.makedirs(LOG_DIR, exist_ok=True)
        for dir_path in CAMERA_DIRS.values(): os.makedirs(dir_path, exist_ok=True)
        log_paths={'collision':os.path.join(LOG_DIR,'collision_log.csv'), 'lane':os.path.join(LOG_DIR,'lane_invasion_log.csv')}
        headers={'collision':['ts','frame','intensity','actor_type','actor_id'], 'lane':['ts','frame','lane_types']}
        for name, path in log_paths.items():
            self.log_files[name] = open(path, 'w', newline=''); self.csv_writers[name] = csv.writer(self.log_files[name]); self.csv_writers[name].writerow(headers[name])
    
    def _spawn_sensor_with_backups(self, name: str, sensor_type: str, transform: carla.Transform,
                                   attributes: Dict = None, display_pos: Optional[Tuple[int, int]] = None,
                                   camera_name: str = ''):
        sensor_list = []
        primary_sensor = SensorManager(self.world, self.display_manager, sensor_type, transform, self.vehicle, attributes, display_pos, camera_name if camera_name else name)
        sensor_list.append(primary_sensor)
        for i in range(NUM_BACKUPS):
            loc = transform.location; offset = (i + 1) * 0.02
            perturbed_transform = carla.Transform(carla.Location(loc.x + offset, loc.y + offset, loc.z), transform.rotation)
            backup_sensor = SensorManager(self.world, None, sensor_type, perturbed_transform, self.vehicle, attributes, None, camera_name)
            sensor_list.append(backup_sensor)
        self.sensors[name] = sensor_list
        print(f"  - Spawned {name} with {NUM_BACKUPS} backups.")

    def _setup_actors_and_sensors(self):
        vehicle_bp = self.world.get_blueprint_library().find(VEHICLE_MODEL)
        spawn_point = random.choice(self.world.get_map().get_spawn_points())
        self.vehicle = self.world.spawn_actor(vehicle_bp, spawn_point)
        print(f"Spawned vehicle {self.vehicle.type_id}")
        
        print("Spawning Cameras with backups...")
        self._spawn_sensor_with_backups('cam_front', 'RGB', carla.Transform(carla.Location(x=1.5, z=2.4)), display_pos=(0, 0), camera_name='front')
        self._spawn_sensor_with_backups('cam_rear', 'RGB', carla.Transform(carla.Location(x=-2.0, z=0.8), carla.Rotation(yaw=180)), display_pos=(1, 0), camera_name='back')
        self._spawn_sensor_with_backups('cam_left', 'RGB', carla.Transform(carla.Location(x=1.3, y=-0.9, z=1.2), carla.Rotation(yaw=-110)), display_pos=(0, 1), camera_name='left')
        self._spawn_sensor_with_backups('cam_right', 'RGB', carla.Transform(carla.Location(x=1.3, y=0.9, z=1.2), carla.Rotation(yaw=110)), display_pos=(1, 1), camera_name='right')
        self._spawn_sensor_with_backups('cam_interior', 'RGB', carla.Transform(carla.Location(x=0.5, y=-0.3, z=1.4), carla.Rotation(pitch=-15, yaw=180)), display_pos=(2, 0), camera_name='interior')
        
        print("Spawning LiDARs with backups...")
        self._spawn_sensor_with_backups('lidar', 'LIDAR', carla.Transform(carla.Location(z=2.5)), attributes={'range': '100'})
        
        print("Spawning GPS, IMU, Radar, and Ultrasonic sensors with backups...")
        self._spawn_sensor_with_backups('gnss', 'GNSS', carla.Transform(carla.Location(x=-0.5, z=2.6)))
        self._spawn_sensor_with_backups('imu', 'IMU', carla.Transform(carla.Location(x=0, z=0.5)))
        self._spawn_sensor_with_backups('radar_front', 'RADAR', carla.Transform(carla.Location(x=2.2, z=0.5)), attributes={'range': '150', 'fov': '30'})
        self._spawn_sensor_with_backups('radar_rear', 'RADAR', carla.Transform(carla.Location(x=-2.2, z=0.5), carla.Rotation(yaw=180)), attributes={'range': '80', 'fov': '90'})
        self._spawn_sensor_with_backups('ultrasonic', 'ULTRASONIC', carla.Transform(carla.Location(x=2.3, z=0.6)))

        print("Spawning event-based sensors...")
        self._spawn_sensor_with_backups('collision', 'COLLISION', carla.Transform())
        self._spawn_sensor_with_backups('lane_inv', 'LANE_INVASION', carla.Transform())

    def _tick_simulation(self):
        self.world.tick()
        snapshot = self.world.get_snapshot()

        self._run_sensor_fusion()
        self._process_sensor_data(snapshot)
        self._send_udp_data(snapshot)
        
        display = self.display_manager.render()
        if self.show_hud:
            sim_state = self._get_simulation_state(snapshot.frame)
            self.hud.render(display, sim_state)
        pygame.display.flip()

    def _run_sensor_fusion(self):
        gnss_sensor = self.sensors.get('gnss', [None])[0]
        imu_sensor = self.sensors.get('imu', [None])[0]
        gps_pos, imu_accel = None, None
        if gnss_sensor and gnss_sensor.data: gps_pos = np.array([self.vehicle.get_transform().location.x, self.vehicle.get_transform().location.y])
        if imu_sensor and imu_sensor.data: acc = imu_sensor.data.accelerometer; imu_accel = np.array([acc.x, acc.y])
        self.kalman_filter.update(gps_pos, imu_accel)
        self.fused_state = self.kalman_filter.get_fused_state()

    def _process_sensor_data(self, snapshot: carla.WorldSnapshot):
        current_frame = snapshot.frame
        if self.recording_images:
            for sensor_list in self.sensors.values():
                if sensor_list[0].sensor_type == 'RGB':
                    for i, sensor in enumerate(sensor_list):
                        if sensor.surface:
                            filepath = os.path.join(CAMERA_DIRS[sensor.camera_name], f'frame_{current_frame:06d}_{i}.png')
                            self.image_saver.save_image(filepath, sensor.surface)
        
        for sensor in self.sensors.get('collision', []):
            for event in sensor.get_events():
                self.collision_count += 1
                intensity = np.linalg.norm([event.normal_impulse.x, event.normal_impulse.y, event.normal_impulse.z])
                self.csv_writers['collision'].writerow([time.time(), event.frame, intensity, event.other_actor.type_id, event.other_actor.id])
        
        for sensor in self.sensors.get('lane_inv', []):
            for event in sensor.get_events():
                self.lane_invasion_count += 1
                lane_types = ','.join([str(m.type) for m in event.crossed_lane_markings])
                self.csv_writers['lane'].writerow([time.time(), event.frame, lane_types])

    def _send_udp_data(self, snapshot: carla.WorldSnapshot):
        t = self.vehicle.get_transform()
        v = self.vehicle.get_velocity()
        
        sensor_health_data = {name: [s.get_health_status(snapshot.frame) for s in s_list] for name, s_list in self.sensors.items()}

        data_packet = {
            'timestamp': snapshot.timestamp.elapsed_seconds, 'frame': snapshot.frame,
            'speed': float(np.linalg.norm([v.x, v.y, v.z]) * 3.6),
            'position_raw': {'x': t.location.x, 'y': t.location.y, 'z': t.location.z},
            'rotation': {'pitch': t.rotation.pitch, 'yaw': t.rotation.yaw, 'roll': t.rotation.roll},
            'control': self.controller.get_control_state(), 'collisions': self.collision_count,
            'lane_invasions': self.lane_invasion_count, 'environment': self._get_environmental_data(),
            'fused_state': self.fused_state,
            'sensor_health': sensor_health_data
        }
        
        for name, sensor_list in self.sensors.items():
            if sensor_list[0].sensor_type in ['COLLISION', 'LANE_INVASION']: 
                continue

            primary_sensor = sensor_list[0]
            if primary_sensor.data is None:
                continue

            data = primary_sensor.data
            processed_data = None

            if primary_sensor.sensor_type == 'RGB':
                _, jpeg_img = cv2.imencode('.jpg', data)
                processed_data = base64.b64encode(jpeg_img).decode('utf-8')
            elif primary_sensor.sensor_type == 'LIDAR': 
                processed_data = base64.b64encode(data.raw_data).decode('utf-8')
            elif primary_sensor.sensor_type == 'IMU': 
                processed_data = {'accelerometer': {'x': data.accelerometer.x, 'y': data.accelerometer.y, 'z': data.accelerometer.z}, 'gyroscope': {'x': data.gyroscope.x, 'y': data.gyroscope.y, 'z': data.gyroscope.z}, 'compass': data.compass}
            elif primary_sensor.sensor_type == 'GNSS': 
                processed_data = {'latitude': data.latitude, 'longitude': data.longitude, 'altitude': data.altitude}
            else: 
                processed_data = data
            
            if processed_data is not None:
                data_packet[name] = processed_data

        debug_packet = data_packet.copy()
        for key, value in debug_packet.items():
            if isinstance(value, str) and len(value) > 100:
                debug_packet[key] = f"<base64 data len:{len(value)}>"

        print("--- Sending JSON Packet (Frame {}) ---".format(data_packet.get('frame', 'N/A')))
        print(json.dumps(debug_packet, indent=4))
        print("--------------------------------------")

        self.udp_sender.send(data_packet)

    def _cleanup(self):
        print("Cleaning up resources...")
        if self.image_saver: self.image_saver.stop(); self.image_saver.join()
        if self.udp_sender: self.udp_sender.close()
        for f in self.log_files.values(): f.close()
        if self.world:
            settings = self.world.get_settings(); settings.synchronous_mode = False; settings.fixed_delta_seconds = None
            self.world.apply_settings(settings)
            self._cleanup_actors()
        pygame.quit()
        print("Simulation ended.")

    def _cleanup_actors(self):
        for sensor_list in self.sensors.values():
            for sensor in sensor_list: sensor.destroy()
        self.sensors.clear()
        if self.vehicle and self.vehicle.is_alive: self.vehicle.destroy()
        self.vehicle = None

    def apply_vehicle_control(self, control_state:Dict[str,Any]): self.vehicle.apply_control(carla.VehicleControl(**control_state))
    def toggle_autopilot(self): self.autopilot = not self.autopilot; self.vehicle.set_autopilot(self.autopilot)
    def toggle_image_recording(self): self.recording_images = not self.recording_images
    def toggle_sim_recording(self):
        self.recording_sim = not self.recording_sim
        if self.recording_sim: self.client.start_recorder(os.path.join(LOG_DIR, f"sim_rec_{time.time()}.log"))
        else: self.client.stop_recorder()
    def change_weather(self, d: int): self.current_weather_index = (self.current_weather_index + d) % len(WEATHER_PRESETS); self.world.set_weather(WEATHER_PRESETS[self.current_weather_index])
    def change_map_layer(self, d: int): self.current_layer_index = (self.current_layer_index + d) % len(MAP_LAYERS)
    def toggle_current_map_layer(self, unload: bool): self.world.unload_map_layer(MAP_LAYERS[self.current_layer_index]) if unload else self.world.load_map_layer(MAP_LAYERS[self.current_layer_index])
    def toggle_hud(self): self.show_hud = not self.show_hud
    def toggle_help(self): self.hud.show_help = not self.hud.show_help
    def change_map(self, map_name: str):
        if self.world.get_map().name.endswith(map_name): return
        self._cleanup_actors()
        self.world = self.client.load_world(map_name)
        settings = self.world.get_settings(); settings.synchronous_mode = True; settings.fixed_delta_seconds = 1.0 / TICK_RATE
        self.world.apply_settings(settings)
        self._setup_actors_and_sensors()

    def _get_simulation_state(self, current_frame: int) -> Dict[str, Any]:
        v = self.vehicle.get_velocity()
        sensor_health_data = {name: [s.get_health_status(current_frame) for s in s_list] for name, s_list in self.sensors.items()}
        state = {
            'speed': np.linalg.norm([v.x, v.y, v.z]) * 3.6,
            'map_name': self.world.get_map().name.split('/')[-1],
            'collision_count': self.collision_count, 'lane_invasion_count': self.lane_invasion_count,
            'control': self.controller.get_control_state() if self.controller else {},
            'autopilot': self.autopilot, 'recording_images': self.recording_images,
            'recording_sim': self.recording_sim,
            'fused_pos': f"({self.fused_state['x']:.2f}, {self.fused_state['y']:.2f})" if self.fused_state else "Initializing...",
            'sensor_health': sensor_health_data
        }
        return state

    def _get_environmental_data(self) -> Dict[str, Any]:
        weather = self.world.get_weather()
        weather_dict = {
            'cloudiness': weather.cloudiness,
            'precipitation': weather.precipitation,
            'precipitation_deposits': weather.precipitation_deposits,
            'wind_intensity': weather.wind_intensity,
            'sun_azimuth_angle': weather.sun_azimuth_angle,
            'sun_altitude_angle': weather.sun_altitude_angle,
            'fog_density': weather.fog_density,
            'fog_distance': weather.fog_distance,
            'wetness': weather.wetness,
            'fog_falloff': weather.fog_falloff,
            'scattering_intensity': weather.scattering_intensity,
            'mie_scattering_scale': weather.mie_scattering_scale,
            'rayleigh_scattering_scale': weather.rayleigh_scattering_scale,
        }
        return {
            'map': self.world.get_map().name, 
            'weather': weather_dict, 
            'timestamp': self.world.get_snapshot().timestamp.elapsed_seconds
        }

# ==============================================================================
# -- Main Execution ------------------------------------------------------------
# ==============================================================================
if __name__ == '__main__':
    simulation = CarlaSimulation()
    simulation.run()