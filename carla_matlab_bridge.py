import glob
import os
import sys
import random
import numpy as np
import pygame
import carla
import csv
import time
import threading
import socket
import json
import base64
import cv2
from pygame.locals import *

# Initialize Pygame and font module first
pygame.init()
pygame.font.init()

class ManualControl:
    def __init__(self, vehicle, world, client):
        self.vehicle = vehicle
        self.world = world
        self.client = client
        self.autopilot = False
        self.constant_velocity_mode = False
        self.recording_images = False
        self.recording_simulation = False
        self.replay_start = 0
        self.show_hud = True
        self.show_help = False
        self.collision_count = 0
        self.lane_invasion_count = 0
        self.control_state = {
            'throttle': 0.0,
            'brake': 0.0,
            'steer': 0.0,
            'reverse': False,
            'hand_brake': False
        }
        
        # Enhanced autopilot parameters
        self.autopilot_params = {
            'target_speed': 30.0,  # km/h
            'safety_distance': 5.0,  # meters
            'aggressiveness': 0.3,  # 0.0 (cautious) to 1.0 (aggressive)
            'ignore_lights': False,
            'ignore_signs': False
        }
        
        # Weather presets
        self.weather_presets = [
            carla.WeatherParameters.ClearNoon,
            carla.WeatherParameters.CloudyNoon,
            carla.WeatherParameters.WetNoon,
            carla.WeatherParameters.WetCloudyNoon,
            carla.WeatherParameters.MidRainyNoon,
            carla.WeatherParameters.HardRainNoon,
            carla.WeatherParameters.SoftRainNoon,
            carla.WeatherParameters.ClearSunset,
            carla.WeatherParameters.CloudySunset,
            carla.WeatherParameters.WetSunset,
            carla.WeatherParameters.WetCloudySunset,
            carla.WeatherParameters.MidRainSunset,
            carla.WeatherParameters.HardRainSunset,
            carla.WeatherParameters.SoftRainSunset
        ]
        self.current_weather_index = 0
        
        # Map layers
        self.map_layers = [
            carla.MapLayer.NONE,
            carla.MapLayer.Buildings,
            carla.MapLayer.Decals,
            carla.MapLayer.Foliage,
            carla.MapLayer.Ground,
            carla.MapLayer.ParkedVehicles,
            carla.MapLayer.Particles,
            carla.MapLayer.Props,
            carla.MapLayer.StreetLights,
            carla.MapLayer.Walls,
            carla.MapLayer.All
        ]
        self.current_layer_index = 0
        self.current_layer = self.map_layers[self.current_layer_index]
        
        # Initialize weather
        self.world.set_weather(self.weather_presets[self.current_weather_index])
        
        # Create data directories for each camera side
        self.data_dir = 'data'
        self.image_dir = os.path.join(self.data_dir, 'images')
        self.log_dir = os.path.join(self.data_dir, 'logs')
        
        os.makedirs(self.data_dir, exist_ok=True)
        os.makedirs(self.log_dir, exist_ok=True)
        
        # Create subdirectories for each camera view
        self.camera_dirs = {
            'front': os.path.join(self.image_dir, 'front'),
            'back': os.path.join(self.image_dir, 'back'),
            'left': os.path.join(self.image_dir, 'left'),
            'right': os.path.join(self.image_dir, 'right')
        }
        
        for dir_path in self.camera_dirs.values():
            os.makedirs(dir_path, exist_ok=True)
        
        # Font for HUD
        self.font = pygame.font.SysFont('Arial', 12)

    def handle_key_event(self, event):
        if event.type == pygame.KEYDOWN:
            # Basic vehicle controls
            if event.key == pygame.K_w:
                self.control_state['throttle'] = 1.0
                self.control_state['brake'] = 0.0
            elif event.key == pygame.K_s:
                self.control_state['brake'] = 1.0
                self.control_state['throttle'] = 0.0
            elif event.key == pygame.K_a:
                self.control_state['steer'] = -0.7  # Reduced steering for better control
            elif event.key == pygame.K_d:
                self.control_state['steer'] = 0.7   # Reduced steering for better control
            elif event.key == pygame.K_q:
                self.control_state['reverse'] = True
            elif event.key == pygame.K_SPACE:
                self.control_state['hand_brake'] = True
            elif event.key == pygame.K_p:
                self.toggle_autopilot()
            elif event.key == pygame.K_w and pygame.key.get_mods() & pygame.KMOD_CTRL:
                self.toggle_constant_velocity_mode()
            
            # Weather control
            elif event.key == pygame.K_c:
                if pygame.key.get_mods() & pygame.KMOD_SHIFT:
                    self.change_weather(-1)  # Previous weather
                else:
                    self.change_weather(1)   # Next weather
            
            # Map layer control
            elif event.key == pygame.K_v:
                if pygame.key.get_mods() & pygame.KMOD_SHIFT:
                    self.change_map_layer(-1)  # Previous layer
                else:
                    self.change_map_layer(1)   # Next layer
            elif event.key == pygame.K_b:
                if pygame.key.get_mods() & pygame.KMOD_SHIFT:
                    self.world.unload_map_layer(self.current_layer)  # Unload layer
                else:
                    self.world.load_map_layer(self.current_layer)    # Load layer
            
            # Recording control
            elif event.key == pygame.K_r:
                self.recording_images = not self.recording_images
            elif event.key == pygame.K_r and pygame.key.get_mods() & pygame.KMOD_CTRL:
                self.toggle_recording_simulation()
            elif event.key == pygame.K_p and pygame.key.get_mods() & pygame.KMOD_CTRL:
                self.start_replay()
            
            # Replay time adjustment
            elif event.key == pygame.K_PLUS or event.key == pygame.K_EQUALS:
                if pygame.key.get_mods() & pygame.KMOD_CTRL:
                    increment = 10 if pygame.key.get_mods() & pygame.KMOD_SHIFT else 1
                    self.replay_start += increment
            elif event.key == pygame.K_MINUS:
                if pygame.key.get_mods() & pygame.KMOD_CTRL:
                    decrement = 10 if pygame.key.get_mods() & pygame.KMOD_SHIFT else 1
                    self.replay_start = max(0, self.replay_start - decrement)
            
            # Display control
            elif event.key == pygame.K_F1:
                self.show_hud = not self.show_hud
            elif event.key == pygame.K_h or event.key == pygame.K_SLASH:
                self.show_help = not self.show_help
            
            # Map selection (updated with correct town names)
            elif event.key == pygame.K_3:
                self.change_map('Town03')
            elif event.key == pygame.K_0:
                # Town10HD is the correct name for Town10
                self.change_map('Town10HD')
            elif event.key == pygame.K_ESCAPE:
                return False  # Signal to quit
        
        elif event.type == pygame.KEYUP:
            # Reset controls when keys are released
            if event.key == pygame.K_w:
                self.control_state['throttle'] = 0.0
            elif event.key == pygame.K_s:
                self.control_state['brake'] = 0.0
            elif event.key == pygame.K_a or event.key == pygame.K_d:
                self.control_state['steer'] = 0.0
            elif event.key == pygame.K_q:
                self.control_state['reverse'] = False
            elif event.key == pygame.K_SPACE:
                self.control_state['hand_brake'] = False
        
        # Apply controls if not in autopilot
        if not self.autopilot:
            self.apply_control()
        
        return True

    def apply_control(self):
        """Apply current control state to vehicle"""
        control = carla.VehicleControl(
            throttle=self.control_state['throttle'],
            steer=self.control_state['steer'],
            brake=self.control_state['brake'],
            hand_brake=self.control_state['hand_brake'],
            reverse=self.control_state['reverse']
        )
        self.vehicle.apply_control(control)

    def toggle_autopilot(self):
        self.autopilot = not self.autopilot
        
        if self.autopilot:
            # Get traffic manager and configure for safer driving
            tm = self.client.get_trafficmanager()
            tm_port = tm.get_port()
            
            # Configure safer driving parameters
            self.vehicle.set_autopilot(True, tm_port)
            tm.ignore_lights_percentage(self.vehicle, 0)
            tm.ignore_signs_percentage(self.vehicle, 0)
            tm.ignore_vehicles_percentage(self.vehicle, 0)
            tm.distance_to_leading_vehicle(self.vehicle, 5.0)  # Safe distance
            tm.vehicle_percentage_speed_difference(self.vehicle, -20.0)  # Drive slower
            
            # Add additional safety checks
            self.add_autopilot_safety_checks()
            
            # Reset controls
            self.control_state = {
                'throttle': 0.0,
                'brake': 0.0,
                'steer': 0.0,
                'reverse': False,
                'hand_brake': False
            }
        else:
            self.vehicle.set_autopilot(False)
            self.apply_control()
    
    def add_autopilot_safety_checks(self):
        """Add safety checks to prevent accidents in autopilot mode"""
        # Add collision sensor specifically for autopilot safety
        blueprint = self.world.get_blueprint_library().find('sensor.other.collision')
        transform = carla.Transform(carla.Location(x=0, z=2.5))
        self.autopilot_collision_sensor = self.world.spawn_actor(blueprint, transform, attach_to=self.vehicle)
        
        # Define callback for collision events
        def on_collision(event):
            self.collision_count += 1
            intensity = np.sqrt(event.normal_impulse.x**2 + event.normal_impulse.y**2 + event.normal_impulse.z**2)
            
            # If collision is serious, apply emergency brake
            if intensity > 500:
                control = carla.VehicleControl()
                control.throttle = 0.0
                control.brake = 1.0
                control.steer = 0.0
                self.vehicle.apply_control(control)
                print(f"EMERGENCY BRAKE! Collision intensity: {intensity}")
        
        # Register callback
        self.autopilot_collision_sensor.listen(on_collision)

    def toggle_constant_velocity_mode(self):
        self.constant_velocity_mode = not self.constant_velocity_mode
        if self.constant_velocity_mode:
            self.vehicle.apply_control(carla.VehicleControl(throttle=0.7, steer=0, brake=0))  # Reduced throttle
        else:
            self.vehicle.apply_control(carla.VehicleControl())

    def change_weather(self, direction):
        self.current_weather_index = (self.current_weather_index + direction) % len(self.weather_presets)
        weather = self.weather_presets[self.current_weather_index]
        
        # Reduce lighting changes to help autopilot
        weather.sun_altitude_angle = 70  # Higher sun angle for more consistent lighting
        weather.sun_azimuth_angle = 0    # Fixed azimuth
        self.world.set_weather(weather)

    def change_map_layer(self, direction):
        self.current_layer_index = (self.current_layer_index + direction) % len(self.map_layers)
        self.current_layer = self.map_layers[self.current_layer_index]

    def toggle_recording_simulation(self):
        if self.recording_simulation:
            self.client.stop_recorder()
            print("Stopped recording simulation")
        else:
            filename = os.path.join(self.log_dir, f"recording_{time.strftime('%Y%m%d_%H%M%S')}.log")
            self.client.start_recorder(filename)
            print(f"Started recording simulation to {filename}")
        self.recording_simulation = not self.recording_simulation

    def start_replay(self):
        # Look for the most recent recording
        recordings = glob.glob(os.path.join(self.log_dir, "recording_*.log"))
        if recordings:
            # Get the latest recording
            latest_recording = max(recordings, key=os.path.getctime)
            self.client.replay_file(latest_recording, self.replay_start, 0, 0)
            print(f"Started replay from {self.replay_start} seconds: {os.path.basename(latest_recording)}")
        else:
            print("No recording found")

    def change_map(self, map_name):
        """Change map with validation and error handling"""
        # Check available maps
        available_maps = [os.path.basename(x) for x in self.client.get_available_maps()]
        available_maps = [x.split('.')[0] for x in available_maps]  # Remove file extensions
        
        print(f"Available maps: {available_maps}")
        
        if map_name not in available_maps:
            print(f"Map {map_name} not available. Available maps: {available_maps}")
            return
            
        print(f"Loading map: {map_name}")
        try:
            # Destroy all actors first
            print("Destroying all actors...")
            if self.vehicle:
                self.vehicle.destroy()
            actors = self.world.get_actors()
            for actor in actors:
                if actor.type_id.startswith('sensor') or actor.type_id.startswith('vehicle'):
                    actor.destroy()
            
            # Give time for destruction to complete
            time.sleep(1.0)
            
            # Load new map
            self.client.load_world(map_name)
            
            # Wait for map to load
            time.sleep(3.0)
            
            # Reinitialize after map change
            self.world = self.client.get_world()
            self.world.set_weather(self.weather_presets[self.current_weather_index])
            self.world.load_map_layer(self.current_layer)
            
            # Reinitialize vehicle
            blueprint_library = self.world.get_blueprint_library()
            vehicle_models = [
                'vehicle.audi.etron',
                'vehicle.volkswagen.t2',
                'vehicle.nissan.patrol',
                'vehicle.toyota.prius'
            ]
            vehicle_bp = random.choice([bp for bp in blueprint_library.filter('vehicle.*') 
                                        if bp.id in vehicle_models])
            
            spawn_points = self.world.get_map().get_spawn_points()
            valid_spawns = [p for p in spawn_points if p.location.z > 0.5]
            spawn_point = random.choice(valid_spawns) if valid_spawns else spawn_points[0]
            
            self.vehicle = self.world.spawn_actor(vehicle_bp, spawn_point)
            
            print(f"Successfully loaded {map_name}")
            return True
        except RuntimeError as e:
            print(f"Error loading map: {str(e)}")
        except Exception as e:
            print(f"Unexpected error loading map: {str(e)}")
        return False

    def render_hud(self, display):
        if not self.show_hud:
            return
            
        # Get vehicle speed
        velocity = self.vehicle.get_velocity()
        speed = 3.6 * np.linalg.norm(np.array([velocity.x, velocity.y, velocity.z]))  # m/s to km/h
        
        # Create HUD text
        hud_text = [
            f"Speed: {speed:.2f} km/h",
            f"Autopilot: {'ON' if self.autopilot else 'OFF'}",
            f"Recording Images: {'ON' if self.recording_images else 'OFF'}",
            f"Recording Sim: {'ON' if self.recording_simulation else 'OFF'}",
            f"Replay Start: {self.replay_start}s",
            f"Weather: {type(self.weather_presets[self.current_weather_index]).__name__}",
            f"Map Layer: {self.current_layer.name if hasattr(self.current_layer, 'name') else str(self.current_layer)}",
            f"Map: {self.world.get_map().name}",
            f"Collisions: {self.collision_count}",
            f"Lane Invasions: {self.lane_invasion_count}",
            f"Steering: {self.control_state['steer']:.2f}",
            f"Throttle: {self.control_state['throttle']:.2f}",
            f"Brake: {self.control_state['brake']:.2f}"
        ]
        
        # Render HUD text
        y_offset = 10
        for text in hud_text:
            text_surface = self.font.render(text, True, (255, 255, 255))
            display.blit(text_surface, (10, y_offset))
            y_offset += 25

    def render_help(self, display):
        help_text = [
            "Controls:",
            "W/S: Throttle/Brake",
            "A/D: Steer Left/Right",
            "Q: Toggle Reverse",
            "Space: Hand Brake",
            "P: Toggle Autopilot",
            "Ctrl+W: Constant Velocity Mode",
            "C: Next Weather (Shift+C: Prev)",
            "V: Next Map Layer (Shift+V: Prev)",
            "B: Load Layer (Shift+B: Unload)",
            "R: Toggle Image Recording",
            "Ctrl+R: Toggle Simulation Recording",
            "Ctrl+P: Start Replay",
            "Ctrl++/-: Adjust Replay Time",
            "F1: Toggle HUD",
            "H: Toggle Help",
            "3: Switch to Town03",
            "0: Switch to Town10HD (Town10)",
            "ESC: Quit"
        ]
        
        # Create a semi-transparent background
        s = pygame.Surface((500, 500))
        s.set_alpha(200)
        s.fill((0, 0, 0))
        display.blit(s, (50, 50))
        
        # Render help text
        y_offset = 60
        for text in help_text:
            text_surface = self.font.render(text, True, (255, 255, 255))
            display.blit(text_surface, (60, y_offset))
            y_offset += 30

class DisplayManager:
    def __init__(self, grid_size):
        self.display = pygame.display.set_mode((grid_size[0]*320, grid_size[1]*240))
        pygame.display.set_caption("Sensor Data")
        self.grid_size = grid_size
        self.sensors = []

    def attach_sensor(self, sensor_manager, position):
        self.sensors.append((sensor_manager, position))

    def render(self):
        self.display.fill((0, 0, 0))
        for sensor, position in self.sensors:
            if sensor.surface is not None:
                self.display.blit(sensor.surface, (position[0]*320, position[1]*240))
        return self.display

class SensorManager:
    def __init__(self, world, display_manager, sensor_type, transform, attached_vehicle, attributes=None, display_pos=(0,0), camera_name='front'):
        blueprint_library = world.get_blueprint_library()
        self.sensor_type = sensor_type
        self.camera_name = camera_name  # Store camera name for saving images
        self.collision_events = []
        self.collision_lock = threading.Lock()
        self.lane_invasion_events = []
        self.lane_invasion_lock = threading.Lock()

        if sensor_type == 'RGB':
            bp = blueprint_library.find('sensor.camera.rgb')
            bp.set_attribute('image_size_x', '320')
            bp.set_attribute('image_size_y', '240')
            bp.set_attribute('fov', '90')
            
            # Only set exposure attributes if they exist in this CARLA version
            if bp.has_attribute('exposure_mode'):
                bp.set_attribute('exposure_mode', 'manual')
            if bp.has_attribute('exposure_min_bright'):
                bp.set_attribute('exposure_min_bright', '10')
            if bp.has_attribute('exposure_max_bright'):
                bp.set_attribute('exposure_max_bright', '20')
            
        elif sensor_type == 'LIDAR':
            bp = blueprint_library.find('sensor.lidar.ray_cast')
            bp.set_attribute('range', '50')
            bp.set_attribute('rotation_frequency', '10')
            bp.set_attribute('channels', '64')
            bp.set_attribute('points_per_second', '56000')
        elif sensor_type == 'IMU':
            bp = blueprint_library.find('sensor.other.imu')
        elif sensor_type == 'GNSS':
            bp = blueprint_library.find('sensor.other.gnss')
            bp.set_attribute('noise_alt_stddev', '0.0')
            bp.set_attribute('noise_lat_stddev', '0.0')
            bp.set_attribute('noise_lon_stddev', '0.0')
        elif sensor_type == 'COLLISION':
            bp = blueprint_library.find('sensor.other.collision')
        elif sensor_type == 'LANE_INVASION':
            bp = blueprint_library.find('sensor.other.lane_invasion')
        else:
            raise ValueError(f"Unknown sensor type: {sensor_type}")

        if attributes:
            for attr_name, attr_value in attributes.items():
                if bp.has_attribute(attr_name):
                    bp.set_attribute(attr_name, attr_value)

        self.sensor = world.spawn_actor(bp, transform, attach_to=attached_vehicle)
        self.surface = None
        self.data = None

        # Set up appropriate callback based on sensor type
        if sensor_type == 'RGB':
            self.sensor.listen(lambda data: self._parse_image(data))
        elif sensor_type in ['LIDAR', 'IMU', 'GNSS']:
            self.sensor.listen(lambda data: setattr(self, 'data', data))
        elif sensor_type == 'COLLISION':
            self.sensor.listen(lambda event: self._parse_collision(event))
        elif sensor_type == 'LANE_INVASION':
            self.sensor.listen(lambda event: self._parse_lane_invasion(event))

        if display_manager and display_pos is not None:
            display_manager.attach_sensor(self, display_pos)

    def _parse_image(self, image):
        if self.sensor_type == 'RGB':
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (image.height, image.width, 4))
            array = array[:, :, :3]
            array = array[:, :, ::-1]
            self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
            self.data = array

    def _parse_collision(self, event):
        with self.collision_lock:
            self.collision_events.append(event)

    def _parse_lane_invasion(self, event):
        with self.lane_invasion_lock:
            self.lane_invasion_events.append(event)

    def get_collision_events(self):
        with self.collision_lock:
            events = self.collision_events[:]
            self.collision_events = []
        return events

    def get_lane_invasion_events(self):
        with self.lane_invasion_lock:
            events = self.lane_invasion_events[:]
            self.lane_invasion_events = []
        return events

class UDPSender:
    def __init__(self, ip='127.0.0.1', port=10000):
        self.ip = ip
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        print(f"UDP sender initialized to {ip}:{port}")

    def send_data(self, data_dict):
        """Send dictionary data as JSON over UDP"""
        try:
            # Compress and encode images
            for camera in ['front', 'back', 'left', 'right']:
                if f'image_{camera}' in data_dict:
                    img = data_dict[f'image_{camera}']
                    # Compress image to JPEG
                    _, jpeg_img = cv2.imencode('.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
                    # Convert to base64 string
                    data_dict[f'image_{camera}'] = base64.b64encode(jpeg_img).decode('utf-8')
            
            # Process LIDAR data
            if 'lidar' in data_dict:
                # Convert to bytes and base64 encode
                lidar_bytes = data_dict['lidar'].tobytes()
                data_dict['lidar'] = base64.b64encode(lidar_bytes).decode('utf-8')
            
            # Process IMU data
            if 'imu' in data_dict:
                # Convert to bytes and base64 encode
                imu_data = np.array([
                    data_dict['imu']['accel_x'],
                    data_dict['imu']['accel_y'],
                    data_dict['imu']['accel_z'],
                    data_dict['imu']['gyro_x'],
                    data_dict['imu']['gyro_y'],
                    data_dict['imu']['gyro_z']
                ], dtype=np.float32)
                imu_bytes = imu_data.tobytes()
                data_dict['imu'] = base64.b64encode(imu_bytes).decode('utf-8')
            
            json_data = json.dumps(data_dict)
            
            # Check if data is too large
            if len(json_data) > 60000:
                # Split into chunks
                chunks = [json_data[i:i+60000] for i in range(0, len(json_data), 60000)]
                frame_id = data_dict.get('frame', -1)  # Use -1 or other default if not found
                for i, chunk in enumerate(chunks):
                    packet = {
                        'frame': frame_id,
                        'chunk': i,
                        'total_chunks': len(chunks),
                        'data': chunk
                    }
                    self.sock.sendto(json.dumps(packet).encode('utf-8'), (self.ip, self.port))
            else:
                self.sock.sendto(json_data.encode('utf-8'), (self.ip, self.port))
        except Exception as e:
            print(f"UDP send error: {e}")

    def close(self):
        self.sock.close()

# Main execution
def main():
    # Create data directories if they don't exist
    data_dir = 'data'
    image_dir = os.path.join(data_dir, 'images')
    log_dir = os.path.join(data_dir, 'logs')
    os.makedirs(data_dir, exist_ok=True)
    os.makedirs(log_dir, exist_ok=True)

    # Create subdirectories for each camera view
    camera_dirs = {
        'front': os.path.join(image_dir, 'front'),
        'back': os.path.join(image_dir, 'back'),
        'left': os.path.join(image_dir, 'left'),
        'right': os.path.join(image_dir, 'right')
    }

    for dir_path in camera_dirs.values():
        os.makedirs(dir_path, exist_ok=True)

    # Initialize variables for cleanup
    csv_file = None
    collision_log_file = None
    lane_invasion_log_file = None
    vehicle = None
    world = None
    display_manager = None
    manual_control = None
    
    try:
        # Connect to CARLA and load Town03
        client = carla.Client('localhost', 2000)
        client.set_timeout(20.0)
        
        # Load Town03 by default
        print("Loading Town03...")
        client.load_world('Town03')
        world = client.get_world()
        print(f"Current map: {world.get_map().name}")
        
        # Set default weather to reduce shadow issues
        weather = carla.WeatherParameters(
            cloudiness=30,
            precipitation=0,
            sun_altitude_angle=70.0,
            fog_density=0
        )
        world.set_weather(weather)

        # Set synchronous mode
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        world.apply_settings(settings)

        blueprint_library = world.get_blueprint_library()
        
        # Choose a safer vehicle model
        vehicle_models = [
            'vehicle.audi.etron',
            'vehicle.volkswagen.t2',
            'vehicle.nissan.patrol',
            'vehicle.toyota.prius'
        ]
        vehicle_bp = random.choice([bp for bp in blueprint_library.filter('vehicle.*') 
                                    if bp.id in vehicle_models])
        
        # Choose spawn point carefully to avoid autopilot issues
        spawn_points = world.get_map().get_spawn_points()
        valid_spawns = [p for p in spawn_points if p.location.z > 0.5]
        spawn_point = random.choice(valid_spawns) if valid_spawns else spawn_points[0]
            
        vehicle = world.spawn_actor(vehicle_bp, spawn_point)

        # Create display manager with a 2x2 grid for the 4 cameras
        display_manager = DisplayManager(grid_size=(2, 2))

        # Manual Control Setup
        manual_control = ManualControl(vehicle, world, client)

        # Initialize UDP sender
        udp_sender = UDPSender()

        # Sensor Setup - Add 4 RGB cameras for each side
        # Front camera
        camera_front = SensorManager(world, display_manager, 'RGB', 
                                   carla.Transform(carla.Location(x=1.5, z=2.4)), 
                                   vehicle, display_pos=(0, 0), camera_name='front')

        # Back camera
        camera_back = SensorManager(world, display_manager, 'RGB', 
                                  carla.Transform(carla.Location(x=-1.5, z=2.4), carla.Rotation(yaw=180)), 
                                  vehicle, display_pos=(1, 0), camera_name='back')

        # Left camera
        camera_left = SensorManager(world, display_manager, 'RGB', 
                                  carla.Transform(carla.Location(y=-1.5, z=2.4), carla.Rotation(yaw=-90)), 
                                  vehicle, display_pos=(0, 1), camera_name='left')

        # Right camera
        camera_right = SensorManager(world, display_manager, 'RGB', 
                                   carla.Transform(carla.Location(y=1.5, z=2.4), carla.Rotation(yaw=90)), 
                                   vehicle, display_pos=(1, 1), camera_name='right')

        # Other sensors (positioned at the center of the vehicle)
        lidar_sensor = SensorManager(world, display_manager, 'LIDAR', carla.Transform(carla.Location(x=0, z=2.5)), vehicle)
        imu_sensor = SensorManager(world, display_manager, 'IMU', carla.Transform(carla.Location()), vehicle)
        gnss_sensor = SensorManager(world, display_manager, 'GNSS', carla.Transform(carla.Location()), vehicle)
        
        # Collision and lane detection sensors
        collision_sensor = SensorManager(world, None, 'COLLISION', carla.Transform(carla.Location()), vehicle)
        lane_invasion_sensor = SensorManager(world, None, 'LANE_INVASION', carla.Transform(carla.Location()), vehicle)

        # CSV Data Saving
        csv_path = os.path.join(data_dir, 'sensor_data.csv')
        csv_file = open(csv_path, mode='w', newline='')
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow(['timestamp', 'speed_kmph', 'vehicle_x', 'vehicle_y', 'vehicle_z',
                             'gnss_latitude', 'gnss_longitude', 'gnss_altitude',
                             'imu_acc_x', 'imu_acc_y', 'imu_acc_z',
                             'imu_gyro_x', 'imu_gyro_y', 'imu_gyro_z',
                             'collision_count', 'lane_invasion_count'])

        # Collision log file
        collision_log_path = os.path.join(log_dir, 'collision_log.csv')
        collision_log_file = open(collision_log_path, 'w', newline='')
        collision_log_writer = csv.writer(collision_log_file)
        collision_log_writer.writerow(['timestamp', 'frame', 'intensity', 'other_actor_type', 'other_actor_id'])

        # Lane invasion log file
        lane_invasion_log_path = os.path.join(log_dir, 'lane_invasion_log.csv')
        lane_invasion_log_file = open(lane_invasion_log_path, 'w', newline='')
        lane_invasion_log_writer = csv.writer(lane_invasion_log_file)
        lane_invasion_log_writer.writerow(['timestamp', 'frame', 'lane_types', 'invasion_count'])

        clock = pygame.time.Clock()
        frame_count = 0
        last_udp_send_time = time.time()

        running = True
        while running:
            # Handle pygame events to prevent freezing
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type in (pygame.KEYDOWN, pygame.KEYUP):
                    if not manual_control.handle_key_event(event):
                        running = False
                    elif event.type == pygame.KEYDOWN and event.key in (pygame.K_3, pygame.K_0):
                        # Map change requested - reinitialize sensors
                        if manual_control.change_map('Town03' if event.key == pygame.K_3 else 'Town10HD'):
                            # Reinitialize sensors after map change
                            world = manual_control.world
                            vehicle = manual_control.vehicle
                            
                            # Recreate sensors
                            camera_front = SensorManager(world, display_manager, 'RGB', 
                                                       carla.Transform(carla.Location(x=1.5, z=2.4)), 
                                                       vehicle, display_pos=(0, 0), camera_name='front')
                            camera_back = SensorManager(world, display_manager, 'RGB', 
                                                      carla.Transform(carla.Location(x=-1.5, z=2.4), carla.Rotation(yaw=180)), 
                                                      vehicle, display_pos=(1, 0), camera_name='back')
                            camera_left = SensorManager(world, display_manager, 'RGB', 
                                                      carla.Transform(carla.Location(y=-1.5, z=2.4), carla.Rotation(yaw=-90)), 
                                                      vehicle, display_pos=(0, 1), camera_name='left')
                            camera_right = SensorManager(world, display_manager, 'RGB', 
                                                       carla.Transform(carla.Location(y=1.5, z=2.4), carla.Rotation(yaw=90)), 
                                                       vehicle, display_pos=(1, 1), camera_name='right')
                            
                            # Other sensors
                            lidar_sensor = SensorManager(world, display_manager, 'LIDAR', carla.Transform(carla.Location(x=0, z=2.5)), vehicle)
                            imu_sensor = SensorManager(world, display_manager, 'IMU', carla.Transform(carla.Location()), vehicle)
                            gnss_sensor = SensorManager(world, display_manager, 'GNSS', carla.Transform(carla.Location()), vehicle)
                            collision_sensor = SensorManager(world, None, 'COLLISION', carla.Transform(carla.Location()), vehicle)
                            lane_invasion_sensor = SensorManager(world, None, 'LANE_INVASION', carla.Transform(carla.Location()), vehicle)

            # Tick the world
            world.tick()
            
            # Render the display
            display = display_manager.render()
            
            # Save RGB images from all cameras if recording is enabled
            if manual_control.recording_images:
                # Save front camera image
                if camera_front.data is not None:
                    image_path = os.path.join(camera_dirs['front'], f'frame_{frame_count:05d}.png')
                    pygame.image.save(camera_front.surface, image_path)
                
                # Save back camera image
                if camera_back.data is not None:
                    image_path = os.path.join(camera_dirs['back'], f'frame_{frame_count:05d}.png')
                    pygame.image.save(camera_back.surface, image_path)
                
                # Save left camera image
                if camera_left.data is not None:
                    image_path = os.path.join(camera_dirs['left'], f'frame_{frame_count:05d}.png')
                    pygame.image.save(camera_left.surface, image_path)
                
                # Save right camera image
                if camera_right.data is not None:
                    image_path = os.path.join(camera_dirs['right'], f'frame_{frame_count:05d}.png')
                    pygame.image.save(camera_right.surface, image_path)
                
                frame_count += 1

            # Process collision events
            collision_events = collision_sensor.get_collision_events()
            for event in collision_events:
                manual_control.collision_count += 1
                intensity = np.sqrt(event.normal_impulse.x**2 + event.normal_impulse.y**2 + event.normal_impulse.z**2)
                other_actor = event.other_actor
                collision_log_writer.writerow([
                    time.time(),
                    frame_count,
                    intensity,
                    other_actor.type_id if other_actor else "None",
                    other_actor.id if other_actor else "None"
                ])

            # Process lane invasion events
            lane_invasion_events = lane_invasion_sensor.get_lane_invasion_events()
            for event in lane_invasion_events:
                manual_control.lane_invasion_count += 1
                lane_types = [marking.type.name for marking in event.crossed_lane_markings]
                lane_invasion_log_writer.writerow([
                    time.time(),
                    frame_count,
                    ','.join(lane_types),
                    len(event.crossed_lane_markings)
                ])

            # Render HUD and help
            manual_control.render_hud(display)
            if manual_control.show_help:
                manual_control.render_help(display)
            pygame.display.flip()

            # Get vehicle data
            velocity = vehicle.get_velocity()
            speed = 3.6 * np.linalg.norm(np.array([velocity.x, velocity.y, velocity.z]))
            transform = vehicle.get_transform()
            location = transform.location
            rotation = transform.rotation

            # Prepare data for UDP transmission
            current_time = time.time()
            if current_time - last_udp_send_time >= 0.05:  # Send at 20Hz (increased frequency)
                data_dict = {
                    'timestamp': current_time,
                    'frame': frame_count,
                    'speed': float(speed),
                    'position': {
                        'x': float(location.x),
                        'y': float(location.y),
                        'z': float(location.z)
                    },
                    'rotation': {
                        'pitch': float(rotation.pitch),
                        'yaw': float(rotation.yaw),
                        'roll': float(rotation.roll)
                    },
                    'control': manual_control.control_state,
                    'collisions': manual_control.collision_count,
                    'lane_invasions': manual_control.lane_invasion_count
                }
                
                # Add camera images
                if camera_front.data is not None:
                    data_dict['image_front'] = camera_front.data
                if camera_back.data is not None:
                    data_dict['image_back'] = camera_back.data
                if camera_left.data is not None:
                    data_dict['image_left'] = camera_left.data
                if camera_right.data is not None:
                    data_dict['image_right'] = camera_right.data
                
                # Add LIDAR data
                if lidar_sensor.data is not None:
                    points = np.frombuffer(lidar_sensor.data.raw_data, dtype=np.dtype('f4'))
                    points = np.reshape(points, (int(points.shape[0]/4), 4))
                    data_dict['lidar'] = points
                
                # Add IMU data
                if imu_sensor.data is not None:
                    data_dict['imu'] = {
                        'accel_x': float(imu_sensor.data.accelerometer.x),
                        'accel_y': float(imu_sensor.data.accelerometer.y),
                        'accel_z': float(imu_sensor.data.accelerometer.z),
                        'gyro_x': float(imu_sensor.data.gyroscope.x),
                        'gyro_y': float(imu_sensor.data.gyroscope.y),
                        'gyro_z': float(imu_sensor.data.gyroscope.z)
                    }
                
                # Add GNSS data
                if gnss_sensor.data is not None:
                    data_dict['gnss'] = {
                        'lat': float(gnss_sensor.data.latitude),
                        'lon': float(gnss_sensor.data.longitude),
                        'alt': float(gnss_sensor.data.altitude)
                    }
                
                # Send data via UDP
                udp_sender.send_data(data_dict)
                last_udp_send_time = current_time

            # Get vehicle position
            transform = vehicle.get_transform()
            location = transform.location

            # GNSS data
            if gnss_sensor.data:
                gnss_lat = gnss_sensor.data.latitude
                gnss_lon = gnss_sensor.data.longitude
                gnss_alt = gnss_sensor.data.altitude
            else:
                gnss_lat = gnss_lon = gnss_alt = None

            # IMU data
            if imu_sensor.data:
                imu_acc = imu_sensor.data.accelerometer
                imu_gyro = imu_sensor.data.gyroscope
            else:
                imu_acc = imu_gyro = None

            # Write to CSV
            csv_writer.writerow([
                time.time(),
                speed,
                location.x, location.y, location.z,
                gnss_lat, gnss_lon, gnss_alt,
                imu_acc.x if imu_acc else None,
                imu_acc.y if imu_acc else None,
                imu_acc.z if imu_acc else None,
                imu_gyro.x if imu_gyro else None,
                imu_gyro.y if imu_gyro else None,
                imu_gyro.z if imu_gyro else None,
                manual_control.collision_count,
                manual_control.lane_invasion_count
            ])

            clock.tick(30)

    except KeyboardInterrupt:
        print('Simulation interrupted by user.')
    except Exception as e:
        print(f'Error occurred: {e}')
    finally:
        try:
            # Close UDP socket
            if 'udp_sender' in locals():
                udp_sender.close()
            
            # Close CSV files
            if csv_file: csv_file.close()
            if collision_log_file: collision_log_file.close()
            if lane_invasion_log_file: lane_invasion_log_file.close()
            
            if 'csv_path' in locals():
                print(f"Data saved to: {csv_path}")
            if 'collision_log_path' in locals():
                print(f"Collision log saved to: {collision_log_path}")
            if 'lane_invasion_log_path' in locals():
                print(f"Lane invasion log saved to: {lane_invasion_log_path}")
            
            # Destroy actors
            if 'vehicle' in locals() and vehicle:
                print("Destroying actors...")
                vehicle.destroy()
                if 'world' in locals():
                    for sensor in world.get_actors().filter('*sensor*'):
                        sensor.destroy()
            
            # Reset world settings
            if 'world' in locals():
                settings = world.get_settings()
                settings.synchronous_mode = False
                settings.fixed_delta_seconds = None
                world.apply_settings(settings)
        except Exception as e:
            print(f"Error during cleanup: {e}")
        
        pygame.quit()
        print("Simulation ended.")

if __name__ == '__main__':
    main()