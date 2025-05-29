import glob
import os
import sys
import random
import numpy as np
import pygame
import carla
import csv
import time
import rospy
from sensor_msgs.msg import Image, NavSatFix, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Vector3
from std_msgs.msg import String, Float32
from cv_bridge import CvBridge
import math
from pygame.locals import *

# Initialize Pygame and font module first
pygame.init()
pygame.font.init()

# Initialize ROS node
rospy.init_node('carla_data_collector')
bridge = CvBridge()

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
        
        # Navigation mode tracking
        self.current_nav_mode = "lane_keeping"  # Default mode
        self.nav_modes = ["lane_keeping", "turning_left", "turning_right", 
                         "stopping", "changing_lanes", "reversing"]
        
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
        
        # ROS Publishers
        self.nav_mode_pub = rospy.Publisher('/carla/navigation_mode', String, queue_size=10)
        self.control_pub = rospy.Publisher('/carla/vehicle_control', String, queue_size=10)

    def handle_key_event(self, event):
        if event.type == pygame.KEYDOWN:
            # Navigation mode selection
            if event.key == pygame.K_1:
                self.current_nav_mode = "lane_keeping"
            elif event.key == pygame.K_2:
                self.current_nav_mode = "turning_left"
            elif event.key == pygame.K_3:
                self.current_nav_mode = "turning_right"
            elif event.key == pygame.K_4:
                self.current_nav_mode = "stopping"
            elif event.key == pygame.K_5:
                self.current_nav_mode = "changing_lanes"
            elif event.key == pygame.K_6:
                self.current_nav_mode = "reversing"
            
            # Basic vehicle controls
            elif event.key == pygame.K_w:
                self.vehicle.apply_control(carla.VehicleControl(throttle=1.0))
                self.publish_control("throttle")
            elif event.key == pygame.K_s:
                self.vehicle.apply_control(carla.VehicleControl(brake=1.0))
                self.publish_control("brake")
            elif event.key == pygame.K_a:
                self.vehicle.apply_control(carla.VehicleControl(steer=-1.0))
                self.publish_control("steer_left")
            elif event.key == pygame.K_d:
                self.vehicle.apply_control(carla.VehicleControl(steer=1.0))
                self.publish_control("steer_right")
            elif event.key == pygame.K_q:
                self.vehicle.apply_control(carla.VehicleControl(reverse=True))
                self.publish_control("reverse")
            elif event.key == pygame.K_SPACE:
                self.vehicle.apply_control(carla.VehicleControl(hand_brake=True))
                self.publish_control("hand_brake")
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
            
            # Map selection
            elif event.key == pygame.K_3:
                self.change_map('Town03')
            elif event.key == pygame.K_0:
                self.change_map('Town10')
            elif event.key == pygame.K_ESCAPE:
                return False  # Signal to quit
        
        elif event.type == pygame.KEYUP:
            # Reset controls when keys are released
            if event.key in (pygame.K_w, pygame.K_s, pygame.K_a, pygame.K_d, 
                            pygame.K_q, pygame.K_SPACE):
                self.vehicle.apply_control(carla.VehicleControl())
        
        # Always publish navigation mode
        self.nav_mode_pub.publish(self.current_nav_mode)
        return True

    def publish_control(self, control_type):
        self.control_pub.publish(control_type)

    def toggle_autopilot(self):
        self.autopilot = not self.autopilot
        self.vehicle.set_autopilot(self.autopilot)
        status = "autopilot_on" if self.autopilot else "autopilot_off"
        self.control_pub.publish(status)

    def toggle_constant_velocity_mode(self):
        self.constant_velocity_mode = not self.constant_velocity_mode
        if self.constant_velocity_mode:
            self.vehicle.apply_control(carla.VehicleControl(throttle=1.0, steer=0, brake=0))
            self.control_pub.publish("constant_velocity_on")
        else:
            self.vehicle.apply_control(carla.VehicleControl())
            self.control_pub.publish("constant_velocity_off")

    def change_weather(self, direction):
        self.current_weather_index = (self.current_weather_index + direction) % len(self.weather_presets)
        self.world.set_weather(self.weather_presets[self.current_weather_index])

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
        print(f"Loading map: {map_name}")
        self.client.load_world(map_name)
        # Reinitialize after map change
        self.world = self.client.get_world()
        self.world.set_weather(self.weather_presets[self.current_weather_index])
        self.world.load_map_layer(self.current_layer)

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
            f"Nav Mode: {self.current_nav_mode}"
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
            "0: Switch to Town10",
            "1-6: Navigation Modes",
            "ESC: Quit",
            "",
            "Navigation Modes:",
            "1: Lane Keeping",
            "2: Turning Left",
            "3: Turning Right",
            "4: Stopping",
            "5: Changing Lanes",
            "6: Reversing"
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

        if sensor_type == 'RGB':
            bp = blueprint_library.find('sensor.camera.rgb')
            bp.set_attribute('image_size_x', '320')
            bp.set_attribute('image_size_y', '240')
            bp.set_attribute('fov', '90')
            # ROS Publisher for camera
            self.ros_publisher = rospy.Publisher(f'/carla/{camera_name}/image_raw', Image, queue_size=10)
        elif sensor_type == 'LIDAR':
            bp = blueprint_library.find('sensor.lidar.ray_cast')
            bp.set_attribute('range', '50')
            bp.set_attribute('rotation_frequency', '10')
            bp.set_attribute('channels', '64')
            bp.set_attribute('points_per_second', '56000')
        elif sensor_type == 'IMU':
            bp = blueprint_library.find('sensor.other.imu')
            # ROS Publisher for IMU
            self.ros_publisher = rospy.Publisher('/carla/imu', Imu, queue_size=10)
        elif sensor_type == 'GNSS':
            bp = blueprint_library.find('sensor.other.gnss')
            bp.set_attribute('noise_alt_stddev', '0.0')
            bp.set_attribute('noise_lat_stddev', '0.0')
            bp.set_attribute('noise_lon_stddev', '0.0')
            # ROS Publisher for GNSS
            self.ros_publisher = rospy.Publisher('/carla/gnss', NavSatFix, queue_size=10)
        else:
            raise ValueError(f"Unknown sensor type: {sensor_type}")

        if attributes:
            for attr_name, attr_value in attributes.items():
                bp.set_attribute(attr_name, attr_value)

        self.sensor = world.spawn_actor(bp, transform, attach_to=attached_vehicle)
        self.surface = None
        self.data = None
        self.attached_vehicle = attached_vehicle

        self.sensor.listen(lambda data: self._parse_image(data))
        display_manager.attach_sensor(self, display_pos)

    def _parse_image(self, data):
        if self.sensor_type == 'RGB':
            array = np.frombuffer(data.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (data.height, data.width, 4))
            array = array[:, :, :3]
            array = array[:, :, ::-1]
            self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
            self.data = array
            
            # Publish to ROS
            ros_image = bridge.cv2_to_imgmsg(array, encoding="bgr8")
            ros_image.header.stamp = rospy.Time.now()
            ros_image.header.frame_id = f"{self.camera_name}_camera"
            self.ros_publisher.publish(ros_image)
            
        elif self.sensor_type == 'IMU':
            self.data = data
            # Create and publish ROS IMU message
            imu_msg = Imu()
            imu_msg.header.stamp = rospy.Time.now()
            imu_msg.header.frame_id = "imu_link"
            
            # Convert from CARLA to ROS coordinate system
            imu_msg.linear_acceleration.x = data.accelerometer.x
            imu_msg.linear_acceleration.y = -data.accelerometer.y
            imu_msg.linear_acceleration.z = data.accelerometer.z
            
            imu_msg.angular_velocity.x = data.gyroscope.x
            imu_msg.angular_velocity.y = -data.gyroscope.y
            imu_msg.angular_velocity.z = data.gyroscope.z
            
            self.ros_publisher.publish(imu_msg)
            
        elif self.sensor_type == 'GNSS':
            self.data = data
            # Create and publish ROS NavSatFix message
            gnss_msg = NavSatFix()
            gnss_msg.header.stamp = rospy.Time.now()
            gnss_msg.header.frame_id = "gps"
            gnss_msg.latitude = data.latitude
            gnss_msg.longitude = data.longitude
            gnss_msg.altitude = data.altitude
            self.ros_publisher.publish(gnss_msg)

# ROS Publisher for vehicle state
class VehicleStatePublisher:
    def __init__(self, vehicle):
        self.vehicle = vehicle
        self.odom_pub = rospy.Publisher('/carla/odometry', Odometry, queue_size=10)
        self.speed_pub = rospy.Publisher('/carla/speed', Float32, queue_size=10)
        
    def publish(self):
        # Get vehicle state
        transform = self.vehicle.get_transform()
        velocity = self.vehicle.get_velocity()
        angular_velocity = self.vehicle.get_angular_velocity()
        
        # Create Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = "map"
        odom_msg.child_frame_id = "base_link"
        
        # Position
        odom_msg.pose.pose.position.x = transform.location.x
        odom_msg.pose.pose.position.y = -transform.location.y  # Convert to ROS right-handed
        odom_msg.pose.pose.position.z = transform.location.z
        
        # Orientation - convert from Euler angles to quaternion
        roll = math.radians(transform.rotation.roll)
        pitch = math.radians(transform.rotation.pitch)
        yaw = math.radians(-transform.rotation.yaw)  # Convert to ROS right-handed
        
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        odom_msg.pose.pose.orientation.w = cy * cp * cr + sy * sp * sr
        odom_msg.pose.pose.orientation.x = cy * cp * sr - sy * sp * cr
        odom_msg.pose.pose.orientation.y = sy * cp * sr + cy * sp * cr
        odom_msg.pose.pose.orientation.z = sy * cp * cr - cy * sp * sr
        
        # Velocity
        odom_msg.twist.twist.linear.x = velocity.x
        odom_msg.twist.twist.linear.y = -velocity.y  # Convert to ROS right-handed
        odom_msg.twist.twist.linear.z = velocity.z
        
        odom_msg.twist.twist.angular.x = angular_velocity.x
        odom_msg.twist.twist.angular.y = -angular_velocity.y  # Convert to ROS right-handed
        odom_msg.twist.twist.angular.z = angular_velocity.z
        
        self.odom_pub.publish(odom_msg)
        
        # Publish speed separately
        speed = 3.6 * math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)  # km/h
        self.speed_pub.publish(speed)

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

    try:
        # Connect to CARLA and load Town03
        client = carla.Client('localhost', 2000)
        client.set_timeout(20.0)
        client.load_world('Town03')
        world = client.get_world()

        # Set synchronous mode
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        world.apply_settings(settings)

        blueprint_library = world.get_blueprint_library()
        vehicle_bp = random.choice(blueprint_library.filter('vehicle.*'))
        spawn_point = random.choice(world.get_map().get_spawn_points())
        vehicle = world.spawn_actor(vehicle_bp, spawn_point)

        # Create display manager with a 2x2 grid for the 4 cameras
        display_manager = DisplayManager(grid_size=(2, 2))

        # Manual Control Setup
        manual_control = ManualControl(vehicle, world, client)
        
        # Vehicle state publisher
        vehicle_publisher = VehicleStatePublisher(vehicle)

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

        # CSV Data Saving
        csv_path = os.path.join(data_dir, 'sensor_data.csv')
        csv_file = open(csv_path, mode='w', newline='')
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow(['timestamp', 'speed_kmph', 'vehicle_x', 'vehicle_y', 'vehicle_z',
                             'gnss_latitude', 'gnss_longitude', 'gnss_altitude',
                             'imu_acc_x', 'imu_acc_y', 'imu_acc_z',
                             'imu_gyro_x', 'imu_gyro_y', 'imu_gyro_z',
                             'nav_mode'])

        clock = pygame.time.Clock()
        frame_count = 0

        running = True
        while running and not rospy.is_shutdown():
            # Handle pygame events to prevent freezing
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type in (pygame.KEYDOWN, pygame.KEYUP):
                    if not manual_control.handle_key_event(event):
                        running = False

            # Tick the world
            world.tick()
            
            # Publish vehicle state to ROS
            vehicle_publisher.publish()
            
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

            # Render HUD and help
            manual_control.render_hud(display)
            if manual_control.show_help:
                manual_control.render_help(display)
            pygame.display.flip()

            # Get vehicle speed
            velocity = vehicle.get_velocity()
            speed = 3.6 * np.linalg.norm(np.array([velocity.x, velocity.y, velocity.z]))  # m/s to km/h

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
                manual_control.current_nav_mode
            ])

            clock.tick(30)

    except KeyboardInterrupt:
        print('Simulation interrupted by user.')
    except Exception as e:
        print(f'Error occurred: {e}')
    finally:
        try:
            csv_file.close()
            print(f"Data saved to: {csv_path}")
            
            # Destroy actors
            print("Destroying actors...")
            vehicle.destroy()
            for sensor in world.get_actors().filter('*sensor*'):
                sensor.destroy()
            
            # Reset world settings
            settings = world.get_settings()
            settings.synchronous_mode = False
            settings.fixed_delta_seconds = None
            world.apply_settings(settings)
        except:
            pass
        
        pygame.quit()
        print("Simulation ended.")

if __name__ == '__main__':
    main()