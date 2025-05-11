import glob
import os
import sys
import random
import numpy as np
import pygame
import carla
import csv
import time

"""
Welcome to CARLA manual control.

Use ARROWS or WASD keys for control.

    W            : throttle
    S            : brake
    A/D          : steer left/right
    Q            : toggle reverse
    Space        : hand-brake
    P            : toggle autopilot
    M            : toggle manual transmission
    ,/.          : gear up/down
    CTRL + W     : toggle constant velocity mode at 60 km/h

    L            : toggle next light type
    SHIFT + L    : toggle high beam
    Z/X          : toggle right/left blinker
    I            : toggle interior light

    TAB          : change sensor position
     or N       : next sensor
    [1-9]        : change to sensor [1-9]
    G            : toggle radar visualization
    C            : change weather (Shift+C reverse)
    Backspace    : change vehicle

    O            : open/close all doors of vehicle
    T            : toggle vehicle's telemetry

    V            : Select next map layer (Shift+V reverse)
    B            : Load current selected map layer (Shift+B to unload)

    R            : toggle recording images to disk

    CTRL + R     : toggle recording of simulation (replacing any previous)
    CTRL + P     : start replaying last recorded simulation
    CTRL + +     : increments the start time of the replay by 1 second (+SHIFT = 10 seconds)
    CTRL + -     : decrements the start time of the replay by 1 second (+SHIFT = 10 seconds)

    F1           : toggle HUD
    H/?          : toggle help
    ESC          : quit
"""

class ManualControl:
    def __init__(self, vehicle, traffic_manager):
        self.vehicle = vehicle
        self.autopilot = False
        self.manual_transmission = False
        self.constant_velocity_mode = False
        self.traffic_manager = traffic_manager
        self.current_gear = 1  # start from first gear

    def handle_key_press(self, key, mod):
        if key == pygame.K_w:
            self.vehicle.apply_control(carla.VehicleControl(throttle=1.0))
        elif key == pygame.K_s:
            self.vehicle.apply_control(carla.VehicleControl(brake=1.0))
        elif key == pygame.K_a:
            self.vehicle.apply_control(carla.VehicleControl(steer=-1.0))
        elif key == pygame.K_d:
            self.vehicle.apply_control(carla.VehicleControl(steer=1.0))
        elif key == pygame.K_q:
            self.vehicle.apply_control(carla.VehicleControl(reverse=True))
        elif key == pygame.K_SPACE:
            self.vehicle.apply_control(carla.VehicleControl(hand_brake=True))
        elif key == pygame.K_p:
            self.toggle_autopilot()
        elif key == pygame.K_m:
            self.toggle_manual_transmission()
        elif key == pygame.K_COMMA:
            self.shift_gear(-1)
        elif key == pygame.K_PERIOD:
            self.shift_gear(1)
        elif (key == pygame.K_w) and (mod & pygame.KMOD_CTRL):
            self.toggle_constant_velocity_mode()

    def toggle_autopilot(self):
        self.autopilot = not self.autopilot
        self.vehicle.set_autopilot(self.autopilot, self.traffic_manager.get_port())

        if self.autopilot:
            print("Autopilot ON (using Traffic Manager with safety settings)")

            # Traffic Manager Safety Settings
            self.traffic_manager.ignore_lights_percentage(self.vehicle, 0)  # obey all lights
            self.traffic_manager.ignore_signs_percentage(self.vehicle, 0)   # obey all signs
            self.traffic_manager.ignore_vehicles_percentage(self.vehicle, 0)  # respect all vehicles
            self.traffic_manager.ignore_walkers_percentage(self.vehicle, 0)   # respect all pedestrians

            # Allow lane changes, but don't force them
            self.traffic_manager.auto_lane_change(self.vehicle, True)  # allow lane changes, controlled

            # Set vehicle following distance and speed settings
            self.traffic_manager.distance_to_leading_vehicle(self.vehicle, 6.0)  # keep 6m gap

            # Set the vehicle's speed behavior (driving slower than the speed limit)
            self.traffic_manager.vehicle_percentage_speed_difference(self.vehicle, 30)  # 30% slower

            # Global settings to ensure consistency across the simulation
            self.traffic_manager.set_global_distance_to_leading_vehicle(6.0)  
            self.traffic_manager.set_synchronous_mode(True)  # ensure sync in simulation
            self.traffic_manager.set_random_device_seed(0)  # deterministic behavior

        else:
            print("Autopilot OFF")

    def toggle_manual_transmission(self):
        self.manual_transmission = not self.manual_transmission
        print(f"Manual Transmission {'ON' if self.manual_transmission else 'OFF'}")

    def shift_gear(self, direction):
        if self.manual_transmission:
            current_control = self.vehicle.get_control()
            if direction == 1:
                self.current_gear += 1
            elif direction == -1:
                self.current_gear = max(0, self.current_gear - 1)

            current_control.manual_gear_shift = True
            current_control.gear = self.current_gear
            self.vehicle.apply_control(current_control)
            print(f"Shifted to gear {self.current_gear}")
        else:
            print("Enable manual transmission (press M) to shift gears manually.")

    def toggle_constant_velocity_mode(self):
        self.constant_velocity_mode = not self.constant_velocity_mode
        if self.constant_velocity_mode:
            self.vehicle.enable_constant_velocity(carla.Vector3D(16.66, 0, 0))  # 60 km/h ≈ 16.66 m/s
            print("Constant velocity mode ON (60 km/h)")
        else:
            self.vehicle.disable_constant_velocity()
            print("Constant velocity mode OFF")

class DisplayManager:
    def __init__(self, grid_size):
        pygame.init()
        self.display = pygame.display.set_mode((grid_size[0]*320, grid_size[1]*240))
        pygame.display.set_caption("Sensor Data")
        self.grid_size = grid_size
        self.sensors = []

    def attach_sensor(self, sensor_manager, position):
        self.sensors.append((sensor_manager, position))

    def render(self):
        for sensor, position in self.sensors:
            if sensor.surface is not None:
                self.display.blit(sensor.surface, (position[0]*320, position[1]*240))
        pygame.display.flip()

class SensorManager:
    def __init__(self, world, display_manager, sensor_type, transform, attached_vehicle, attributes=None, display_pos=(0,0)):
        blueprint_library = world.get_blueprint_library()
        self.sensor_type = sensor_type

        if sensor_type == 'RGB':
            bp = blueprint_library.find('sensor.camera.rgb')
            bp.set_attribute('image_size_x', '320')
            bp.set_attribute('image_size_y', '240')
            bp.set_attribute('fov', '90')
        elif sensor_type == 'Depth':
            bp = blueprint_library.find('sensor.camera.depth')
            bp.set_attribute('image_size_x', '320')
            bp.set_attribute('image_size_y', '240')
            bp.set_attribute('fov', '90')
        elif sensor_type == 'Semantic':
            bp = blueprint_library.find('sensor.camera.semantic_segmentation')
            bp.set_attribute('image_size_x', '320')
            bp.set_attribute('image_size_y', '240')
            bp.set_attribute('fov', '90')
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
        else:
            raise ValueError(f"Unknown sensor type: {sensor_type}")

        if attributes:
            for attr_name, attr_value in attributes.items():
                bp.set_attribute(attr_name, attr_value)

        self.sensor = world.spawn_actor(bp, transform, attach_to=attached_vehicle)
        self.surface = None
        self.data = None

        self.sensor.listen(lambda data: self._parse_image(data))
        display_manager.attach_sensor(self, display_pos)

    def _parse_image(self, image):
        if self.sensor_type in ['RGB', 'Depth', 'Semantic']:
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (image.height, image.width, 4))
            array = array[:, :, :3]
            array = array[:, :, ::-1]
            self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
            self.data = array
        else:
            self.data = image

# Connect to CARLA
client = carla.Client('localhost', 2000)
client.set_timeout(10.0)
world = client.get_world()

traffic_manager = client.get_trafficmanager(8000)
traffic_manager.set_synchronous_mode(True)  # Optional: only if your simulation is synchronous

blueprint_library = world.get_blueprint_library()
vehicle_bp = random.choice(blueprint_library.filter('vehicle.*'))
spawn_point = random.choice(world.get_map().get_spawn_points())
vehicle = world.spawn_actor(vehicle_bp, spawn_point)

# Manual Control Setup
manual_control = ManualControl(vehicle, traffic_manager)

# Sensor Setup
display_manager = DisplayManager(grid_size=(3, 2))

camera_rgb = SensorManager(world, display_manager, 'RGB', carla.Transform(carla.Location(x=1.5, z=2.4)), vehicle, display_pos=(0, 0))
camera_depth = SensorManager(world, display_manager, 'Depth', carla.Transform(carla.Location(x=1.5, z=2.4)), vehicle, display_pos=(1, 0))
camera_semantic = SensorManager(world, display_manager, 'Semantic', carla.Transform(carla.Location(x=1.5, z=2.4)), vehicle, display_pos=(0, 1))
lidar_sensor = SensorManager(world, display_manager, 'LIDAR', carla.Transform(carla.Location(x=0, z=2.5)), vehicle, display_pos=(1, 1))

imu_sensor = SensorManager(world, display_manager, 'IMU', carla.Transform(carla.Location()), vehicle, display_pos=(2, 0))
gnss_sensor = SensorManager(world, display_manager, 'GNSS', carla.Transform(carla.Location()), vehicle, display_pos=(2, 1))

# CSV Data Saving
csv_file = open('carla_sensor_data.csv', mode='w', newline='')
csv_writer = csv.writer(csv_file)
csv_writer.writerow(['timestamp', 'speed_kmph', 'vehicle_x', 'vehicle_y', 'vehicle_z',
                     'gnss_latitude', 'gnss_longitude', 'gnss_altitude',
                     'imu_acc_x', 'imu_acc_y', 'imu_acc_z',
                     'imu_gyro_x', 'imu_gyro_y', 'imu_gyro_z'])

clock = pygame.time.Clock()

try:
    running = True
    while running:
        # Handle pygame events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                manual_control.handle_key_press(event.key, event.mod)

        world.tick()
        display_manager.render()

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
        ])

        clock.tick(30)

except KeyboardInterrupt:
    print('Simulation interrupted by user.')
finally:
    csv_file.close()
    vehicle.destroy()
    for sensor in world.get_actors().filter('*sensor*'):
        sensor.destroy()
    pygame.quit()
