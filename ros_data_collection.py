#!/usr/bin/env python

import glob
import os
import sys
import random
import numpy as np
import pygame
import carla
import csv
import time
import cv2
from tf.transformations import quaternion_from_euler

# ROS imports
import rospy
from sensor_msgs.msg import NavSatFix, Imu, Image, PointCloud2
from geometry_msgs.msg import TwistStamped, PoseStamped
from std_msgs.msg import Header
from cv_bridge import CvBridge
from carla import Transform, Location, Rotation

class ManualControl:
    def __init__(self, vehicle, traffic_manager):
        self.vehicle = vehicle
        self.autopilot = False
        self.manual_transmission = False
        self.constant_velocity_mode = False
        self.traffic_manager = traffic_manager
        self.current_gear = 1

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
        if self.manual_transmission:
            self.toggle_manual_transmission()

        self.autopilot = not self.autopilot
        self.vehicle.set_autopilot(self.autopilot, self.traffic_manager.get_port())
        if self.autopilot:
            print("Autopilot ON")
            self.traffic_manager.ignore_lights_percentage(self.vehicle, 0)
            self.traffic_manager.ignore_signs_percentage(self.vehicle, 0)
            self.traffic_manager.ignore_vehicles_percentage(self.vehicle, 0)
            self.traffic_manager.ignore_walkers_percentage(self.vehicle, 0)
            self.traffic_manager.auto_lane_change(self.vehicle, True)
            self.traffic_manager.distance_to_leading_vehicle(self.vehicle, 6.0)
            self.traffic_manager.vehicle_percentage_speed_difference(self.vehicle, 30)
            self.traffic_manager.set_global_distance_to_leading_vehicle(6.0)
            self.traffic_manager.set_synchronous_mode(True)
            self.traffic_manager.set_random_device_seed(0)
        else:
            print("Autopilot OFF")

    def toggle_manual_transmission(self):
        if self.autopilot:
            self.toggle_autopilot()

        self.manual_transmission = not self.manual_transmission
        print(f"Manual Transmission {'ON' if self.manual_transmission else 'OFF'}")

    def shift_gear(self, direction):
        if self.manual_transmission:
            control = self.vehicle.get_control()
            if direction == 1:
                self.current_gear += 1
            elif direction == -1:
                self.current_gear = max(0, self.current_gear - 1)
            control.manual_gear_shift = True
            control.gear = self.current_gear
            self.vehicle.apply_control(control)
            print(f"Shifted to gear {self.current_gear}")
        else:
            print("Manual transmission is not enabled. Press 'M' to enable.")

    def toggle_constant_velocity_mode(self):
        self.constant_velocity_mode = not self.constant_velocity_mode
        if self.constant_velocity_mode:
            self.vehicle.enable_constant_velocity(carla.Vector3D(16.66, 0, 0))
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
    def __init__(self, world, display_manager, sensor_type, transform, attached_vehicle, attributes=None, display_pos=None, topic_suffix=""):
        blueprint_library = world.get_blueprint_library()
        self.sensor_type = sensor_type
        self.topic_suffix = topic_suffix

        if sensor_type == 'RGB':
            bp = blueprint_library.find('sensor.camera.rgb')
            bp.set_attribute('image_size_x', '640')
            bp.set_attribute('image_size_y', '480')
            bp.set_attribute('fov', '90')
        elif sensor_type == 'Depth':
            bp = blueprint_library.find('sensor.camera.depth')
            bp.set_attribute('image_size_x', '640')
            bp.set_attribute('image_size_y', '480')
            bp.set_attribute('fov', '90')
        elif sensor_type == 'Semantic':
            bp = blueprint_library.find('sensor.camera.semantic_segmentation')
            bp.set_attribute('image_size_x', '640')
            bp.set_attribute('image_size_y', '480')
            bp.set_attribute('fov', '90')
        elif sensor_type == 'LIDAR':
            bp = blueprint_library.find('sensor.lidar.ray_cast')
            bp.set_attribute('range', '50')
            bp.set_attribute('rotation_frequency', '20')
            bp.set_attribute('channels', '64')
            bp.set_attribute('points_per_second', '100000')
        elif sensor_type == 'IMU':
            bp = blueprint_library.find('sensor.other.imu')
        elif sensor_type == 'GNSS':
            bp = blueprint_library.find('sensor.other.gnss')
            bp.set_attribute('noise_alt_stddev', '0.0')
            bp.set_attribute('noise_lat_stddev', '0.0')
            bp.set_attribute('noise_lon_stddev', '0.0')
        elif sensor_type == 'Radar':
            bp = blueprint_library.find('sensor.other.radar')
            bp.set_attribute('horizontal_fov', '30')
            bp.set_attribute('vertical_fov', '30')
            bp.set_attribute('range', '50')
        else:
            raise ValueError(f"Unknown sensor type: {sensor_type}")

        if attributes:
            for attr_name, attr_value in attributes.items():
                bp.set_attribute(attr_name, attr_value)

        self.sensor = world.spawn_actor(bp, transform, attach_to=attached_vehicle)
        self.surface = None
        self.data = None

        self.sensor.listen(lambda data: self._parse_image(data))
        if display_manager and display_pos:
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

def setup_sensors(world, vehicle, display_manager=None):
    """Configure and return all sensors with multi-directional coverage"""
    sensors = []
    
    # Camera configurations (front, rear, left, right)
    camera_positions = [
        ('front', Transform(Location(x=1.5, z=2.4), Rotation())),
        ('rear', Transform(Location(x=-1.5, z=2.4), Rotation(yaw=180))),
        ('left', Transform(Location(y=-1.0, z=2.4), Rotation(yaw=-90))),
        ('right', Transform(Location(y=1.0, z=2.4), Rotation(yaw=90)))
    ]
    
    # Add cameras
    for suffix, transform in camera_positions:
        sensors.append(
            SensorManager(world, display_manager, 'RGB', transform, vehicle, 
                        display_pos=None, topic_suffix=f"_{suffix}")
    )
    
    # Depth and Semantic (front only)
    sensors.append(
        SensorManager(world, display_manager, 'Depth', 
                    Transform(Location(x=1.5, z=2.4)), 
                    vehicle, display_pos=(1, 0))
    )
    sensors.append(
        SensorManager(world, display_manager, 'Semantic', 
                    Transform(Location(x=1.5, z=2.4)), 
                    vehicle, display_pos=(0, 1))
    )
    
    # LIDAR (360-degree coverage)
    sensors.append(
        SensorManager(world, display_manager, 'LIDAR',
                    Transform(Location(z=2.5)), 
                    vehicle, display_pos=(1, 1))
    )
    
    # IMU and GNSS
    sensors.append(
        SensorManager(world, None, 'IMU', 
                    Transform(), vehicle)
    )
    sensors.append(
        SensorManager(world, None, 'GNSS',
                    Transform(), vehicle)
    )
    
    return sensors

def create_publishers():
    """Create ROS publishers for all sensors"""
    pubs = {
        'gnss': rospy.Publisher('/carla/gnss', NavSatFix, queue_size=10),
        'imu': rospy.Publisher('/carla/imu', Imu, queue_size=10),
        'speed': rospy.Publisher('/carla/speed', TwistStamped, queue_size=10),
        'pose': rospy.Publisher('/carla/pose', PoseStamped, queue_size=10),
        'lidar': rospy.Publisher('/carla/lidar', PointCloud2, queue_size=10),
        'depth': rospy.Publisher('/carla/depth', Image, queue_size=10),
        'semantic': rospy.Publisher('/carla/semantic', Image, queue_size=10)
    }
    
    # Add camera publishers
    for direction in ['front', 'rear', 'left', 'right']:
        pubs[f'rgb_{direction}'] = rospy.Publisher(
            f'/carla/rgb_{direction}', Image, queue_size=10)
    
    return pubs

def publish_sensor_data(pubs, sensors, vehicle, bridge):
    """Publish all sensor data to ROS"""
    current_time = rospy.Time.now()
    
    # Vehicle data
    velocity = vehicle.get_velocity()
    transform = vehicle.get_transform()
    location = transform.location
    rotation = transform.rotation
    
    # Publish vehicle state
    twist_msg = TwistStamped()
    twist_msg.header = Header(stamp=current_time, frame_id="carla")
    twist_msg.twist.linear.x = velocity.x
    twist_msg.twist.linear.y = velocity.y
    twist_msg.twist.linear.z = velocity.z
    pubs['speed'].publish(twist_msg)
    
    pose_msg = PoseStamped()
    pose_msg.header = Header(stamp=current_time, frame_id="carla")
    pose_msg.pose.position.x = location.x
    pose_msg.pose.position.y = location.y
    pose_msg.pose.position.z = location.z
    q = quaternion_from_euler(
        np.radians(rotation.roll),
        np.radians(rotation.pitch),
        np.radians(rotation.yaw)
    )
    pose_msg.pose.orientation.x = q[0]
    pose_msg.pose.orientation.y = q[1]
    pose_msg.pose.orientation.z = q[2]
    pose_msg.pose.orientation.w = q[3]
    pubs['pose'].publish(pose_msg)
    
    # Publish sensor data
    for sensor in sensors:
        if sensor.data is None:
            continue
            
        if sensor.sensor_type == 'RGB':
            img_msg = bridge.cv2_to_imgmsg(sensor.data, encoding="bgr8")
            img_msg.header = Header(stamp=current_time, frame_id="carla")
            pubs[f'rgb_{sensor.topic_suffix[1:]}'].publish(img_msg)
        elif sensor.sensor_type == 'Depth':
            img_msg = bridge.cv2_to_imgmsg(cv2.cvtColor(sensor.data, cv2.COLOR_BGR2GRAY), encoding="mono8")
            img_msg.header = Header(stamp=current_time, frame_id="carla")
            pubs['depth'].publish(img_msg)
        elif sensor.sensor_type == 'Semantic':
            img_msg = bridge.cv2_to_imgmsg(sensor.data, encoding="bgr8")
            img_msg.header = Header(stamp=current_time, frame_id="carla")
            pubs['semantic'].publish(img_msg)
        elif sensor.sensor_type == 'IMU' and hasattr(sensor.data, 'accelerometer'):
            imu_msg = Imu()
            imu_msg.header = Header(stamp=current_time, frame_id="carla")
            imu_msg.linear_acceleration.x = sensor.data.accelerometer.x
            imu_msg.linear_acceleration.y = sensor.data.accelerometer.y
            imu_msg.linear_acceleration.z = sensor.data.accelerometer.z
            imu_msg.angular_velocity.x = sensor.data.gyroscope.x
            imu_msg.angular_velocity.y = sensor.data.gyroscope.y
            imu_msg.angular_velocity.z = sensor.data.gyroscope.z
            pubs['imu'].publish(imu_msg)
        elif sensor.sensor_type == 'GNSS' and hasattr(sensor.data, 'latitude'):
            gnss_msg = NavSatFix()
            gnss_msg.header = Header(stamp=current_time, frame_id="carla")
            gnss_msg.latitude = sensor.data.latitude
            gnss_msg.longitude = sensor.data.longitude
            gnss_msg.altitude = sensor.data.altitude
            pubs['gnss'].publish(gnss_msg)

def main():
    rospy.init_node('carla_ros_bridge', anonymous=True)
    bridge = CvBridge()
    
    try:
        # Connect to CARLA
        client = carla.Client('192.168.31.244', 2000)
        client.set_timeout(10.0)
        world = client.get_world()
        
        # Setup synchronous mode
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05  # 20 FPS
        world.apply_settings(settings)
        
        # Spawn vehicle
        blueprint_library = world.get_blueprint_library()
        vehicle_bp = random.choice(blueprint_library.filter('vehicle.*'))
        spawn_point = random.choice(world.get_map().get_spawn_points())
        vehicle = world.spawn_actor(vehicle_bp, spawn_point)
        
        # Setup traffic manager
        traffic_manager = client.get_trafficmanager(8000)
        traffic_manager.set_synchronous_mode(True)
        
        # Setup manual control
        manual_control = ManualControl(vehicle, traffic_manager)
        
        # Setup display and sensors
        display_manager = DisplayManager(grid_size=(3, 2))
        sensors = setup_sensors(world, vehicle, display_manager)
        pubs = create_publishers()
        
        # Main loop
        rate = rospy.Rate(20)  # Match CARLA's 20 FPS
        while not rospy.is_shutdown():
            # Handle Pygame events
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    return
                elif event.type == pygame.KEYDOWN:
                    manual_control.handle_key_press(event.key, event.mod)
            
            # Tick CARLA world
            world.tick()
            
            # Update display
            display_manager.render()
            
            # Publish all sensor data
            publish_sensor_data(pubs, sensors, vehicle, bridge)
            
            rate.sleep()

    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        # Cleanup
        print('Destroying actors...')
        for sensor in sensors:
            if sensor.sensor.is_alive:
                sensor.sensor.destroy()
        if 'vehicle' in locals() and vehicle.is_alive:
            vehicle.destroy()
        pygame.quit()
        print('Done.')

if __name__ == '__main__':
    main()