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
from sensor_msgs.msg import NavSatFix, Imu, Image
from geometry_msgs.msg import TwistStamped, PoseStamped
from std_msgs.msg import Header
from cv_bridge import CvBridge

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

    TAB/N        : change sensor position
    [1-9]        : change to sensor [1-9]
    G            : toggle radar visualization
    C            : change weather (Shift+C reverse)
    Backspace    : change vehicle

    O            : open/close all doors
    T            : toggle vehicle telemetry

    V            : select next map layer (Shift+V reverse)
    B            : load selected map layer (Shift+B to unload)

    R            : toggle recording images to disk
    CTRL + R     : start/stop recording simulation
    CTRL + P     : replay last recording
    CTRL + +/-   : adjust replay start time

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

def main():
    # === ROS Initialization ===
    rospy.init_node('carla_ros_bridge', anonymous=True)
    bridge = CvBridge()

    # Publishers
    gnss_pub = rospy.Publisher('/carla/gnss', NavSatFix, queue_size=10)
    imu_pub = rospy.Publisher('/carla/imu', Imu, queue_size=10)
    speed_pub = rospy.Publisher('/carla/speed', TwistStamped, queue_size=10)
    pose_pub = rospy.Publisher('/carla/pose', PoseStamped, queue_size=10)
    rgb_pub = rospy.Publisher('/carla/rgb', Image, queue_size=10)
    depth_pub = rospy.Publisher('/carla/depth', Image, queue_size=10)
    semantic_pub = rospy.Publisher('/carla/semantic', Image, queue_size=10)

    # Connect to CARLA
    try:
        client = carla.Client('localhost', 2000)
        client.set_timeout(10.0)
        world = client.get_world()

        traffic_manager = client.get_trafficmanager(8000)
        traffic_manager.set_synchronous_mode(True)

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

        rate = rospy.Rate(30)  # 30Hz
        running = True

        while running and not rospy.is_shutdown():
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    manual_control.handle_key_press(event.key, event.mod)

            world.tick()
            display_manager.render()

            velocity = vehicle.get_velocity()
            speed = 3.6 * np.linalg.norm([velocity.x, velocity.y, velocity.z])

            transform = vehicle.get_transform()
            location = transform.location
            rotation = transform.rotation

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
                imu_gyro.z if imu_gyro else None
            ])

            # === ROS Publishing ===
            current_time = rospy.Time.now()

            # Publish GNSS
            if gnss_lat is not None:
                gnss_msg = NavSatFix()
                gnss_msg.header = Header(stamp=current_time, frame_id="carla")
                gnss_msg.latitude = gnss_lat
                gnss_msg.longitude = gnss_lon
                gnss_msg.altitude = gnss_alt
                gnss_pub.publish(gnss_msg)

            # Publish IMU
            if imu_acc is not None and imu_gyro is not None:
                imu_msg = Imu()
                imu_msg.header = Header(stamp=current_time, frame_id="carla")
                imu_msg.linear_acceleration.x = imu_acc.x
                imu_msg.linear_acceleration.y = imu_acc.y
                imu_msg.linear_acceleration.z = imu_acc.z
                imu_msg.angular_velocity.x = imu_gyro.x
                imu_msg.angular_velocity.y = imu_gyro.y
                imu_msg.angular_velocity.z = imu_gyro.z
                imu_pub.publish(imu_msg)

            # Publish Speed (TwistStamped)
            twist_msg = TwistStamped()
            twist_msg.header = Header(stamp=current_time, frame_id="carla")
            twist_msg.twist.linear.x = velocity.x
            twist_msg.twist.linear.y = velocity.y
            twist_msg.twist.linear.z = velocity.z
            speed_pub.publish(twist_msg)

            # Publish Pose (PoseStamped)
            pose_msg = PoseStamped()
            pose_msg.header = Header(stamp=current_time, frame_id="carla")
            pose_msg.pose.position.x = location.x
            pose_msg.pose.position.y = location.y
            pose_msg.pose.position.z = location.z
            
            # Convert Euler angles to quaternion
            q = quaternion_from_euler(
                np.radians(rotation.roll),
                np.radians(rotation.pitch),
                np.radians(rotation.yaw)
            )
            pose_msg.pose.orientation.x = q[0]
            pose_msg.pose.orientation.y = q[1]
            pose_msg.pose.orientation.z = q[2]
            pose_msg.pose.orientation.w = q[3]
            pose_pub.publish(pose_msg)

            # Publish Camera Data (RGB, Depth, Semantic)
            if camera_rgb.data is not None:
                rgb_msg = bridge.cv2_to_imgmsg(camera_rgb.data, encoding="bgr8")
                rgb_msg.header = Header(stamp=current_time, frame_id="carla")
                rgb_pub.publish(rgb_msg)

            if camera_depth.data is not None:
                depth_msg = bridge.cv2_to_imgmsg(cv2.cvtColor(camera_depth.data, cv2.COLOR_BGR2GRAY), encoding="mono8")
                depth_msg.header = Header(stamp=current_time, frame_id="carla")
                depth_pub.publish(depth_msg)

            if camera_semantic.data is not None:
                semantic_msg = bridge.cv2_to_imgmsg(camera_semantic.data, encoding="bgr8")
                semantic_msg.header = Header(stamp=current_time, frame_id="carla")
                semantic_pub.publish(semantic_msg)

            rate.sleep()

    except KeyboardInterrupt:
        print("Process interrupted. Cleaning up...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        print('Destroying actors...')
        for actor in world.get_actors().filter('*vehicle*'):
            actor.destroy()
        csv_file.close()
        pygame.quit()
        print('Done.')

if __name__ == '__main__':
    main()