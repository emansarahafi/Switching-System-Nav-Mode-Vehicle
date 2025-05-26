import os
import random
import numpy as np
import pygame
import carla
import csv
import time
from collections import deque
from datetime import datetime
import rospy
from std_msgs.msg import Float32MultiArray, String

class ManualControl:
    def __init__(self, vehicle, traffic_manager):
        self.vehicle = vehicle
        self.autopilot = False
        self.manual_transmission = False
        self.constant_velocity_mode = False
        self.traffic_manager = traffic_manager
        self.current_gear = 1
        self.control_history = deque(maxlen=30)
        self.driver_presence = True
        self.eye_gaze = (0, 0)
        self.head_pose = (0, 0, 0)
        self.save_images = False
        self.session_id = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.create_data_directories()

    def create_data_directories(self):
        base_dir = f"data/{self.session_id}/images"
        for cam_type in ['rgb', 'depth', 'semantic']:
            for side in ['front', 'rear', 'left', 'right']:
                os.makedirs(f"{base_dir}/{cam_type}/{side}", exist_ok=True)

    def handle_key_press(self, key, mod):
        control = self.vehicle.get_control()
        
        if key == pygame.K_w:
            control.throttle = 1.0
        elif key == pygame.K_s:
            control.brake = 1.0
        elif key == pygame.K_a:
            control.steer = -1.0
        elif key == pygame.K_d:
            control.steer = 1.0
        elif key == pygame.K_q:
            control.reverse = not control.reverse
        elif key == pygame.K_SPACE:
            control.hand_brake = not control.hand_brake
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
        elif key == pygame.K_r:
            pass  # Handled in main loop
        
        self.vehicle.apply_control(control)
        self.control_history.append(control)
        
        self.eye_gaze = (random.uniform(-0.1, 0.1), random.uniform(-0.1, 0.1))
        self.head_pose = (random.uniform(-5, 5), random.uniform(-10, 10), random.uniform(-2, 2))

    def toggle_autopilot(self):
        if self.manual_transmission:
            self.toggle_manual_transmission()

        self.autopilot = not self.autopilot
        self.vehicle.set_autopilot(self.autopilot, self.traffic_manager.get_port())
        print(f"Autopilot {'ON' if self.autopilot else 'OFF'}")

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
        pygame.display.set_caption("CARLA Front View + LIDAR")
        self.grid_size = grid_size
        self.sensors = []

    def attach_sensor(self, sensor_manager, position):
        self.sensors.append((sensor_manager, position))

    def render(self):
        self.display.fill((0, 0, 0))
        for sensor, position in self.sensors:
            if sensor.surface is not None:
                self.display.blit(sensor.surface, (position[0]*320, position[1]*240))
        pygame.display.flip()

class SensorManager:
    def __init__(self, world, display_manager, sensor_type, transform, attached_vehicle, camera_position="front", attributes=None, display_pos=(0,0)):
        self.world = world
        self.display_manager = display_manager
        self.sensor_type = sensor_type.lower()
        self.camera_position = camera_position
        self.save_images = False
        self.frame_count = 0
        self.display_pos = display_pos
        self.surface = None
        self.data = None
        self.combined_surface = None
        
        self.initialize_sensor(transform, attached_vehicle, attributes)
        
        if display_pos is not None:
            display_manager.attach_sensor(self, display_pos)

    def initialize_sensor(self, transform, attached_vehicle, attributes):
        blueprint_library = self.world.get_blueprint_library()
        
        # Get the correct blueprint
        if self.sensor_type == 'rgb':
            bp = blueprint_library.find('sensor.camera.rgb')
        elif self.sensor_type == 'depth':
            bp = blueprint_library.find('sensor.camera.depth')
        elif self.sensor_type == 'semantic':
            bp = blueprint_library.find('sensor.camera.semantic_segmentation')
        elif self.sensor_type == 'lidar':
            bp = blueprint_library.find('sensor.lidar.ray_cast')
        elif self.sensor_type == 'imu':
            bp = blueprint_library.find('sensor.other.imu')
        elif self.sensor_type == 'gnss':
            bp = blueprint_library.find('sensor.other.gnss')
        else:
            raise ValueError(f"Unknown sensor type: {self.sensor_type}")

        # Set default attributes
        if self.sensor_type in ['rgb', 'depth', 'semantic']:
            bp.set_attribute('image_size_x', '320')
            bp.set_attribute('image_size_y', '240')
            bp.set_attribute('fov', '90')

        if self.sensor_type == 'lidar':
            bp.set_attribute('range', '50')
            bp.set_attribute('rotation_frequency', '10')
            bp.set_attribute('channels', '64')
            bp.set_attribute('points_per_second', '56000')

        # Set custom attributes if provided
        if attributes:
            for attr_name, attr_value in attributes.items():
                bp.set_attribute(attr_name, str(attr_value))

        # Spawn the sensor
        self.sensor = self.world.spawn_actor(bp, transform, attach_to=attached_vehicle)
        
        # Set up listeners based on sensor type
        if self.sensor_type in ['rgb', 'depth', 'semantic']:
            self.sensor.listen(self._parse_image)
            self.detected_objects = []
            self.lane_offset = 0.0
            self.traffic_light_state = "UNKNOWN"
        elif self.sensor_type == 'lidar':
            self.sensor.listen(self._parse_lidar)
            self.obstacle_stats = {
                'front_min': 0, 'front_avg': 0,
                'left_min': 0, 'left_avg': 0,
                'right_min': 0, 'right_avg': 0,
                'rear_min': 0, 'rear_avg': 0,
                'point_density': 0
            }
        else:
            self.sensor.listen(self._parse_other)

    def _parse_image(self, image):
        try:
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (image.height, image.width, 4))
            array = array[:, :, :3]
            
            # Save image if enabled
            if self.save_images:
                self.save_image_data(image, array)
            
            # Process for display
            self.process_image_for_display(array)
            
            # Generate random detection data
            self.generate_detection_data()
            
        except Exception as e:
            print(f"Error processing {self.sensor_type} image: {e}")

    def save_image_data(self, image, array):
        try:
            filename = f"data/{manual_control.session_id}/images/{self.sensor_type}/{self.camera_position}/{image.frame:08d}.png"
            os.makedirs(os.path.dirname(filename), exist_ok=True)
            
            if self.sensor_type == 'depth':
                save_array = (array[:,:,0] / 256).astype(np.uint8)
            elif self.sensor_type == 'semantic':
                save_array = array[:,:,2]
            else:  # RGB
                save_array = array[:, :, ::-1]  # Convert BGR to RGB
            
            pygame_surface = pygame.surfarray.make_surface(save_array.swapaxes(0,1))
            pygame.image.save(pygame_surface, filename)
            self.frame_count += 1
            if self.frame_count % 10 == 0:
                print(f"Saved {filename}")
        except Exception as e:
            print(f"Error saving {self.sensor_type} image: {e}")

    def process_image_for_display(self, array):
        display_array = array[:, :, ::-1] if self.sensor_type == 'rgb' else array
        self.surface = pygame.surfarray.make_surface(display_array.swapaxes(0,1))
        self.data = array

    def generate_detection_data(self):
        self.detected_objects = []
        if random.random() > 0.7:
            self.detected_objects = [{
                'type': random.choice(['vehicle', 'pedestrian', 'bicycle']),
                'x': random.uniform(-10, 10),
                'y': random.uniform(2, 50),
                'width': random.uniform(1, 3),
                'height': random.uniform(1, 3)
            } for _ in range(random.randint(0, 3))]
        
        self.lane_offset = random.uniform(-0.5, 0.5)
        self.traffic_light_state = random.choice(["RED", "GREEN", "YELLOW", "UNKNOWN"])

    def _parse_lidar(self, lidar_data):
        try:
            points = np.frombuffer(lidar_data.raw_data, dtype=np.dtype('f4'))
            points = np.reshape(points, (int(points.shape[0]/4), 4))
            
            # Subsample for visualization
            vis_points = np.copy(points[::10])
            
            # Analyze different regions
            front_points = points[(points[:,1]>-2)&(points[:,1]<2)&(points[:,0]>0)]
            rear_points = points[(points[:,1]>-2)&(points[:,1]<2)&(points[:,0]<0)]
            left_points = points[(points[:,1]>2)&(points[:,0]>0)]
            right_points = points[(points[:,1]<-2)&(points[:,0]>0)]
            
            # Update obstacle stats
            self.update_obstacle_stats(front_points, rear_points, left_points, right_points, points)
            
            # Create visualization if points exist
            if len(vis_points) > 0:
                self.create_lidar_visualization(vis_points)
                
        except Exception as e:
            print(f"Error processing LIDAR data: {e}")

    def update_obstacle_stats(self, front_points, rear_points, left_points, right_points, all_points):
        self.obstacle_stats = {
            'front_min': np.min(front_points[:,0]) if len(front_points)>0 else 999,
            'front_avg': np.mean(front_points[:,0]) if len(front_points)>0 else 999,
            'rear_min': np.min(rear_points[:,0]) if len(rear_points)>0 else 999,
            'rear_avg': np.mean(rear_points[:,0]) if len(rear_points)>0 else 999,
            'left_min': np.min(left_points[:,0]) if len(left_points)>0 else 999,
            'left_avg': np.mean(left_points[:,0]) if len(left_points)>0 else 999,
            'right_min': np.min(right_points[:,0]) if len(right_points)>0 else 999,
            'right_avg': np.mean(right_points[:,0]) if len(right_points)>0 else 999,
            'point_density': len(all_points)/1000
        }

    def create_lidar_visualization(self, points):
        lidar_image = np.zeros((240,320,3), dtype=np.uint8)
        
        # Convert points to image coordinates
        points[:,0] = (points[:,0]/50*320).clip(0,319)
        points[:,1] = (points[:,1]/20*120+120).clip(0,239)
        
        # Color points based on position
        for x,y,z,i in points:
            color = (255,255,255)  # White
            if y < 100: color = (255,0,0)    # Red for right side
            elif y > 140: color = (0,0,255)  # Blue for left side
            elif x < 100: color = (0,255,0)  # Green for close objects
            x,y = int(x),int(y)
            if 0<=x<320 and 0<=y<240:
                lidar_image[y,x] = color
        
        self.surface = pygame.surfarray.make_surface(lidar_image)

    def _parse_other(self, data):
        self.data = data

class ROSPublisher:
    def __init__(self):
        rospy.init_node('carla_data_publisher', anonymous=True)
        self.vehicle_data_pub = rospy.Publisher('/carla/vehicle_data', Float32MultiArray, queue_size=10)
        self.sensor_data_pub = rospy.Publisher('/carla/sensor_data', Float32MultiArray, queue_size=10)
        self.status_pub = rospy.Publisher('/carla/status', String, queue_size=10)
        
    def publish_vehicle_data(self, vehicle_data):
        msg = Float32MultiArray()
        msg.data = vehicle_data
        self.vehicle_data_pub.publish(msg)
        
    def publish_sensor_data(self, sensor_data):
        msg = Float32MultiArray()
        msg.data = sensor_data
        self.sensor_data_pub.publish(msg)
        
    def publish_status(self, status):
        self.status_pub.publish(String(status))

# Main execution
try:
    # Initialize ROS publisher
    ros_publisher = ROSPublisher()
    ros_publisher.publish_status("CARLA simulation starting...")
    
    # Initialize CARLA
    client = carla.Client('192.168.31.28', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    traffic_manager = client.get_trafficmanager(8000)
    traffic_manager.set_synchronous_mode(True)

    # Create vehicle
    blueprint_library = world.get_blueprint_library()
    vehicle_bp = random.choice(blueprint_library.filter('vehicle.*'))
    spawn_point = random.choice(world.get_map().get_spawn_points())
    vehicle = world.spawn_actor(vehicle_bp, spawn_point)

    # Setup control system
    manual_control = ManualControl(vehicle, traffic_manager)
    display_manager = DisplayManager(grid_size=(2, 1))

    # Create all cameras
    cameras = []
    for side, transform in [
        ('front', carla.Transform(carla.Location(x=1.5, z=2.4))),
        ('rear', carla.Transform(carla.Location(x=-1.5, z=2.4), carla.Rotation(yaw=180))),
        ('left', carla.Transform(carla.Location(y=-1.0, z=2.4), carla.Rotation(yaw=-90))),
        ('right', carla.Transform(carla.Location(y=1.0, z=2.4), carla.Rotation(yaw=90)))
    ]:
        for cam_type in ['rgb', 'depth', 'semantic']:
            try:
                display_pos = None
                if side == 'front' and cam_type == 'rgb':
                    display_pos = (0, 0)
                
                cam = SensorManager(world, display_manager, cam_type, transform, vehicle, side, display_pos=display_pos)
                cameras.append(cam)
                print(f"Created {cam_type} camera at {side} position")
            except Exception as e:
                print(f"Failed to create {cam_type} camera at {side} position: {e}")

    # Create LIDAR sensor
    lidar_sensor = SensorManager(world, display_manager, 'lidar',
                               carla.Transform(carla.Location(z=2.5)),
                               vehicle, display_pos=(1, 0))

    # Create other sensors
    imu_sensor = SensorManager(world, display_manager, 'imu',
                             carla.Transform(carla.Location()),
                             vehicle, display_pos=None)

    gnss_sensor = SensorManager(world, display_manager, 'gnss',
                              carla.Transform(carla.Location()),
                              vehicle, display_pos=None)

    # Create CSV file
    csv_path = f"data/{manual_control.session_id}/sensor_data.csv"
    os.makedirs(os.path.dirname(csv_path), exist_ok=True)
    csv_file = open(csv_path, mode='w', newline='')
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow([
        'timestamp', 'frame', 'speed_kmph', 'vehicle_x', 'vehicle_y', 'vehicle_z',
        'throttle', 'brake', 'steer', 'reverse', 'hand_brake', 'gear',
        'autopilot', 'manual_transmission', 'constant_velocity',
        'driver_present', 'eye_gaze_x', 'eye_gaze_y', 'head_pitch', 'head_yaw', 'head_roll',
        'front_objects', 'rear_objects', 'left_objects', 'right_objects',
        'lane_offset', 'traffic_light_state',
        'lidar_front_min', 'lidar_front_avg', 'lidar_left_min', 'lidar_left_avg',
        'lidar_right_min', 'lidar_right_avg', 'lidar_rear_min', 'lidar_rear_avg', 'lidar_point_density',
        'gnss_lat', 'gnss_lon', 'gnss_alt',
        'imu_acc_x', 'imu_acc_y', 'imu_acc_z', 'imu_gyro_x', 'imu_gyro_y', 'imu_gyro_z'
    ])

    clock = pygame.time.Clock()

    running = True
    while running and not rospy.is_shutdown():
        for event in pygame.event.get():
            if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_r:
                    manual_control.save_images = not manual_control.save_images
                    for cam in cameras:
                        cam.save_images = manual_control.save_images
                    print(f"Image saving {'ON' if manual_control.save_images else 'OFF'}")
                manual_control.handle_key_press(event.key, event.mod)

        world.tick()
        display_manager.render()

        # Get current simulation data
        velocity = vehicle.get_velocity()
        speed = 3.6 * np.linalg.norm([velocity.x, velocity.y, velocity.z])
        transform = vehicle.get_transform()
        control = vehicle.get_control()
        frame = world.get_snapshot().frame

        # Prepare data for CSV and ROS
        current_time = time.time()
        
        # Vehicle data for ROS
        vehicle_data = [
            current_time, frame, speed,
            transform.location.x, transform.location.y, transform.location.z,
            control.throttle, control.brake, control.steer,
            float(control.reverse), float(control.hand_brake), control.gear,
            float(manual_control.autopilot), float(manual_control.manual_transmission), 
            float(manual_control.constant_velocity_mode),
            float(manual_control.driver_presence), manual_control.eye_gaze[0], manual_control.eye_gaze[1],
            manual_control.head_pose[0], manual_control.head_pose[1], manual_control.head_pose[2]
        ]
        
        # Sensor data for ROS
        front_cam = [c for c in cameras if c.camera_position=='front' and c.sensor_type=='rgb'][0]
        sensor_data = [
            len(front_cam.detected_objects),
            len([c for c in cameras if c.camera_position=='rear' and c.sensor_type=='rgb'][0].detected_objects),
            len([c for c in cameras if c.camera_position=='left' and c.sensor_type=='rgb'][0].detected_objects),
            len([c for c in cameras if c.camera_position=='right' and c.sensor_type=='rgb'][0].detected_objects),
            front_cam.lane_offset,
            0 if front_cam.traffic_light_state == "RED" else 
            1 if front_cam.traffic_light_state == "GREEN" else 
            2 if front_cam.traffic_light_state == "YELLOW" else 3,
            lidar_sensor.obstacle_stats['front_min'], lidar_sensor.obstacle_stats['front_avg'],
            lidar_sensor.obstacle_stats['left_min'], lidar_sensor.obstacle_stats['left_avg'],
            lidar_sensor.obstacle_stats['right_min'], lidar_sensor.obstacle_stats['right_avg'],
            lidar_sensor.obstacle_stats['rear_min'], lidar_sensor.obstacle_stats['rear_avg'],
            lidar_sensor.obstacle_stats['point_density'],
            gnss_sensor.data.latitude if gnss_sensor.data else 0,
            gnss_sensor.data.longitude if gnss_sensor.data else 0,
            gnss_sensor.data.altitude if gnss_sensor.data else 0,
            imu_sensor.data.accelerometer.x if imu_sensor.data else 0,
            imu_sensor.data.accelerometer.y if imu_sensor.data else 0,
            imu_sensor.data.accelerometer.z if imu_sensor.data else 0,
            imu_sensor.data.gyroscope.x if imu_sensor.data else 0,
            imu_sensor.data.gyroscope.y if imu_sensor.data else 0,
            imu_sensor.data.gyroscope.z if imu_sensor.data else 0
        ]

        # Write to CSV
        csv_writer.writerow([current_time, frame, speed] + vehicle_data[3:] + sensor_data[0:6] + 
                           sensor_data[7:25] + [sensor_data[6]] + sensor_data[25:])
        
        # Publish to ROS
        ros_publisher.publish_vehicle_data(vehicle_data)
        ros_publisher.publish_sensor_data(sensor_data)
        
        clock.tick(30)

except Exception as e:
    ros_publisher.publish_status(f"Error in CARLA simulation: {str(e)}")
    print(f"Error during execution: {e}")
finally:
    ros_publisher.publish_status("CARLA simulation shutting down...")
    print("Cleaning up...")
    try:
        csv_file.close()
    except:
        pass
    
    try:
        for cam in cameras:
            cam.sensor.destroy()
    except:
        pass
    
    try:
        lidar_sensor.sensor.destroy()
    except:
        pass
    
    try:
        imu_sensor.sensor.destroy()
    except:
        pass
    
    try:
        gnss_sensor.sensor.destroy()
    except:
        pass
    
    try:
        vehicle.destroy()
    except:
        pass
    
    pygame.quit()
    print(f"Data saved to: data/{manual_control.session_id if 'manual_control' in locals() else 'unknown_session'}")