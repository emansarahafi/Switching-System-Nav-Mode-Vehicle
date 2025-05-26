import os
import random
import numpy as np
import pygame
import carla
import csv
import time
import threading
import traceback
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
        if self.autopilot and key not in {pygame.K_p, pygame.K_ESCAPE}:
            print("Autopilot active - manual inputs disabled")
            return

        control = self.vehicle.get_control()
        
        if key == pygame.K_w:
            control.throttle = min(control.throttle + 0.1, 1.0)
        elif key == pygame.K_s:
            control.brake = min(control.brake + 0.1, 1.0)
        elif key == pygame.K_a:
            control.steer = max(control.steer - 0.1, -1.0)
        elif key == pygame.K_d:
            control.steer = min(control.steer + 0.1, 1.0)
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
            pass
        
        self.vehicle.apply_control(control)
        self.control_history.append(control)
        
        self.eye_gaze = (random.uniform(-0.1, 0.1), random.uniform(-0.1, 0.1))
        self.head_pose = (random.uniform(-5, 5), random.uniform(-10, 10), random.uniform(-2, 2))

    def toggle_autopilot(self):
        if self.autopilot:
            control = carla.VehicleControl()
            control.throttle = 0.0
            control.brake = 0.5
            self.vehicle.apply_control(control)
            time.sleep(0.5)
        
        self.autopilot = not self.autopilot
        try:
            self.vehicle.set_autopilot(self.autopilot, self.traffic_manager.get_port())
            print(f"Autopilot {'ON' if self.autopilot else 'OFF'} (TM Port: {self.traffic_manager.get_port()})")
        except RuntimeError as e:
            print(f"Autopilot toggle failed: {str(e)}")
            self.autopilot = False

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
            print("Manual transmission not enabled")

    def toggle_constant_velocity_mode(self):
        self.constant_velocity_mode = not self.constant_velocity_mode
        if self.constant_velocity_mode:
            self.vehicle.enable_constant_velocity(carla.Vector3D(16.66, 0, 0))
            print("Constant velocity mode ON (60 km/h)")
        else:
            self.vehicle.disable_constant_velocity()
            print("Constant velocity mode OFF")

class CollisionSensor:
    def __init__(self, vehicle):
        self.vehicle = vehicle
        self.destroyed = False
        world = vehicle.get_world()
        bp = world.get_blueprint_library().find('sensor.other.collision')
        self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=vehicle)
        self.sensor.listen(self.on_collision)

    def on_collision(self, event):
        if self.destroyed: return
        impulse = event.normal_impulse
        intensity = np.sqrt(impulse.x**2 + impulse.y**2 + impulse.z**2)
        if intensity > 500:
            print(f"Emergency stop! Collision intensity: {intensity:.2f}")
            control = self.vehicle.get_control()
            control.throttle = 0.0
            control.brake = 1.0
            self.vehicle.apply_control(control)

    def destroy(self):
        if not self.destroyed:
            self.sensor.destroy()
            self.destroyed = True

class LaneInvasionSensor:
    def __init__(self, vehicle):
        self.vehicle = vehicle
        self.destroyed = False
        world = vehicle.get_world()
        bp = world.get_blueprint_library().find('sensor.other.lane_invasion')
        self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=vehicle)
        self.sensor.listen(self.on_invasion)

    def on_invasion(self, event):
        if self.destroyed: return
        print("Lane invasion detected! Correcting trajectory...")
        control = self.vehicle.get_control()
        control.steer = -0.2 * control.steer
        self.vehicle.apply_control(control)

    def destroy(self):
        if not self.destroyed:
            self.sensor.destroy()
            self.destroyed = True

class DisplayManager:
    def __init__(self, grid_size):
        pygame.init()
        self.display = pygame.display.set_mode((grid_size[0]*320, grid_size[1]*240))
        pygame.display.set_caption("CARLA Multi-Sensor View")
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
        self.destroyed = False
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

        if self.sensor_type in ['rgb', 'depth', 'semantic']:
            bp.set_attribute('image_size_x', '320')
            bp.set_attribute('image_size_y', '240')
            bp.set_attribute('fov', '90')

        if self.sensor_type == 'lidar':
            bp.set_attribute('range', '50')
            bp.set_attribute('rotation_frequency', '10')
            bp.set_attribute('channels', '64')
            bp.set_attribute('points_per_second', '56000')

        if attributes:
            for attr_name, attr_value in attributes.items():
                bp.set_attribute(attr_name, str(attr_value))

        self.sensor = self.world.spawn_actor(bp, transform, attach_to=attached_vehicle)
        
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
        if self.destroyed: return
        try:
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (image.height, image.width, 4))
            array = array[:, :, :3]
            
            if self.save_images:
                self.save_image_data(image, array)
            
            self.process_image_for_display(array)
            self.generate_detection_data()
            
        except Exception as e:
            print(f"Image processing error: {e}")

    def save_image_data(self, image, array):
        try:
            filename = f"data/{manual_control.session_id}/images/{self.sensor_type}/{self.camera_position}/{image.frame:08d}.png"
            os.makedirs(os.path.dirname(filename), exist_ok=True)
            
            if self.sensor_type == 'depth':
                save_array = (array[:,:,0] / 256).astype(np.uint8)
            elif self.sensor_type == 'semantic':
                save_array = array[:,:,2]
            else:
                save_array = array[:, :, ::-1]

            pygame_surface = pygame.surfarray.make_surface(save_array.swapaxes(0,1))
            pygame.image.save(pygame_surface, filename)
            self.frame_count += 1
        except Exception as e:
            print(f"Image save error: {e}")

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
        if self.destroyed: return
        try:
            points = np.frombuffer(lidar_data.raw_data, dtype=np.dtype('f4'))
            points = np.reshape(points, (int(points.shape[0]/4), 4))
            
            vis_points = np.copy(points[::10])
            
            front_points = points[(points[:,1]>-2)&(points[:,1]<2)&(points[:,0]>0)]
            rear_points = points[(points[:,1]>-2)&(points[:,1]<2)&(points[:,0]<0)]
            left_points = points[(points[:,1]>2)&(points[:,0]>0)]
            right_points = points[(points[:,1]<-2)&(points[:,0]>0)]
            
            self.update_obstacle_stats(front_points, rear_points, left_points, right_points, points)
            
            if len(vis_points) > 0:
                self.create_lidar_visualization(vis_points)
                
        except Exception as e:
            print(f"LIDAR error: {e}")

    def update_obstacle_stats(self, front_points, rear_points, left_points, right_points, all_points):
        try:
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
        except:
            self.obstacle_stats = dict.fromkeys(self.obstacle_stats.keys(), 0)

    def create_lidar_visualization(self, points):
        lidar_image = np.zeros((240,320,3), dtype=np.uint8)
        
        points[:,0] = (points[:,0]/50*320).clip(0,319)
        points[:,1] = (points[:,1]/20*120+120).clip(0,239)
        
        for x,y,z,i in points:
            color = (255,255,255)
            if y < 100: color = (255,0,0)
            elif y > 140: color = (0,0,255)
            elif x < 100: color = (0,255,0)
            x,y = int(x),int(y)
            if 0<=x<320 and 0<=y<240:
                lidar_image[y,x] = color
        
        self.surface = pygame.surfarray.make_surface(lidar_image)

    def _parse_other(self, data):
        if self.destroyed: return
        self.data = data

    def destroy(self):
        if not self.destroyed:
            self.sensor.destroy()
            self.destroyed = True

class ROSPublisher:
    def __init__(self):
        rospy.init_node('carla_data_publisher', anonymous=True)
        self.lock = threading.Lock()
        self.vehicle_data_pub = rospy.Publisher('/carla/vehicle_data', Float32MultiArray, queue_size=10)
        self.sensor_data_pub = rospy.Publisher('/carla/sensor_data', Float32MultiArray, queue_size=10)
        self.status_pub = rospy.Publisher('/carla/status', String, queue_size=10)
        
    def publish_vehicle_data(self, vehicle_data):
        with self.lock:
            msg = Float32MultiArray()
            msg.data = vehicle_data
            self.vehicle_data_pub.publish(msg)
        
    def publish_sensor_data(self, sensor_data):
        with self.lock:
            msg = Float32MultiArray()
            msg.data = sensor_data
            self.sensor_data_pub.publish(msg)
        
    def publish_status(self, status):
        with self.lock:
            self.status_pub.publish(String(status))

def system_health_check(vehicle):
    try:
        if vehicle.get_velocity().length() > 50:
            control = vehicle.get_control()
            control.throttle = 0.0
            vehicle.apply_control(control)
            print("Speed limit exceeded! Engaging speed limiter")
            return False
        return True
    except:
        return False

try:
    ros_publisher = ROSPublisher()
    ros_publisher.publish_status("CARLA simulation starting...")

    client = carla.Client('192.168.1.19', 2000)
    client.set_timeout(20.0)
    world = client.get_world()
    
    # Traffic Manager Setup
    traffic_manager = client.get_trafficmanager(8000 + random.randint(0, 100))
    traffic_manager.set_synchronous_mode(True)
    traffic_manager.set_global_distance_to_leading_vehicle(3.0)
    traffic_manager.global_percentage_speed_difference(30.0)
    traffic_manager.set_random_device_seed(0)
    traffic_manager.set_hybrid_physics_mode(True)

    # Vehicle Setup
    blueprint_library = world.get_blueprint_library()
    vehicle_bp = random.choice(blueprint_library.filter('vehicle.*'))
    map = world.get_map()
    valid_spawn_points = [sp for sp in map.get_spawn_points() 
                        if map.get_waypoint(sp.location).is_junction == False]
    spawn_point = random.choice(valid_spawn_points)
    vehicle = world.spawn_actor(vehicle_bp, spawn_point)

    # Configure TM Route
    destination = random.choice(valid_spawn_points).location
    traffic_manager.set_path(vehicle, [destination])
    traffic_manager.auto_lane_change(vehicle, False)
    traffic_manager.distance_to_leading_vehicle(vehicle, 5)
    traffic_manager.ignore_lights_percentage(vehicle, 0)
    traffic_manager.ignore_signs_percentage(vehicle, 0)

    manual_control = ManualControl(vehicle, traffic_manager)
    display_manager = DisplayManager(grid_size=(2, 1))

    # Create Sensors
    collision_sensor = CollisionSensor(vehicle)
    lane_invasion_sensor = LaneInvasionSensor(vehicle)
    
    cameras = []
    for side, transform in [
        ('front', carla.Transform(carla.Location(x=1.5, z=2.4))),
        ('rear', carla.Transform(carla.Location(x=-1.5, z=2.4), carla.Rotation(yaw=180))),
        ('left', carla.Transform(carla.Location(y=-1.0, z=2.4), carla.Rotation(yaw=-90))),
        ('right', carla.Transform(carla.Location(y=1.0, z=2.4), carla.Rotation(yaw=90)))
    ]:
        for cam_type in ['rgb', 'depth', 'semantic']:
            try:
                display_pos = (0, 0) if side == 'front' and cam_type == 'rgb' else None
                cam = SensorManager(world, display_manager, cam_type, transform, vehicle, side, display_pos=display_pos)
                cameras.append(cam)
            except Exception as e:
                print(f"Sensor error: {e}")

    lidar_sensor = SensorManager(world, display_manager, 'lidar',
                               carla.Transform(carla.Location(z=2.5)),
                               vehicle, display_pos=(1, 0))

    imu_sensor = SensorManager(world, display_manager, 'imu',
                             carla.Transform(carla.Location()),
                             vehicle, display_pos=None)

    gnss_sensor = SensorManager(world, display_manager, 'gnss',
                              carla.Transform(carla.Location()),
                              vehicle, display_pos=None)

    # Data Logging Setup
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
        if not system_health_check(vehicle):
            ros_publisher.publish_status("System health check failed!")
            break

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

        # Data Collection
        velocity = vehicle.get_velocity()
        speed = 3.6 * np.linalg.norm([velocity.x, velocity.y, velocity.z])
        transform = vehicle.get_transform()
        control = vehicle.get_control()
        frame = world.get_snapshot().frame

        # Prepare Data
        current_time = time.time()
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

        # Log Data
        csv_writer.writerow(vehicle_data + sensor_data)
        ros_publisher.publish_vehicle_data(vehicle_data)
        ros_publisher.publish_sensor_data(sensor_data)
        
        clock.tick(30)

except Exception as e:
    ros_publisher.publish_status(f"Critical error: {str(e)}")
    traceback.print_exc()

finally:
    print("Cleaning up...")
    try:
        time.sleep(1)
        csv_file.close()
    except: pass
    
    sensor_destruction_order = [
        'collision_sensor', 'lane_invasion_sensor',
        'lidar_sensor', 'imu_sensor', 'gnss_sensor'
    ]
    
    for sensor in sensor_destruction_order:
        if sensor in locals():
            try: locals()[sensor].destroy()
            except: pass
    
    try: [cam.destroy() for cam in cameras]
    except: pass
    
    try: vehicle.destroy()
    except: pass
    
    pygame.quit()
    print("Simulation closed")