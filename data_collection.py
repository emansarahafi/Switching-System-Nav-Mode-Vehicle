import os
import random
import numpy as np
import pygame
import carla
import csv
import time
from collections import deque
from datetime import datetime

class ManualControl:
    def __init__(self, vehicle, traffic_manager):
        self.vehicle = vehicle
        self.autopilot = False
        self.traffic_manager = traffic_manager
        self.control_history = deque(maxlen=30)
        self.save_images = False
        self.session_id = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.create_data_directories()
        
    def create_data_directories(self):
        base_dir = f"data/{self.session_id}"
        os.makedirs(f"{base_dir}/images/rgb/front", exist_ok=True)
        os.makedirs(f"{base_dir}/logs", exist_ok=True)

    def handle_key_press(self, key):
        control = self.vehicle.get_control()
        
        if key == pygame.K_w:
            control.throttle = min(control.throttle + 0.1, 1.0)
        elif key == pygame.K_s:
            control.brake = min(control.brake + 0.1, 1.0)
        elif key == pygame.K_a:
            control.steer = max(control.steer - 0.1, -1.0)
        elif key == pygame.K_d:
            control.steer = min(control.steer + 0.1, 1.0)
        elif key == pygame.K_p:
            self.toggle_autopilot()
        elif key == pygame.K_r:
            self.save_images = not self.save_images
            print(f"Image saving {'ENABLED' if self.save_images else 'DISABLED'}")
        
        self.vehicle.apply_control(control)

    def toggle_autopilot(self):
        self.autopilot = not self.autopilot
        self.vehicle.set_autopilot(self.autopilot, self.traffic_manager.get_port())
        print(f"Autopilot {'ENABLED' if self.autopilot else 'DISABLED'}")

class CollisionSensor:
    def __init__(self, vehicle):
        self.vehicle = vehicle
        world = vehicle.get_world()
        bp = world.get_blueprint_library().find('sensor.other.collision')
        self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=vehicle)
        self.sensor.listen(self.on_collision)
        
    def on_collision(self, event):
        impulse = event.normal_impulse
        intensity = np.sqrt(impulse.x**2 + impulse.y**2 + impulse.z**2)
        if intensity > 500:
            print(f"Collision detected! Intensity: {intensity:.2f}")
            control = self.vehicle.get_control()
            control.throttle = 0.0
            control.brake = 1.0
            self.vehicle.apply_control(control)

class CameraSensor:
    def __init__(self, vehicle, display_pos=(0,0)):
        self.display_pos = display_pos
        blueprint = vehicle.get_world().get_blueprint_library().find('sensor.camera.rgb')
        blueprint.set_attribute('image_size_x', '320')
        blueprint.set_attribute('image_size_y', '240')
        blueprint.set_attribute('fov', '90')
        
        self.sensor = vehicle.get_world().spawn_actor(
            blueprint,
            carla.Transform(carla.Location(x=1.5, z=2.4)),
            attach_to=vehicle
        )
        self.sensor.listen(self.process_image)
        self.surface = None
        
    def process_image(self, image):
        array = np.frombuffer(image.raw_data, dtype=np.uint8)
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        self.surface = pygame.surfarray.make_surface(array.swapaxes(0,1))

class LidarSensor:
    def __init__(self, vehicle, display_pos=(1,0)):
        self.display_pos = display_pos
        blueprint = vehicle.get_world().get_blueprint_library().find('sensor.lidar.ray_cast')
        blueprint.set_attribute('range', '50')
        blueprint.set_attribute('rotation_frequency', '10')
        blueprint.set_attribute('channels', '64')
        blueprint.set_attribute('points_per_second', '56000')
        
        self.sensor = vehicle.get_world().spawn_actor(
            blueprint,
            carla.Transform(carla.Location(z=2.5)),
            attach_to=vehicle
        )
        self.sensor.listen(self.process_lidar)
        self.surface = None
        
    def process_lidar(self, data):
        points = np.frombuffer(data.raw_data, dtype=np.dtype('f4'))
        points = np.reshape(points, (int(points.shape[0]/4), 4))
        lidar_image = np.zeros((240, 320, 3), dtype=np.uint8)
        
        for point in points[::10]:
            x = int((point[0]/50) * 320)
            y = int((point[1]/20) * 120 + 120)
            if 0 <= x < 320 and 0 <= y < 240:
                lidar_image[y, x] = (255, 255, 255)
                
        self.surface = pygame.surfarray.make_surface(lidar_image)

def main():
    try:
        # Initialize Pygame
        pygame.init()
        display = pygame.display.set_mode((640, 240))
        pygame.display.set_caption("CARLA Autopilot")
        
        # Connect to CARLA
        client = carla.Client('192.168.1.19', 2000)
        client.set_timeout(10.0)
        world = client.get_world()
        
        # Setup Traffic Manager
        traffic_manager = client.get_trafficmanager(8000)
        traffic_manager.set_global_distance_to_leading_vehicle(3.0)
        traffic_manager.global_percentage_speed_difference(30.0)
        traffic_manager.set_hybrid_physics_mode(True)
        
        # Spawn vehicle
        blueprint = world.get_blueprint_library().filter('vehicle.tesla.model3')[0]
        spawn_point = random.choice(world.get_map().get_spawn_points())
        vehicle = world.spawn_actor(blueprint, spawn_point)
        
        # Initialize systems
        control = ManualControl(vehicle, traffic_manager)
        collision_sensor = CollisionSensor(vehicle)
        camera = CameraSensor(vehicle)
        lidar = LidarSensor(vehicle)
        
        # Data logging
        csv_file = open(f"data/{control.session_id}/logs/driving.csv", 'w')
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow(['timestamp', 'speed', 'throttle', 'brake', 'steer', 'autopilot'])
        
        clock = pygame.time.Clock()
        running = True
        
        while running:
            clock.tick(30)
            
            # Handle input
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    control.handle_key_press(event.key)
            
            # Update display
            display.fill((0,0,0))
            if camera.surface:
                display.blit(camera.surface, (0,0))
            if lidar.surface:
                display.blit(lidar.surface, (320,0))
            pygame.display.flip()
            
            # Log data
            velocity = vehicle.get_velocity()
            speed = 3.6 * (velocity.x**2 + velocity.y**2 + velocity.z**2)**0.5
            control_state = vehicle.get_control()
            csv_writer.writerow([
                time.time(),
                speed,
                control_state.throttle,
                control_state.brake,
                control_state.steer,
                control.autopilot
            ])
            
    finally:
        print("Cleaning up...")
        pygame.quit()
        if 'vehicle' in locals():
            vehicle.destroy()
        if 'csv_file' in locals():
            csv_file.close()

if __name__ == '__main__':
    main()