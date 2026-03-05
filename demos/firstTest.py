"""
Simplest CARLA Planner
- Spawn a vehicle
- Generate a straight-line trajectory
- Follow it with simple steering control
"""

import carla
import numpy as np
import pygame
import queue
import sys

# Constants
SCREEN_WIDTH = 1280
SCREEN_HEIGHT = 720
FPS = 20

class CameraManager:
    """Manages CARLA camera and image processing"""
    def __init__(self, world, vehicle, width=SCREEN_WIDTH, height=SCREEN_HEIGHT):
        self.world = world
        self.vehicle = vehicle
        self.width = width
        self.height = height
        self.surface = None
        self.image_queue = queue.Queue()
        
        # Spawn camera
        blueprint_library = world.get_blueprint_library()
        camera_bp = blueprint_library.find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', str(width))
        camera_bp.set_attribute('image_size_y', str(height))
        camera_bp.set_attribute('fov', '110')
        
        # Attach camera to vehicle
        spawn_point = carla.Transform(
            carla.Location(x=-5.5, z=2.8),  # Behind and above vehicle
            carla.Rotation(pitch=-15)
        )
        self.camera = world.spawn_actor(
            camera_bp, 
            spawn_point, 
            attach_to=vehicle
        )
        
        # Start listening
        self.camera.listen(self.image_queue.put)
    
    def get_image(self):
        """Retrieve and process the latest image"""
        if not self.image_queue.empty():
            image = self.image_queue.get()
            
            # Convert CARLA image to pygame surface
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (image.height, image.width, 4))
            array = array[:, :, :3]  # Remove alpha channel
            array = array[:, :, ::-1]  # BGR to RGB
            
            self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        
        return self.surface
    
    def destroy(self):
        if self.camera is not None:
            self.camera.stop()
            self.camera.destroy()


class HUD:
    """Heads-up display for showing vehicle information"""
    def __init__(self, width, height):
        self.width = width
        self.height = height
        pygame.font.init()
        self.font = pygame.font.Font(pygame.font.get_default_font(), 20)
        self.font_small = pygame.font.Font(pygame.font.get_default_font(), 14)
    
    def render(self, display, vehicle, target_idx, total_waypoints, steering):
        """Render HUD elements"""
        # Get vehicle info
        velocity = vehicle.get_velocity()
        speed = 3.6 * np.sqrt(velocity.x**2 + velocity.y**2)  # m/s to km/h
        transform = vehicle.get_transform()
        
        # Create semi-transparent background
        info_surface = pygame.Surface((300, 200))
        info_surface.set_alpha(200)
        info_surface.fill((0, 0, 0))
        display.blit(info_surface, (10, 10))
        
        # Display information
        texts = [
            f"Speed: {speed:.1f} km/h",
            f"Location: ({transform.location.x:.1f}, {transform.location.y:.1f})",
            f"Heading: {transform.rotation.yaw:.1f}°",
            f"Steering: {steering:.2f}",
            f"Waypoint: {target_idx}/{total_waypoints}",
            f"Progress: {100*target_idx/total_waypoints:.1f}%"
        ]
        
        y_offset = 20
        for text in texts:
            text_surface = self.font.render(text, True, (255, 255, 255))
            display.blit(text_surface, (20, y_offset))
            y_offset += 30
        
        # Instructions
        instructions = [
            "ESC: Quit",
            "R: Reset",
            "Space: Pause"
        ]
        
        y_offset = self.height - 100
        for instruction in instructions:
            text_surface = self.font_small.render(instruction, True, (200, 200, 200))
            display.blit(text_surface, (20, y_offset))
            y_offset += 20


class SimplePlanner:
    def __init__(self, waypoints, target_speed=10.0):
        self.waypoints = waypoints
        self.target_speed = target_speed
        
    def get_target_point(self, vehicle_location):
        min_dist = float('inf')
        closest_idx = 0
        
        for i, wp in enumerate(self.waypoints):
            dist = vehicle_location.distance(wp)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        
        lookahead = 5
        target_idx = min(closest_idx + lookahead, len(self.waypoints) - 1)
        
        return self.waypoints[target_idx], target_idx


class PurePursuitController:
    def __init__(self, lookahead_dist=5.0):
        self.lookahead_dist = lookahead_dist
        
    def compute_control(self, vehicle_transform, target_location, target_speed, current_velocity):
        vehicle_loc = vehicle_transform.location
        vehicle_yaw = np.radians(vehicle_transform.rotation.yaw)
        
        dx = target_location.x - vehicle_loc.x
        dy = target_location.y - vehicle_loc.y
        
        dx_veh = dx * np.cos(vehicle_yaw) + dy * np.sin(vehicle_yaw)
        dy_veh = -dx * np.sin(vehicle_yaw) + dy * np.cos(vehicle_yaw)
        
        ld = np.sqrt(dx_veh**2 + dy_veh**2)
        steering = np.arctan2(2.0 * dy_veh, ld) / np.pi
        steering = np.clip(steering, -1.0, 1.0)
        
        current_speed = np.sqrt(current_velocity.x**2 + current_velocity.y**2)
        speed_error = target_speed - current_speed
        throttle = np.clip(0.5 * speed_error, 0.0, 1.0)
        
        brake = 0.0
        if speed_error < -2.0:
            brake = 0.5
            throttle = 0.0
        
        return steering, throttle, brake


def generate_straight_trajectory(start_location, length=100, num_points=100):
    waypoints = []
    for i in range(num_points):
        x = start_location.x + (length / num_points) * i
        y = start_location.y
        z = start_location.z
        waypoints.append(carla.Location(x=x, y=y, z=z))
    return waypoints


def main():
    # Initialize pygame
    pygame.init()
    display = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    pygame.display.set_caption("CARLA Autonomous Driving")
    clock = pygame.time.Clock()
    
    # Connect to CARLA
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    
    print("Connected to CARLA version:", client.get_client_version())
    
    world = client.get_world()
    
    # Set synchronous mode
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.05
    world.apply_settings(settings)
    
    vehicle = None
    camera_manager = None
    
    try:
        # Spawn vehicle
        blueprint_library = world.get_blueprint_library()
        vehicle_bp = blueprint_library.filter('vehicle.tesla.model3')[0]
        
        spawn_points = world.get_map().get_spawn_points()
        spawn_point = spawn_points[0]
        
        print(f"Spawning vehicle at {spawn_point.location}")
        vehicle = world.spawn_actor(vehicle_bp, spawn_point)
        
        # Create camera
        camera_manager = CameraManager(world, vehicle)
        
        # Initialize HUD
        hud = HUD(SCREEN_WIDTH, SCREEN_HEIGHT)
        
        # Wait for vehicle to settle
        for _ in range(10):
            world.tick()
        
        # Generate trajectory
        start_loc = spawn_point.location
        waypoints = generate_straight_trajectory(start_loc, length=100, num_points=100)
        
        print(f"Generated {len(waypoints)} waypoints")
        
        # Visualize waypoints in CARLA
        for wp in waypoints[::10]:
            world.debug.draw_point(
                wp, size=0.1,
                color=carla.Color(0, 255, 0),
                life_time=100.0
            )
        
        # Initialize planner and controller
        planner = SimplePlanner(waypoints, target_speed=10.0)
        controller = PurePursuitController(lookahead_dist=5.0)
        
        print("Starting control loop...")
        
        running = True
        paused = False
        steering = 0.0
        target_idx = 0
        
        while running:
            # Handle pygame events
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False
                    elif event.key == pygame.K_SPACE:
                        paused = not paused
                        print("Paused" if paused else "Resumed")
                    elif event.key == pygame.K_r:
                        print("Reset vehicle")
                        vehicle.set_transform(spawn_point)
            
            if not paused:
                # Get vehicle state
                vehicle_transform = vehicle.get_transform()
                vehicle_velocity = vehicle.get_velocity()
                
                # Get target from planner
                target_location, target_idx = planner.get_target_point(
                    vehicle_transform.location
                )
                
                # Compute control
                steering, throttle, brake = controller.compute_control(
                    vehicle_transform,
                    target_location,
                    planner.target_speed,
                    vehicle_velocity
                )
                
                # Apply control
                control = carla.VehicleControl()
                control.steer = steering
                control.throttle = throttle
                control.brake = brake
                vehicle.apply_control(control)
                
                # Tick simulation
                world.tick()
                
                # Check if reached end
                if target_idx >= len(waypoints) - 1:
                    print("Reached end of trajectory!")
                    paused = True
            
            # Render
            image = camera_manager.get_image()
            if image is not None:
                display.blit(image, (0, 0))
            
            # Render HUD
            hud.render(display, vehicle, target_idx, len(waypoints), steering)
            
            pygame.display.flip()
            clock.tick(FPS)
        
        print("Done!")
        
    finally:
        print("Cleaning up...")
        
        if camera_manager is not None:
            camera_manager.destroy()
        
        if vehicle is not None:
            vehicle.destroy()
        
        settings.synchronous_mode = False
        world.apply_settings(settings)
        
        pygame.quit()


if __name__ == '__main__':
    main()
