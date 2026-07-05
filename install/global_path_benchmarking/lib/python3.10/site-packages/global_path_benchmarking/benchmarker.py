#!/usr/bin/env python3

import os
import sys
import time
import yaml
import json
import argparse
import subprocess
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image, ImageDraw

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs.msg import OccupancyGrid, Path
from nav2_msgs.action import ComputePathToPose
from geometry_msgs.msg import PoseStamped, Quaternion
from std_msgs.msg import Header

# Helper function to kill old ROS/Nav2/Gazebo processes
def clean_old_processes():
    print("[INFO] Cleaning up stale ROS 2, Nav2, and Gazebo processes...")
    # List of process patterns to kill
    targets = [
        "planner_server",
        "controller_server",
        "bt_navigator",
        "waypoint_follower",
        "nav2_container",
        "gzserver",
        "gzclient",
        "gazebo",
        "mock_planner"
    ]
    for target in targets:
        try:
            # Run pkill command
            subprocess.run(["pkill", "-f", "-9", target], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception as e:
            pass
    time.sleep(1.0)
    print("[INFO] Stale processes killed successfully.")

# Helper to generate mock/synthetic maps if they don't exist
def generate_synthetic_maps():
    os.makedirs("maps", exist_ok=True)
    os.makedirs("reports", exist_ok=True)
    
    # Pre-clean any existing synthetic map files to prevent stale configurations
    synthetic_files = [
        "maps/empty_world.png",
        "maps/scattered_rocks.png",
        "maps/canyon_gate.png",
        "maps/marsyard_slopes.png",
        "maps/marsyard_labyrinth.png",
        "maps/crater_yard.npy",
        "maps/snake_passage.png",
        "maps/dead_end_trap.png",
        "maps/crater_field.png"
    ]
    for f in synthetic_files:
        if os.path.exists(f):
            try:
                os.remove(f)
            except Exception:
                pass
                
    # 1. Empty World
    empty_path = "maps/empty_world.png"
    img = Image.new("L", (200, 200), 255)
    draw = ImageDraw.Draw(img)
    # Draw a black border (obstacles around boundaries)
    draw.rectangle([0, 0, 199, 199], outline=0, width=5)
    img.save(empty_path)
    print(f"[INFO] Generated synthetic map: {empty_path}")

    # 2. Scattered Rocks
    rocks_path = "maps/scattered_rocks.png"
    img = Image.new("L", (200, 200), 255)
    draw = ImageDraw.Draw(img)
    # Border
    draw.rectangle([0, 0, 199, 199], outline=0, width=5)
    # Draw some "rocks" (black circles)
    draw.ellipse([40, 40, 60, 60], fill=0)
    draw.ellipse([110, 70, 140, 100], fill=0)
    draw.ellipse([70, 130, 95, 155], fill=0)
    draw.ellipse([150, 140, 170, 160], fill=0)
    img.save(rocks_path)
    print(f"[INFO] Generated synthetic map: {rocks_path}")

    # 3. Canyon Gate (Narrow Passage)
    canyon_path = "maps/canyon_gate.png"
    img = Image.new("L", (200, 200), 255)
    draw = ImageDraw.Draw(img)
    # Border
    draw.rectangle([0, 0, 199, 199], outline=0, width=5)
    # Draw a horizontal wall with a gap in the center
    draw.rectangle([0, 95, 80, 105], fill=0)     # Left wall
    draw.rectangle([120, 95, 199, 105], fill=0)   # Right wall (gap is 80 to 120)
    img.save(canyon_path)
    print(f"[INFO] Generated synthetic map: {canyon_path}")

    # 4. Marsyard Rough Slopes (Hard Terrain)
    slopes_path = "maps/marsyard_slopes.png"
    img = Image.new("L", (200, 200), 255)
    draw = ImageDraw.Draw(img)
    # Border
    draw.rectangle([0, 0, 199, 199], outline=0, width=5)
    # Slopes (High cost gray area)
    draw.rectangle([60, 60, 140, 140], fill=120)
    # Rock Obstacles
    draw.ellipse([90, 90, 110, 110], fill=0)
    draw.rectangle([20, 80, 40, 120], fill=0)
    draw.rectangle([160, 80, 180, 120], fill=0)
    img.save(slopes_path)
    print(f"[INFO] Generated synthetic map: {slopes_path}")

    # 5. Marsyard Labyrinth (Hard Obstacles)
    lab_path = "maps/marsyard_labyrinth.png"
    img = Image.new("L", (200, 200), 255)
    draw = ImageDraw.Draw(img)
    # Border
    draw.rectangle([0, 0, 199, 199], outline=0, width=5)
    # Maze walls
    draw.rectangle([0, 50, 140, 60], fill=0)
    draw.rectangle([60, 100, 199, 110], fill=0)
    draw.rectangle([0, 150, 140, 160], fill=0)
    img.save(lab_path)
    print(f"[INFO] Generated synthetic map: {lab_path}")

    # 6. Elevation Height Map (.npy)
    heightmap_path = "maps/crater_yard.npy"
    grid = np.zeros((200, 200), dtype=np.float32)
    # Add a Gaussian hill in the center (x=100, y=100)
    y, x = np.ogrid[0:200, 0:200]
    hill = 1.5 * np.exp(-((x - 100)**2 + (y - 100)**2) / (2 * 25**2))
    # Add a crater at (x=50, y=130)
    crater = -1.0 * np.exp(-((x - 50)**2 + (y - 130)**2) / (2 * 15**2))
    grid = grid + hill + crater
    np.save(heightmap_path, grid)
    print(f"[INFO] Generated synthetic heightmap: {heightmap_path}")

    # 7. Marsyard Snake Passage (S-Curve winding corridor)
    snake_path = "maps/snake_passage.png"
    img = Image.new("L", (200, 200), 255)
    draw = ImageDraw.Draw(img)
    # Border
    draw.rectangle([0, 0, 199, 199], outline=0, width=5)
    # Two offset horizontal walls forming an S-curve corridor
    draw.rectangle([0, 60, 140, 70], fill=0)
    draw.rectangle([60, 130, 199, 140], fill=0)
    img.save(snake_path)
    print(f"[INFO] Generated synthetic map: {snake_path}")

    # 8. Dead End Trap (U-shaped obstacle facing start)
    trap_path = "maps/dead_end_trap.png"
    img = Image.new("L", (200, 200), 255)
    draw = ImageDraw.Draw(img)
    # Border
    draw.rectangle([0, 0, 199, 199], outline=0, width=5)
    # U-shaped pocket blocking direct path to goal
    draw.rectangle([60, 115, 140, 125], fill=0)   # Cap
    draw.rectangle([60, 60, 70, 115], fill=0)     # Left side
    draw.rectangle([130, 60, 140, 115], fill=0)    # Right side
    img.save(trap_path)
    print(f"[INFO] Generated synthetic map: {trap_path}")

    # 9. Crater Field (Scattered variable-slope cost zones)
    field_path = "maps/crater_field.png"
    img = Image.new("L", (200, 200), 255)
    draw = ImageDraw.Draw(img)
    # Border
    draw.rectangle([0, 0, 199, 199], outline=0, width=5)
    # Multiple slope craters of intermediate cost gray levels
    draw.ellipse([80, 80, 120, 120], fill=180)   # Center high cost
    draw.ellipse([36, 116, 84, 148], fill=140)   # Left medium cost
    draw.ellipse([116, 36, 148, 84], fill=150)   # Right medium cost
    img.save(field_path)
    print(f"[INFO] Generated synthetic map: {field_path}")

# YAML Scenarios Generator if scenarios.yaml is missing
def generate_default_scenarios_yaml():
    os.makedirs("config", exist_ok=True)
    config_path = "config/scenarios.yaml"
    if not os.path.exists(config_path):
        scenarios = {
            "scenarios": [
                {
                    "id": "empty_straight",
                    "map_image": "maps/empty_world.png",
                    "resolution": 0.05,
                    "origin": [-5.0, -5.0],
                    "robot_radius": 0.3,
                    "start": [-3.0, -3.0],
                    "goal": [3.0, 3.0],
                    "reference_path": [
                        [-3.0, -3.0],
                        [-1.5, -1.5],
                        [0.0, 0.0],
                        [1.5, 1.5],
                        [3.0, 3.0]
                    ]
                },
                {
                    "id": "scattered_rocks_detour",
                    "map_image": "maps/scattered_rocks.png",
                    "resolution": 0.05,
                    "origin": [-5.0, -5.0],
                    "robot_radius": 0.3,
                    "start": [-4.0, -4.0],
                    "goal": [4.0, 4.0],
                    "reference_path": [
                        [-4.0, -4.0],
                        [-2.5, -1.5],
                        [-1.0, 1.0],
                        [1.0, 2.5],
                        [4.0, 4.0]
                    ],
                    "dynamic_obstacles": [
                        {
                            "trigger_time": 0.5,
                            "x": 1.0,
                            "y": 2.5,
                            "radius": 0.4
                        }
                    ]
                },
                {
                    "id": "canyon_gate_passage",
                    "map_image": "maps/canyon_gate.png",
                    "resolution": 0.05,
                    "origin": [-5.0, -5.0],
                    "robot_radius": 0.3,
                    "start": [0.0, -4.0],
                    "goal": [0.0, 4.0],
                    "reference_path": [
                        [0.0, -4.0],
                        [0.0, -1.0],
                        [0.0, 0.0],
                        [0.0, 2.0],
                        [0.0, 4.0]
                    ]
                },
                {
                    "id": "marsyard_rough_slopes",
                    "map_image": "maps/marsyard_slopes.png",
                    "resolution": 0.05,
                    "origin": [-5.0, -5.0],
                    "robot_radius": 0.3,
                    "start": [-4.0, -4.0],
                    "goal": [4.0, 4.0],
                    "reference_path": [
                        [-4.0, -4.0],
                        [-3.5, 0.0],
                        [-3.0, 3.0],
                        [0.0, 4.0],
                        [4.0, 4.0]
                    ]
                },
                {
                    "id": "marsyard_labyrinth",
                    "map_image": "maps/marsyard_labyrinth.png",
                    "resolution": 0.05,
                    "origin": [-5.0, -5.0],
                    "robot_radius": 0.3,
                    "start": [-4.0, -4.0],
                    "goal": [4.0, 4.0],
                    "reference_path": [
                        [-4.0, -4.0],
                        [3.5, -4.0],
                        [3.5, -1.0],
                        [-3.5, -1.0],
                        [-3.5, 1.5],
                        [3.5, 1.5],
                        [3.5, 4.0],
                        [4.0, 4.0]
                    ]
                },
                {
                    "id": "canyon_gate_blocked",
                    "map_image": "maps/canyon_gate.png",
                    "resolution": 0.05,
                    "origin": [-5.0, -5.0],
                    "robot_radius": 0.3,
                    "start": [0.0, -4.0],
                    "goal": [0.0, 4.0],
                    "reference_path": [
                        [0.0, -4.0],
                        [0.0, 4.0]
                    ],
                    "dynamic_obstacles": [
                        {
                            "trigger_time": 0.5,
                            "x": 0.0,
                            "y": 0.0,
                            "radius": 1.5
                        }
                    ]
                },
                {
                    "id": "marsyard_snake_passage",
                    "map_image": "maps/snake_passage.png",
                    "resolution": 0.05,
                    "origin": [-5.0, -5.0],
                    "robot_radius": 0.3,
                    "start": [-4.0, -4.0],
                    "goal": [4.0, 4.0],
                    "reference_path": [
                        [-4.0, -4.0],
                        [3.5, -4.0],
                        [3.5, 0.0],
                        [-3.5, 0.0],
                        [-3.5, 4.0],
                        [4.0, 4.0]
                    ]
                },
                {
                    "id": "dead_end_trap",
                    "map_image": "maps/dead_end_trap.png",
                    "resolution": 0.05,
                    "origin": [-5.0, -5.0],
                    "robot_radius": 0.3,
                    "start": [0.0, -1.0],
                    "goal": [0.0, 4.0],
                    "reference_path": [
                        [0.0, -1.0],
                        [0.0, -3.5],
                        [3.5, -3.5],
                        [3.5, 2.5],
                        [0.0, 4.0]
                    ]
                },
                {
                    "id": "crater_field",
                    "map_image": "maps/crater_field.png",
                    "resolution": 0.05,
                    "origin": [-5.0, -5.0],
                    "robot_radius": 0.3,
                    "start": [-4.0, -4.0],
                    "goal": [4.0, 4.0],
                    "reference_path": [
                        [-4.0, -4.0],
                        [-2.5, -1.5],
                        [0.0, -2.5],
                        [2.5, 0.0],
                        [1.5, 2.5],
                        [4.0, 4.0]
                    ]
                }
            ]
        }
        with open(config_path, "w") as f:
            yaml.dump(scenarios, f, default_flow_style=False)
        print(f"[INFO] Generated default scenarios: {config_path}")


class PathBenchmarker(Node):
    def __init__(self, config_path):
        super().__init__("global_path_benchmarker")
        self.config_path = config_path
        
        # Load scenarios
        with open(config_path, "r") as f:
            self.scenarios = yaml.safe_load(f)["scenarios"]
        
        # ROS 2 Publishers
        self.costmap_pub = self.create_publisher(OccupancyGrid, "/global_costmap/costmap", 10)
        self.trav_pub = self.create_publisher(OccupancyGrid, "/traversability_map", 10)
        
        # ROS 2 Action Client
        self.nav_client = ActionClient(self, ComputePathToPose, "compute_path_to_pose")
        
        self.get_logger().info("PathBenchmarker Node Initialized.")

    def cleanup_previous_reports(self, prefix="result_scenario_"):
        import glob
        if prefix == "result_scenario_":
            for f in ["reports/results.json", "reports/numerical_report.md", "reports/numerical_report.pdf"]:
                if os.path.exists(f):
                    try:
                        os.remove(f)
                    except Exception:
                        pass
        files = glob.glob(f"reports/{prefix}*.png")
        for f in files:
            try:
                os.remove(f)
            except Exception:
                pass

    def publish_map(self, scenario):
        resolution = scenario["resolution"]
        origin_coords = scenario["origin"]
        robot_radius = scenario.get("robot_radius", 0.3)
        
        # Load the map image
        map_img_path = scenario["map_image"]
        img = Image.open(map_img_path).convert("L")
        width, height = img.size
        
        # Invert grayscale (255=free -> 0, 0=blocked -> 100)
        img_arr = np.array(img)
        cost_arr = np.zeros_like(img_arr)
        # Map values
        cost_arr[img_arr == 0] = 100 # lethal
        cost_arr[img_arr == 255] = 0 # free
        # For gray scales: linear scaling to 0-99
        gray_mask = (img_arr > 0) & (img_arr < 255)
        cost_arr[gray_mask] = (100 - (img_arr[gray_mask] / 2.55)).astype(np.int8)

        # Flips to match ROS coordinates (usually origin is bottom-left)
        cost_arr = np.flipud(cost_arr)
        cost_data = cost_arr.flatten().tolist()
        
        # Create OccupancyGrid
        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = "map"
        grid.info.resolution = resolution
        grid.info.width = width
        grid.info.height = height
        grid.info.origin.position.x = float(origin_coords[0])
        grid.info.origin.position.y = float(origin_coords[1])
        grid.info.origin.position.z = 0.0
        # Orientation is identity quaternion
        grid.info.origin.orientation.x = 0.0
        grid.info.origin.orientation.y = 0.0
        grid.info.origin.orientation.z = 0.0
        grid.info.origin.orientation.w = 1.0
        grid.data = cost_data
        
        self.costmap_pub.publish(grid)
        self.trav_pub.publish(grid) # Publish traversability map as grid too
        
        self.get_logger().info(f"Published costmap for {scenario['id']}")
        return grid, cost_arr, width, height

    def update_map_with_obstacle(self, base_grid, cost_arr, scenario, x, y, radius):
        # Convert physical obstacle x,y to grid coordinates
        resolution = scenario["resolution"]
        origin_x = scenario["origin"][0]
        origin_y = scenario["origin"][1]
        width = base_grid.info.width
        height = base_grid.info.height
        
        grid_x = int((x - origin_x) / resolution)
        grid_y = int((y - origin_y) / resolution)
        grid_radius = int(radius / resolution)
        
        # Update cost_arr locally
        new_cost_arr = np.copy(cost_arr)
        for r in range(max(0, grid_y - grid_radius), min(height, grid_y + grid_radius + 1)):
            for c in range(max(0, grid_x - grid_radius), min(width, grid_x + grid_radius + 1)):
                if (r - grid_y)**2 + (c - grid_x)**2 <= grid_radius**2:
                    new_cost_arr[r, c] = 100
        
        updated_grid = OccupancyGrid()
        updated_grid.header = base_grid.header
        updated_grid.header.stamp = self.get_clock().now().to_msg()
        updated_grid.info = base_grid.info
        updated_grid.data = new_cost_arr.flatten().tolist()
        
        self.costmap_pub.publish(updated_grid)
        self.get_logger().info(f"Published dynamic costmap update with obstacle at ({x}, {y})")
        return updated_grid, new_cost_arr

    def wait_for_planner(self, timeout=5.0):
        self.get_logger().info("Waiting for compute_path_to_pose action server...")
        return self.nav_client.wait_for_server(timeout_sec=timeout)

    def request_path(self, start, goal):
        goal_msg = ComputePathToPose.Goal()
        goal_msg.use_start = True
        
        # Set start
        start_pose = PoseStamped()
        start_pose.header.frame_id = "map"
        start_pose.header.stamp = self.get_clock().now().to_msg()
        start_pose.pose.position.x = start[0]
        start_pose.pose.position.y = start[1]
        start_pose.pose.position.z = 0.0
        start_pose.pose.orientation.w = 1.0
        goal_msg.start = start_pose
        
        # Set goal
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = goal[0]
        goal_pose.pose.position.y = goal[1]
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.w = 1.0
        goal_msg.goal = goal_pose
        
        # Send action goal
        start_time = self.get_clock().now()
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Planning request rejected by planner server.")
            return None, 0.0
        
        # Get result
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        end_time = self.get_clock().now()
        
        duration = (end_time - start_time).nanoseconds / 1e9
        result = get_result_future.result()
        
        if result.status == 4: # STATUS_SUCCEEDED
            return result.result.path, duration
        else:
            self.get_logger().error(f"Planning failed with action status: {result.status}")
            return None, duration

    def evaluate_path(self, path, reference_path, cost_arr, scenario):
        if not path or len(path.poses) == 0:
            return {
                "success": False,
                "planning_time": 0.0,
                "length": 0.0,
                "length_ratio": 999.0,
                "blocked_cells": 100,
                "avg_cost": 100.0,
                "safety_margin": 0.0,
                "score": 0.0
            }
            
        resolution = scenario["resolution"]
        origin_x = scenario["origin"][0]
        origin_y = scenario["origin"][1]
        robot_radius = scenario.get("robot_radius", 0.3)
        
        # Convert path points to float tuples
        coords = []
        for pose_stamped in path.poses:
            coords.append((pose_stamped.pose.position.x, pose_stamped.pose.position.y))
            
        # Calculate Path Length
        length = 0.0
        for p1, p2 in zip(coords[:-1], coords[1:]):
            length += np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
            
        # Calculate Reference Path Length
        ref_coords = reference_path
        ref_length = 0.0
        for r1, r2 in zip(ref_coords[:-1], ref_coords[1:]):
            ref_length += np.sqrt((r1[0] - r2[0])**2 + (r1[1] - r2[1])**2)
            
        length_ratio = length / ref_length if ref_length > 0 else 1.0
        
        # Check Collision & Obstacle Costs
        blocked_cells = 0
        total_cost = 0.0
        safety_margin = float('inf')
        
        width = cost_arr.shape[1]
        height = cost_arr.shape[0]
        
        # Find all obstacle coordinates for safety margin computation
        # np.where returns (row_indices, col_indices)
        # Flip index order because row is Y, col is X
        obstacle_rows, obstacle_cols = np.where(cost_arr >= 100)
        obstacle_points = []
        for r, c in zip(obstacle_rows, obstacle_cols):
            x_m = origin_x + c * resolution
            y_m = origin_y + r * resolution
            obstacle_points.append((x_m, y_m))
        
        for x, y in coords:
            # Grid index
            col = int((x - origin_x) / resolution)
            row = int((y - origin_y) / resolution)
            
            # Map cost
            if 0 <= row < height and 0 <= col < width:
                cost = cost_arr[row, col]
                total_cost += cost
            else:
                cost = 100
                total_cost += 100
            
            # Calculate distance to nearest obstacle
            min_dist = float('inf')
            if len(obstacle_points) > 0:
                dists = np.sqrt((x - np.array([op[0] for op in obstacle_points]))**2 + 
                                (y - np.array([op[1] for op in obstacle_points]))**2)
                min_dist = np.min(dists)
                if min_dist < safety_margin:
                    safety_margin = min_dist
            
            # Check footprint collision
            if min_dist < robot_radius:
                blocked_cells += 1
                
        avg_cost = total_cost / len(coords) if len(coords) > 0 else 100.0
        if safety_margin == float('inf'):
            safety_margin = 10.0 # Default if no obstacles
            
        return {
            "success": True,
            "coords": coords,
            "length": length,
            "length_ratio": length_ratio,
            "blocked_cells": blocked_cells,
            "avg_cost": avg_cost,
            "safety_margin": safety_margin
        }

    # Generate pre-test verification plots
    def verify_scenarios(self):
        self.cleanup_previous_reports("verify_scenario_")
        print("\n================ VERIFYING SCENARIOS ================")
        for s in self.scenarios:
            map_image = s["map_image"]
            ref_path = s["reference_path"]
            resolution = s["resolution"]
            origin = s["origin"]
            robot_radius = s.get("robot_radius", 0.3)
            
            img = Image.open(map_image).convert("L")
            img_arr = np.array(img)
            
            plt.figure(figsize=(8, 8))
            # Display image, adjust extent using resolution/origin
            extent = [origin[0], origin[0] + img_arr.shape[1]*resolution, 
                      origin[1], origin[1] + img_arr.shape[0]*resolution]
            plt.imshow(img_arr, cmap='gray', origin='lower', extent=extent)
            
            # Plot reference path
            ref_x = [p[0] for p in ref_path]
            ref_y = [p[1] for p in ref_path]
            plt.plot(ref_x, ref_y, 'g-o', label='Reference Path (Perfect)', linewidth=2)
            
            # Draw robot radius footprint shaded boundaries around start/end/middle
            for px, py in ref_path:
                circle = plt.Circle((px, py), robot_radius, color='g', fill=True, alpha=0.1)
                plt.gca().add_patch(circle)
                
            plt.plot(s["start"][0], s["start"][1], 'bs', markersize=10, label='Start')
            plt.plot(s["goal"][0], s["goal"][1], 'r*', markersize=12, label='Goal')
            
            plt.title(f"Verify Scenario: {s['id']}")
            plt.xlabel("X (meters)")
            plt.ylabel("Y (meters)")
            plt.legend()
            plt.grid(True, alpha=0.3)
            
            out_file = f"reports/verify_scenario_{s['id']}.png"
            plt.savefig(out_file, bbox_inches='tight')
            plt.close()
            print(f"[VERIFIED] Scenario '{s['id']}' image saved to: {out_file}")
        print("=====================================================\n")

    # Run actual tests
    def run_tests(self, select_scenario_id=None):
        self.cleanup_previous_reports("result_scenario_")
        if not self.wait_for_planner(5.0):
            self.get_logger().error("Action server /compute_path_to_pose not available. Make sure your planner is running.")
            return False
            
        results = {}
        for s in self.scenarios:
            if select_scenario_id and s["id"] != select_scenario_id:
                continue
                
            self.get_logger().info(f"Running Scenario: {s['id']}")
            
            # 1. Publish Map
            grid, cost_arr, w, h = self.publish_map(s)
            time.sleep(1.0) # Let costmap sink
            
            # 2. Plan path (Static)
            path, plan_time = self.request_path(s["start"], s["goal"])
            
            # 3. Evaluate static path
            eval_res = self.evaluate_path(path, s["reference_path"], cost_arr, s)
            
            # 4. Check Replanning if dynamic obstacles exist
            replan_success_val = 0.0
            replan_time = 0.0
            replan_path_coords = []
            
            dyn_obs = s.get("dynamic_obstacles", [])
            if len(dyn_obs) > 0 and path and len(path.poses) > 0:
                self.get_logger().info("Triggering dynamic obstacle update for replanning evaluation...")
                
                # Take the first obstacle trigger
                obs = dyn_obs[0]
                # Update map with obstacle directly blocking path or at specific coord
                updated_grid, updated_cost_arr = self.update_map_with_obstacle(
                    grid, cost_arr, s, obs["x"], obs["y"], obs["radius"]
                )
                time.sleep(0.5) # Let map update propagate
                
                # Query planner again
                replan_path, replan_duration = self.request_path(s["start"], s["goal"])
                replan_time = replan_duration
                
                # Evaluate new path
                replan_eval = self.evaluate_path(replan_path, s["reference_path"], updated_cost_arr, s)
                
                # Replan is successful if planner found path without collision with updated map
                if replan_eval["success"] and replan_eval["blocked_cells"] == 0 and replan_duration <= 2.0:
                    replan_success_val = 100.0
                    replan_path_coords = replan_eval["coords"]
                    self.get_logger().info(f"Replanning Succeeded in {replan_duration:.3f} s!")
                else:
                    self.get_logger().warn(f"Replanning Failed or Timed out! Duration: {replan_duration:.3f} s")
            else:
                # No dynamic obstacles -> replanning defaults to 100 if static succeeded
                if eval_res["success"] and eval_res["blocked_cells"] == 0:
                    replan_success_val = 100.0
                else:
                    replan_success_val = 0.0
            
            # 5. Compute scores using the formulas
            # Sub-score S_success
            s_success = 100.0 if eval_res["success"] else 0.0
            
            # Sub-score S_time (target <= 2s, timeout >= 10s)
            if plan_time <= 2.0:
                s_time = 100.0
            elif plan_time >= 10.0:
                s_time = 0.0
            else:
                s_time = 100.0 * (1.0 - (plan_time - 2.0) / 8.0)
                
            # Sub-score S_obstacle (0 blocked cells including footprint)
            s_obstacle = 100.0 if (eval_res["blocked_cells"] == 0 and s_success == 100.0) else 0.0
            
            # Sub-score S_cost
            s_cost = 100.0 - eval_res["avg_cost"]
            
            # Sub-score S_length (target ratio <= 1.35)
            ratio = eval_res["length_ratio"]
            if ratio <= 1.0:
                s_length = 100.0
            elif ratio >= 1.35:
                s_length = 0.0
            else:
                s_length = 100.0 * (1.0 - (ratio - 1.0) / 0.35)
                
            # Sub-score S_replan
            s_replan = replan_success_val
            
            # Compute final weighted Path Planning Score
            final_score = (0.25 * s_success + 
                           0.15 * s_time + 
                           0.25 * s_obstacle + 
                           0.15 * s_cost + 
                           0.10 * s_length + 
                           0.10 * s_replan)
            
            # PASS/FAIL Outcome check (threshold 85.0)
            outcome_val = "PASS" if final_score >= 85.0 else "FAIL"

            # Log results
            scenario_res = {
                "id": s["id"],
                "success": eval_res["success"],
                "planning_time_s": plan_time,
                "planning_time_score": s_time,
                "path_length_m": eval_res["length"],
                "length_ratio": ratio,
                "length_score": s_length,
                "blocked_cells": eval_res["blocked_cells"],
                "obstacle_avoidance_score": s_obstacle,
                "avg_cost": eval_res["avg_cost"],
                "cost_score": s_cost,
                "safety_margin_m": eval_res["safety_margin"],
                "replanning_score": s_replan,
                "replanning_time_s": replan_time,
                "final_score": final_score,
                "outcome": outcome_val
            }
            results[s["id"]] = scenario_res
            
            # Print Console Summary
            print(f"\n================ SCENARIO RESULTS: {s['id']} ================")
            print(f"Status:             {'SUCCESS' if eval_res['success'] else 'FAILED'}")
            print(f"Planning Time:      {plan_time:.3f} s  (Score: {s_time:.1f}/100)")
            print(f"Path Length:        {eval_res['length']:.2f} m  (Ratio: {ratio:.2f}, Score: {s_length:.1f}/100)")
            print(f"Blocked Cells:      {eval_res['blocked_cells']}  (Score: {s_obstacle:.1f}/100)")
            print(f"Average Path Cost:  {eval_res['avg_cost']:.1f}  (Score: {s_cost:.1f}/100)")
            print(f"Safety Margin:      {eval_res['safety_margin']:.2f} m")
            print(f"Replanning:         Score: {s_replan:.1f}/100 (Time: {replan_time:.3f} s)")
            print(f"-----------------------------------------------------")
            print(f"PATH PLANNING SCORE: {final_score:.2f} / 100")
            print(f"OUTCOME:             {outcome_val} (Required: >= 85.0/100)")
            print(f"=====================================================\n")
            
            # Generate post-test comparison PNG
            self.generate_comparison_plot(s, eval_res.get("coords", []), replan_path_coords, final_score, scenario_res)
            
        # Write JSON output
        out_json = "reports/results.json"
        with open(out_json, "w") as f:
            json.dump(results, f, indent=4)
        print(f"[INFO] Numerical results saved to: {out_json}")
        
        # Generate Markdown Report
        out_md = "reports/numerical_report.md"
        cwd = os.getcwd()
        md_content = []
        md_content.append("# 📈 ROS 2 Path Planning Benchmark Report\n")
        md_content.append("This report summarizes the performance evaluation of the global path planner. ")
        md_content.append("The target pass condition is a **Path Planning Score >= 85.0/100**.\n\n")
        
        # Add summary table
        md_content.append("## 📊 Performance Summary Table\n")
        md_content.append("| Scenario ID | Outcome | Score | Success | Planning Time | Blocked Cells | Avg Cost | Replanning |\n")
        md_content.append("| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: |\n")
        
        for sid, res in results.items():
            outcome_emoji = "✅ PASS" if res["outcome"] == "PASS" else "❌ FAIL"
            success_str = "YES" if res["success"] else "NO"
            md_content.append(
                f"| `{sid}` | **{outcome_emoji}** | `{res['final_score']:.1f}/100` | "
                f"{success_str} | `{res['planning_time_s']:.3f} s` | "
                f"`{res['blocked_cells']}` | `{res['avg_cost']:.1f}` | "
                f"`{res['replanning_score']:.1f}` |\n"
            )
        md_content.append("\n---\n\n## 🔍 Detailed Scenario Analyses & Plots\n\n")
        
        for s in self.scenarios:
            sid = s["id"]
            if sid not in results:
                continue
            res = results[sid]
            outcome_emoji = "✅ PASS" if res.get("outcome") == "PASS" else "❌ FAIL"
            plot_path = f"file://{os.path.join(cwd, 'reports', f'result_scenario_{sid}.png')}"
            
            md_content.append(f"### 📍 Scenario: `{sid}` ({outcome_emoji})\n")
            md_content.append(f"- **Final Score**: `{res.get('final_score', 0.0):.2f} / 100`\n")
            md_content.append(f"- **Planning Time**: `{res.get('planning_time_s', 0.0):.3f} s`\n")
            md_content.append(f"- **Path Length / Ratio**: `{res.get('path_length_m', 0.0):.2f} m` (Ratio: `{res.get('length_ratio', 0.0):.2f}`)\n")
            md_content.append(f"- **Footprint Collisions (Blocked Cells)**: `{res.get('blocked_cells', 0)}` cells\n")
            md_content.append(f"- **Average Traversed Cost**: `{res.get('avg_cost', 0.0):.1f}`\n")
            md_content.append(f"- **Safety Margin**: `{res.get('safety_margin_m', 0.0):.2f} m`\n")
            md_content.append(f"- **Replanning Status**: Score `{res.get('replanning_score', 0.0):.1f}/100` in `{res.get('replanning_time_s', 0.0):.3f} s`\n\n")
            
            md_content.append(f"#### Path Visualizer:\n")
            md_content.append(f"![{sid} Plot]({plot_path})\n\n")
            
            # Diagnostic hints
            if res.get("outcome") == "FAIL":
                md_content.append("> [!WARNING]\n")
                if res.get("blocked_cells", 0) > 0:
                    md_content.append(f"> **Collision Warning**: The path center line or its robot footprint ({s.get('robot_radius')}m) intersected wall obstacles. Implement obstacle inflation to fix this.\n")
                elif res.get("avg_cost", 0) > 20:
                    md_content.append("> **High Cost Warning**: The path went directly through high-slope or rough terrain. Tune the planner weight to prioritize safer detours.\n")
                elif res.get("planning_time_s", 0) > 2.0:
                    md_content.append("> **Latency Warning**: Planning exceeded the preferred 2.0s boundary. Optimize search iterations.\n")
            md_content.append("\n---\n\n")
            
        with open(out_md, "w") as f:
            f.writelines(md_content)
        print(f"[INFO] Numerical Markdown report compiled at: {out_md}")
        
        # Generate PDF Report
        out_pdf = "reports/numerical_report.pdf"
        self.compile_pdf_report(results, out_pdf)
        return True

    def generate_comparison_plot(self, scenario, plan_coords, replan_coords, score, metrics):
        map_image = scenario["map_image"]
        ref_path = scenario["reference_path"]
        resolution = scenario["resolution"]
        origin = scenario["origin"]
        robot_radius = scenario.get("robot_radius", 0.3)
        
        img = Image.open(map_image).convert("L")
        img_arr = np.array(img)
        
        fig, ax = plt.subplots(figsize=(10, 10))
        extent = [origin[0], origin[0] + img_arr.shape[1]*resolution, 
                  origin[1], origin[1] + img_arr.shape[0]*resolution]
        ax.imshow(img_arr, cmap='gray', origin='lower', extent=extent)
        
        # 1. Plot Reference Path (Green)
        ref_x = [p[0] for p in ref_path]
        ref_y = [p[1] for p in ref_path]
        ax.plot(ref_x, ref_y, 'g--', label='Reference Path (Perfect)', linewidth=2.5)
        
        # 2. Plot Planner Path (Red)
        if len(plan_coords) > 0:
            plan_x = [p[0] for p in plan_coords]
            plan_y = [p[1] for p in plan_coords]
            ax.plot(plan_x, plan_y, 'r-o', label='Planned Path', linewidth=2, markersize=4)
            
            # Draw circles to show footprint
            for idx, (px, py) in enumerate(plan_coords):
                # only plot every 5th footprint to prevent cluttering
                if idx % 5 == 0 or idx == len(plan_coords) - 1:
                    circle = plt.Circle((px, py), robot_radius, color='r', fill=False, linestyle=':', alpha=0.5)
                    ax.add_patch(circle)
        
        # 3. Plot Replan Path (Cyan) if exists
        if len(replan_coords) > 0:
            rep_x = [p[0] for p in replan_coords]
            rep_y = [p[1] for p in replan_coords]
            ax.plot(rep_x, rep_y, 'c-^', label='Replanned Path (Detour)', linewidth=1.5, markersize=4)
            
        # Draw Start and Goal
        ax.plot(scenario["start"][0], scenario["start"][1], 'bs', markersize=10, label='Start')
        ax.plot(scenario["goal"][0], scenario["goal"][1], 'r*', markersize=12, label='Goal')
        
        # Plot Dynamic Obstacles if any
        for obs in scenario.get("dynamic_obstacles", []):
            obs_circle = plt.Circle((obs["x"], obs["y"]), obs["radius"], color='orange', fill=True, alpha=0.6, label='Dynamic Obstacle')
            ax.add_patch(obs_circle)
        
        # Display scores box
        outcome = metrics.get("outcome", "FAIL")
        text_box = (
            f"Score: {score:.1f} / 100\n"
            f"Outcome: {outcome}\n"
            f"Success: {'YES' if metrics['success'] else 'NO'}\n"
            f"Time: {metrics['planning_time_s']:.3f} s\n"
            f"Length Ratio: {metrics['length_ratio']:.2f}\n"
            f"Safety Margin: {metrics['safety_margin_m']:.2f} m\n"
            f"Blocked Cells: {metrics['blocked_cells']}"
        )
        bg_color = '#d4edda' if outcome == 'PASS' else '#f8d7da'
        props = dict(boxstyle='round', facecolor=bg_color, alpha=0.9)
        ax.text(0.05, 0.95, text_box, transform=ax.transAxes, fontsize=12,
                verticalalignment='top', bbox=props)
        
        ax.set_title(f"Benchmark Results: {scenario['id']}")
        ax.set_xlabel("X (meters)")
        ax.set_ylabel("Y (meters)")
        ax.legend(loc='lower right')
        ax.grid(True, alpha=0.3)
        
        out_file = f"reports/result_scenario_{scenario['id']}.png"
        plt.savefig(out_file, bbox_inches='tight')
        plt.close()
        self.get_logger().info(f"Saved result comparison plot to: {out_file}")

    def compile_pdf_report(self, results, out_pdf):
        try:
            from reportlab.lib.pagesizes import letter
            from reportlab.lib import colors
            from reportlab.platypus import SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle, PageBreak, KeepTogether, Image as RLImage
            from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
            from reportlab.pdfgen import canvas
        except ImportError:
            self.get_logger().error("ReportLab not installed. Cannot generate PDF report.")
            return False

        class ReportCanvas(canvas.Canvas):
            def __init__(self, *args, **kwargs):
                super().__init__(*args, **kwargs)
                self.pages = []
            def showPage(self):
                self.pages.append(dict(self.__dict__))
                self._startPage()
            def save(self):
                num_pages = len(self.pages)
                for page in self.pages:
                    self.__dict__.update(page)
                    # Header
                    self.setFont("Helvetica-Bold", 8)
                    self.setFillColor(colors.HexColor("#1A365D"))
                    self.drawString(54, 750, "PATH PLANNING BENCHMARK PERFORMANCE REPORT")
                    self.setStrokeColor(colors.HexColor("#BDC3C7"))
                    self.setLineWidth(0.5)
                    self.line(54, 742, 558, 742)
                    
                    # Footer
                    self.line(54, 50, 558, 50)
                    self.setFont("Helvetica", 8)
                    self.setFillColor(colors.HexColor("#7F8C8D"))
                    self.drawString(54, 38, f"Generated: {time.strftime('%Y-%m-%d %H:%M:%S')}")
                    self.drawRightString(558, 38, f"Page {self._pageNumber} of {num_pages}")
                    super().showPage()
                super().save()

        doc = SimpleDocTemplate(
            out_pdf,
            pagesize=letter,
            leftMargin=54,
            rightMargin=54,
            topMargin=72,
            bottomMargin=72
        )
        
        styles = getSampleStyleSheet()
        
        title_style = ParagraphStyle(
            'ReportTitle',
            parent=styles['Normal'],
            fontName='Helvetica-Bold',
            fontSize=20,
            leading=24,
            textColor=colors.HexColor("#1A365D"),
            spaceAfter=4
        )
        
        subtitle_style = ParagraphStyle(
            'ReportSubtitle',
            parent=styles['Normal'],
            fontName='Helvetica',
            fontSize=9.5,
            leading=13,
            textColor=colors.HexColor("#4A5568"),
            spaceAfter=15
        )
        
        h1_style = ParagraphStyle(
            'SectionHeader',
            parent=styles['Heading1'],
            fontName='Helvetica-Bold',
            fontSize=13,
            leading=16,
            textColor=colors.HexColor("#1A365D"),
            spaceBefore=12,
            spaceAfter=6,
            keepWithNext=True
        )

        body_style = ParagraphStyle(
            'BodyTextCustom',
            parent=styles['Normal'],
            fontName='Helvetica',
            fontSize=9,
            leading=13,
            textColor=colors.HexColor("#2D3748")
        )
        
        table_cell_style = ParagraphStyle(
            'TableCellText',
            parent=styles['Normal'],
            fontName='Helvetica',
            fontSize=8,
            leading=10,
            textColor=colors.HexColor("#2D3748")
        )

        table_header_style = ParagraphStyle(
            'TableHeaderText',
            parent=styles['Normal'],
            fontName='Helvetica-Bold',
            fontSize=8,
            leading=10,
            textColor=colors.white
        )

        story = []
        
        # Header Info
        story.append(Paragraph("ROS 2 Path Planner Benchmarking Report", title_style))
        story.append(Paragraph(f"Performance Evaluation Results — Pass Threshold >= 85.0/100", subtitle_style))
        
        # Summary Table
        story.append(Paragraph("Performance Summary Table", h1_style))
        
        table_data = [[
            Paragraph("<b>Scenario ID</b>", table_header_style),
            Paragraph("<b>Outcome</b>", table_header_style),
            Paragraph("<b>Score</b>", table_header_style),
            Paragraph("<b>Success</b>", table_header_style),
            Paragraph("<b>Time</b>", table_header_style),
            Paragraph("<b>Collisions</b>", table_header_style),
            Paragraph("<b>Avg Cost</b>", table_header_style),
            Paragraph("<b>Replan</b>", table_header_style)
        ]]
        
        for sid, res in results.items():
            outcome_text = f"<b><font color='green'>PASS</font></b>" if res["outcome"] == "PASS" else "<b><font color='red'>FAIL</font></b>"
            success_str = "YES" if res["success"] else "NO"
            table_data.append([
                Paragraph(f"<code>{sid}</code>", table_cell_style),
                Paragraph(outcome_text, table_cell_style),
                Paragraph(f"{res['final_score']:.1f}/100", table_cell_style),
                Paragraph(success_str, table_cell_style),
                Paragraph(f"{res['planning_time_s']:.3f} s", table_cell_style),
                Paragraph(f"{res['blocked_cells']}", table_cell_style),
                Paragraph(f"{res['avg_cost']:.1f}", table_cell_style),
                Paragraph(f"{res['replanning_score']:.1f}", table_cell_style)
            ])
            
        t_summary = Table(table_data, colWidths=[120, 50, 50, 45, 55, 55, 50, 45])
        t_summary.setStyle(TableStyle([
            ('BACKGROUND', (0,0), (-1,0), colors.HexColor("#1A365D")),
            ('GRID', (0,0), (-1,-1), 0.5, colors.HexColor("#CBD5E0")),
            ('VALIGN', (0,0), (-1,-1), 'MIDDLE'),
            ('ROWBACKGROUNDS', (0,1), (-1,-1), [colors.white, colors.HexColor("#F7FAFC")]),
            ('TOPPADDING', (0,0), (-1,-1), 5),
            ('BOTTOMPADDING', (0,0), (-1,-1), 5),
        ]))
        story.append(t_summary)
        story.append(Spacer(1, 10))
        
        story.append(Paragraph("Detailed Scenario Plots & Insights", h1_style))
        
        for s in self.scenarios:
            sid = s["id"]
            if sid not in results:
                continue
            res = results[sid]
            outcome_str = f"<font color='green'><b>PASS</b></font>" if res["outcome"] == "PASS" else "<font color='red'><b>FAIL</b></font>"
            
            detail_p = (
                f"<b>Scenario:</b> <code>{sid}</code> ({outcome_str})<br/>"
                f"• Final Score: <b>{res['final_score']:.1f}/100</b> | Planning Time: {res['planning_time_s']:.3f} s<br/>"
                f"• Path Length: {res['path_length_m']:.2f} m (Ratio: {res['length_ratio']:.2f}) | Safety Margin: {res['safety_margin_m']:.2f} m<br/>"
                f"• Footprint Collisions: {res['blocked_cells']} | Avg Traversed Cost: {res['avg_cost']:.1f}<br/>"
                f"• Replanning Score: {res['replanning_score']:.1f}/100 (Time: {res['replanning_time_s']:.3f} s)"
            )
            
            plot_file = f"reports/result_scenario_{sid}.png"
            flowables = []
            flowables.append(Paragraph(detail_p, body_style))
            flowables.append(Spacer(1, 4))
            
            if os.path.exists(plot_file):
                flowables.append(RLImage(plot_file, width=280, height=280))
                
            if res["outcome"] == "FAIL":
                warn_text = ""
                if res["blocked_cells"] > 0:
                    warn_text = f"<b>Collision Warning</b>: Footprint radius ({s.get('robot_radius')}m) intersected obstacles. Implement costmap obstacle inflation."
                elif res["avg_cost"] > 20:
                    warn_text = "<b>High Cost Warning</b>: Planner crossed rough/steep slopes. Adjust weights to prefer detours."
                elif res["planning_time_s"] > 2.0:
                    warn_text = "<b>Latency Warning</b>: Planning latency exceeded boundary. Optimize search iterations."
                
                if warn_text:
                    flowables.append(Paragraph(f"<i><font color='red'>{warn_text}</font></i>", body_style))
            
            flowables.append(Spacer(1, 10))
            story.append(KeepTogether(flowables))
            
        doc.build(story, canvasmaker=ReportCanvas)
        self.get_logger().info(f"Numerical PDF report compiled at: {out_pdf}")
        return True


def main(args=None):
    # Parse CLI Arguments first, filtering out ROS-specific launch arguments
    from rclpy.utilities import remove_ros_args
    clean_args = remove_ros_args(args=sys.argv)[1:]

    parser = argparse.ArgumentParser(description="ROS 2 Global Path Planning Benchmarking Tool")
    parser.add_argument("--config", type=str, default="config/scenarios.yaml", help="Path to config/scenarios.yaml")
    parser.add_argument("--verify", action="store_true", help="Generate pre-run verification PNGs and exit")
    parser.add_argument("--clean", action="store_true", help="Kill stale ROS 2 and Gazebo processes before running")
    parser.add_argument("--scenario_id", type=str, default=None, help="Select a specific scenario to run")
    
    cli_args = parser.parse_args(clean_args)
    
    # ROS 2 init args
    rclpy.init(args=args)
    
    # 1. Clean old processes if requested
    if cli_args.clean:
        clean_old_processes()
    
    # 2. Setup mock data if missing
    generate_synthetic_maps()
    generate_default_scenarios_yaml()
    
    # Initialize Node
    benchmarker = PathBenchmarker(cli_args.config)
    
    if cli_args.verify:
        benchmarker.verify_scenarios()
        benchmarker.destroy_node()
        rclpy.shutdown()
        sys.exit(0)
        
    try:
        # Run tests
        success = benchmarker.run_tests(cli_args.scenario_id)
        if not success:
            print("[ERROR] Test execution failed.")
    except Exception as e:
        print(f"[ERROR] Exception during benchmark: {e}")
    finally:
        benchmarker.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()


