#!/usr/bin/env python3

import time
import heapq
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from nav_msgs.msg import OccupancyGrid, Path
from nav2_msgs.action import ComputePathToPose
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import ParameterDescriptor

class MockPlanner(Node):
    def __init__(self):
        super().__init__("mock_planner")
        
        # Declare parameter to toggle planner quality
        self.declare_parameter(
            "use_astar", 
            True,
            ParameterDescriptor(description="If true, use A* pathfinding. If false, use straight line.")
        )
        
        self.map_data = None
        self.map_width = 0
        self.map_height = 0
        self.map_resolution = 0.05
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0
        
        # Subscriber to Costmap
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            "/global_costmap/costmap",
            self.map_callback,
            10
        )
        
        # Action Server
        self.action_server = ActionServer(
            self,
            ComputePathToPose,
            "compute_path_to_pose",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )
        
        self.get_logger().info("MockPlanner Node Started.")
        self.get_logger().info("Subscribed to /global_costmap/costmap")
        self.get_logger().info("Hosting Action Server: compute_path_to_pose")

    def map_callback(self, msg):
        self.map_resolution = msg.info.resolution
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y
        
        # Reshape data to 2D grid
        # Invert flips to match orientation if flipped in benchmarker
        # The data is published bottom-left origin
        self.map_data = np.array(msg.data, dtype=np.int8).reshape((self.map_height, self.map_width))
        self.get_logger().info(f"Received map: {self.map_width}x{self.map_height} at resolution {self.map_resolution}", throttle_duration_sec=5)

    def goal_callback(self, goal_request):
        self.get_logger().info("Received a path planning request.")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Received request to cancel planning.")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        self.get_logger().info("Executing path planning request...")
        feedback_msg = ComputePathToPose.Feedback()
        
        goal = goal_handle.request
        start_pose = goal.start
        goal_pose = goal.goal
        
        start_x = start_pose.pose.position.x
        start_y = start_pose.pose.position.y
        goal_x = goal_pose.pose.position.x
        goal_y = goal_pose.pose.position.y
        
        use_astar = self.get_parameter("use_astar").value
        
        start_time = self.get_clock().now()
        
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        path_points = []
        
        if not use_astar or self.map_data is None:
            # Plan a simple straight line path
            self.get_logger().info(f"Generating Straight Line Path from ({start_x:.2f}, {start_y:.2f}) to ({goal_x:.2f}, {goal_y:.2f})...")
            dist = np.sqrt((goal_x - start_x)**2 + (goal_y - start_y)**2)
            steps = int(dist / 0.1) # Point every 10cm
            if steps < 2:
                steps = 2
            for i in range(steps + 1):
                t = i / steps
                x = start_x + t * (goal_x - start_x)
                y = start_y + t * (goal_y - start_y)
                path_points.append((x, y))
        else:
            # Run A* Pathfinding
            self.get_logger().info(f"Generating A* Path from ({start_x:.2f}, {start_y:.2f}) to ({goal_x:.2f}, {goal_y:.2f})...")
            
            # Convert meters to grid indices
            start_col = int((start_x - self.map_origin_x) / self.map_resolution)
            start_row = int((start_y - self.map_origin_y) / self.map_resolution)
            goal_col = int((goal_x - self.map_origin_x) / self.map_resolution)
            goal_row = int((goal_y - self.map_origin_y) / self.map_resolution)
            
            # Verify coordinates are in map
            start_col = np.clip(start_col, 0, self.map_width - 1)
            start_row = np.clip(start_row, 0, self.map_height - 1)
            goal_col = np.clip(goal_col, 0, self.map_width - 1)
            goal_row = np.clip(goal_row, 0, self.map_height - 1)
            
            grid_path = self.astar((start_row, start_col), (goal_row, goal_col))
            if grid_path:
                # Convert grid path back to meters
                for r, c in grid_path:
                    x = self.map_origin_x + c * self.map_resolution + self.map_resolution/2
                    y = self.map_origin_y + r * self.map_resolution + self.map_resolution/2
                    path_points.append((x, y))
            else:
                self.get_logger().warn("A* failed to find a path. Falling back to straight line.")
                # Fallback to straight line on failure so we still return something
                dist = np.sqrt((goal_x - start_x)**2 + (goal_y - start_y)**2)
                steps = int(dist / 0.1)
                for i in range(steps + 1):
                    t = i / steps
                    x = start_x + t * (goal_x - start_x)
                    y = start_y + t * (goal_y - start_y)
                    path_points.append((x, y))
        
        # Populate path_msg
        for px, py in path_points:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(px)
            pose.pose.position.y = float(py)
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
            
        end_time = self.get_clock().now()
        duration_ns = (end_time - start_time).nanoseconds
        
        result = ComputePathToPose.Result()
        result.path = path_msg
        result.planning_time.sec = int(duration_ns // 1e9)
        result.planning_time.nanosec = int(duration_ns % 1e9)
        
        goal_handle.succeed()
        self.get_logger().info(f"Path generation complete. Planning time: {duration_ns/1e9:.3f} s. Waypoints: {len(path_points)}")
        return result

    def astar(self, start, goal):
        # 8-connected grid offsets
        neighbors = [
            (0, 1, 1.0), (1, 0, 1.0), (0, -1, 1.0), (-1, 0, 1.0),
            (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414)
        ]
        
        open_set = []
        heapq.heappush(open_set, (0.0, start))
        
        came_from = {}
        g_score = {start: 0.0}
        
        # Heuristic: Euclidean distance
        def heuristic(p1, p2):
            return np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
            
        iterations = 0
        max_iterations = 200000 # Safety limit
        
        while open_set and iterations < max_iterations:
            iterations += 1
            current_f, current = heapq.heappop(open_set)
            
            if current == goal:
                # Reconstruct path
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                return path
                
            for dr, dc, cost_multiplier in neighbors:
                neighbor = (current[0] + dr, current[1] + dc)
                
                # Check grid boundaries
                if not (0 <= neighbor[0] < self.map_height and 0 <= neighbor[1] < self.map_width):
                    continue
                    
                # Check if cell is an obstacle (cost >= 100)
                cell_cost = self.map_data[neighbor[0], neighbor[1]]
                if cell_cost >= 100:
                    continue
                    
                # Calculate movement cost
                # Higher cost cells penalty
                move_cost = cost_multiplier * (1.0 + float(cell_cost) / 10.0)
                tentative_g_score = g_score[current] + move_cost
                
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score = tentative_g_score + heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score, neighbor))
                    
        return None # Failed to find path

def main(args=None):
    rclpy.init(args=args)
    node = MockPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
