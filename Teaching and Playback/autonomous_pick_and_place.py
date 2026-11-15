#!/usr/bin/env python3

import json
import asyncio
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor

from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import RobotState
from geometry_msgs.msg import Pose, Point
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class AutonomousPickAndPlace(Node):
    def __init__(self):
        super().__init__('autonomous_pick_place')
        
        # Service client for cartesian path
        self.cartesian_client = self.create_client(GetCartesianPath, '/compute_cartesian_path')

        # Action clients for executing trajectories
        self.execute_client = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')

        # Gripper Action clients
        self.gripper_client_left = ActionClient(self, FollowJointTrajectory, '/left_gripper_controller/follow_joint_trajectory')
        self.gripper_client_right = ActionClient(self, FollowJointTrajectory, '/right_gripper_controller/follow_joint_trajectory')

        self.left_gripper_joint = 'openarm_left_finger_joint1'
        self.right_gripper_joint = 'openarm_right_finger_joint1'

        self.MAX_GRIPPER_RAD = 0.7854

        # Loaded reference poses
        self.sequence_poses = {}

        # Target coordinates from camera
        self.left_coords = None
        self.right_coords = None

        # Subscribers
        self.left_target_sub = self.create_subscription(Point, '/left_arm_target', self.left_target_cb, 10)
        self.right_target_sub = self.create_subscription(Point, '/right_arm_target', self.right_target_cb, 10)

    def left_target_cb(self, msg: Point):
        if self.left_coords is None:
            self.left_coords = (msg.x, msg.y)
            self.get_logger().info(f"Received LEFT target (Red Block) from camera: X={msg.x:.3f}, Y={msg.y:.3f}")

    def right_target_cb(self, msg: Point):
        if self.right_coords is None:
            self.right_coords = (msg.x, msg.y)
            self.get_logger().info(f"Received RIGHT target (Blue Block) from camera: X={msg.x:.3f}, Y={msg.y:.3f}")

    async def wait_for_servers(self):
        self.get_logger().info("Waiting for compute_cartesian_path service...")
        while not self.cartesian_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting again...")
            
        self.get_logger().info("Waiting for ExecuteTrajectory action server...")
        self.execute_client.wait_for_server()
        
        self.get_logger().info("Waiting for left gripper action server...")
        self.gripper_client_left.wait_for_server()
        self.get_logger().info("Waiting for right gripper action server...")
        self.gripper_client_right.wait_for_server()
        self.get_logger().info("All servers online!")

    def scale_trajectory_speed(self, trajectory, scale=0.8):
        factor = 1.0 / scale
        for point in trajectory.joint_trajectory.points:
            if point.velocities:
                point.velocities = [v * scale for v in point.velocities]
            if point.accelerations:
                point.accelerations = [a * scale * scale for a in point.accelerations]
            
            sec = point.time_from_start.sec
            nanosec = point.time_from_start.nanosec
            total_seconds = sec + nanosec * 1e-9
            
            new_total_seconds = total_seconds * factor
            
            point.time_from_start.sec = int(new_total_seconds)
            point.time_from_start.nanosec = int((new_total_seconds - int(new_total_seconds)) * 1e9)
        return trajectory

    async def await_ros_future(self, future):
        while rclpy.ok() and not future.done():
            await asyncio.sleep(0.02)
        if not future.done():
            raise RuntimeError("ROS future did not complete before shutdown")
        return future.result()

    async def move_to_pose(self, arm_side, x, y, z, q_dict, step_name):
        self.get_logger().info(f"[{arm_side.upper()}] {step_name}: Cartesian Path to (x={x:.3f}, y={y:.3f}, z={z:.3f})")
        
        req = GetCartesianPath.Request()
        req.header.frame_id = "world"
        
        if arm_side == "left":
            req.group_name = "left_arm"
            req.link_name = "openarm_left_ee_base_link"
        else:
            req.group_name = "right_arm"
            req.link_name = "openarm_right_ee_base_link"

        target_pose = Pose()
        target_pose.position.x = float(x)
        target_pose.position.y = float(y)
        target_pose.position.z = float(z)
        target_pose.orientation.x = float(q_dict["x"])
        target_pose.orientation.y = float(q_dict["y"])
        target_pose.orientation.z = float(q_dict["z"])
        target_pose.orientation.w = float(q_dict["w"])
        
        req.waypoints = [target_pose]
        req.max_step = 0.02  # 2 cm resolution for faster calculation
        req.jump_threshold = 0.0
        req.avoid_collisions = True

        # Call Cartesian Service
        future = self.cartesian_client.call_async(req)
        response = await self.await_ros_future(future)
        
        if response.fraction < 0.99:
            self.get_logger().error(f"[{arm_side.upper()}] Cartesian Path failed! Only {response.fraction * 100:.1f}% computed.")
            self.get_logger().error(f"Error code: {response.error_code.val}")
            return False

        # Keep trajectory at 100% speed
        scaled_solution = self.scale_trajectory_speed(response.solution, 1.0)

        # Execute Trajectory
        goal_msg = ExecuteTrajectory.Goal()
        goal_msg.trajectory = scaled_solution
        
        send_goal_future = self.execute_client.send_goal_async(goal_msg)
        goal_handle = await self.await_ros_future(send_goal_future)
        if not goal_handle.accepted:
            self.get_logger().error(f"[{arm_side.upper()}] ExecuteTrajectory rejected goal!")
            return False

        result_future = goal_handle.get_result_async()
        result = await self.await_ros_future(result_future)
        
        error_code = result.result.error_code.val
        if error_code == 1:
            # Minimal settle time after motion
            await asyncio.sleep(0.05)
            return True
        else:
            self.get_logger().error(f"[{arm_side.upper()}] Execution failed with error_code: {error_code}")
            return False

    async def move_gripper(self, arm_side, percent, step_name):
        # Calculate target radian
        if arm_side == "left":
            target_rad = self.MAX_GRIPPER_RAD * (percent / 100.0)
        else:
            # Right arm gripper joint is negative
            target_rad = -self.MAX_GRIPPER_RAD * (percent / 100.0)
            
        self.get_logger().info(f"[{arm_side.upper()}] {step_name}: Gripper -> {percent}% ({target_rad:.3f} rad)")
        
        goal_msg = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        
        if arm_side == "left":
            traj.joint_names = [self.left_gripper_joint]
            client = self.gripper_client_left
        else:
            traj.joint_names = [self.right_gripper_joint]
            client = self.gripper_client_right
        
        point = JointTrajectoryPoint()
        point.positions = [target_rad]
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = int(0.5 * 1e9)  # 0.5 seconds for gripper
        traj.points.append(point)
        
        goal_msg.trajectory = traj
        
        send_goal_future = client.send_goal_async(goal_msg)
        goal_handle = await self.await_ros_future(send_goal_future)
        if not goal_handle.accepted:
            self.get_logger().error(f"[{arm_side.upper()}] Gripper rejected!")
            return False
            
        result_future = goal_handle.get_result_async()
        result = await self.await_ros_future(result_future)
        
        if result.result.error_code == 0:
            await asyncio.sleep(0.05) # Minimal settle time
            return True
        else:
            self.get_logger().error(f"[{arm_side.upper()}] Gripper failed: {result.result.error_code}")
            return False

    async def execute_left_sequence(self, block_x, block_y):
        arm_side = "left_arm"
        poses = self.sequence_poses[arm_side]
        
        def get_q(step_idx):
            return poses[step_idx]["orientation"]

        # Step 1: Pre-grasp
        if not await self.move_to_pose("left", -0.1, block_y, 0.47, get_q(0), "Step 1 (Pre-grasp)"): return
        if not await self.move_gripper("left", 0.0, "Step 1 (Gripper)"): return
        
        # Step 2: Open
        if not await self.move_to_pose("left", -0.1, block_y, 0.47, get_q(1), "Step 2 (Open)"): return
        if not await self.move_gripper("left", 60.0, "Step 2 (Gripper)"): return
        
        # Step 3: Go near block
        if not await self.move_to_pose("left", block_x - 0.15, block_y, 0.47, get_q(2), "Step 3 (Go Near)"): return
        if not await self.move_gripper("left", 60.0, "Step 3 (Gripper)"): return
        
        # Step 4: Close
        if not await self.move_to_pose("left", block_x - 0.15, block_y, 0.47, get_q(3), "Step 4 (Close)"): return
        if not await self.move_gripper("left", 20.0, "Step 4 (Gripper)"): return
        
        # Step 5: Ascend and go to bin
        if not await self.move_to_pose("left", 0.153, 0.31, 0.51, get_q(4), "Step 5 (To Bin)"): return
        if not await self.move_gripper("left", 20.0, "Step 5 (Gripper)"): return
        
        # Step 6: Release block
        if not await self.move_to_pose("left", 0.153, 0.31, 0.51, get_q(5), "Step 6 (Release)"): return
        if not await self.move_gripper("left", 60.0, "Step 6 (Gripper)"): return
        
        # Step 7: Home for left arm
        if not await self.move_to_pose("left", 0.154, 0.180, 0.593, get_q(6), "Step 7 (Home)"): return
        if not await self.move_gripper("left", 20.0, "Step 7 (Gripper)"): return

        self.get_logger().info("[LEFT ARM] SEQUENCE COMPLETE!")

    async def execute_right_sequence(self, block_x, block_y):
        arm_side = "right_arm"
        poses = self.sequence_poses[arm_side]
        
        def get_q(step_idx):
            return poses[step_idx]["orientation"]

        # Step 1: Pre-grasp
        if not await self.move_to_pose("right", -0.1, block_y, 0.47, get_q(0), "Step 1 (Pre-grasp)"): return
        if not await self.move_gripper("right", 0.0, "Step 1 (Gripper)"): return
        
        # Step 2: Open
        if not await self.move_to_pose("right", -0.1, block_y, 0.47, get_q(1), "Step 2 (Open)"): return
        if not await self.move_gripper("right", 60.0, "Step 2 (Gripper)"): return
        
        # Step 3: Go near block
        if not await self.move_to_pose("right", block_x - 0.15, block_y, 0.47, get_q(2), "Step 3 (Go Near)"): return
        if not await self.move_gripper("right", 60.0, "Step 3 (Gripper)"): return
        
        # Step 4: Close
        if not await self.move_to_pose("right", block_x - 0.15, block_y, 0.47, get_q(3), "Step 4 (Close)"): return
        if not await self.move_gripper("right", 20.0, "Step 4 (Gripper)"): return
        
        # Step 5: Ascend and go to bin
        if not await self.move_to_pose("right", 0.153, -0.31, 0.51, get_q(4), "Step 5 (To Bin)"): return
        if not await self.move_gripper("right", 20.0, "Step 5 (Gripper)"): return
        
        # Step 6: Release block
        if not await self.move_to_pose("right", 0.153, -0.31, 0.51, get_q(5), "Step 6 (Release)"): return
        if not await self.move_gripper("right", 60.0, "Step 6 (Gripper)"): return
        
        # Step 7: Home
        if not await self.move_to_pose("right", 0.154, -0.180, 0.593, get_q(6), "Step 7 (Home)"): return
        if not await self.move_gripper("right", 20.0, "Step 7 (Gripper)"): return

        self.get_logger().info("[RIGHT ARM] SEQUENCE COMPLETE!")

    async def run_all(self):
        await self.wait_for_servers()
        
        try:
            with open(
                Path(__file__).resolve().parent / "sequence_poses.json", "r"
            ) as f:
                self.sequence_poses = json.load(f)
        except FileNotFoundError:
            self.get_logger().error("Could not find 'sequence_poses.json'. Please run compute_fk_sequence.py first.")
            return

        self.get_logger().info("Waiting for block coordinates from camera...")
        
        # Wait until we have both coordinates from the subscribers
        while rclpy.ok() and (self.left_coords is None or self.right_coords is None):
            await asyncio.sleep(0.1)
            
        if not rclpy.ok():
            return
            
        left_block_x, left_block_y = self.left_coords
        right_block_x, right_block_y = self.right_coords

        self.get_logger().info("========================================")
        self.get_logger().info(f"STARTING AUTONOMOUS SEQUENCE")
        self.get_logger().info(f"Targeting LEFT (Red) at X={left_block_x:.3f}, Y={left_block_y:.3f}")
        self.get_logger().info(f"Targeting RIGHT (Blue) at X={right_block_x:.3f}, Y={right_block_y:.3f}")
        self.get_logger().info("========================================")

        # Run both arms concurrently
        left_task = asyncio.create_task(self.execute_left_sequence(left_block_x, left_block_y))
        right_task = asyncio.create_task(self.execute_right_sequence(right_block_x, right_block_y))

        await asyncio.gather(left_task, right_task)
        
        self.get_logger().info("========================================")
        self.get_logger().info(" AUTONOMOUS OPERATION COMPLETE")
        self.get_logger().info("========================================")

def main(args=None):
    rclpy.init(args=args)
    node = AutonomousPickAndPlace()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    
    loop = asyncio.get_event_loop()
    try:
        loop.run_until_complete(asyncio.gather(
            node.run_all(),
            loop.run_in_executor(None, executor.spin)
        ))
    except (KeyboardInterrupt, RuntimeError):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
