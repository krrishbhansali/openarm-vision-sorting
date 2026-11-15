#!/usr/bin/env python3

import json
import asyncio
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, PositionConstraint, OrientationConstraint, BoundingVolume
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class DynamicPickAndPlace(Node):
    def __init__(self):
        super().__init__('dynamic_pick_place')
        
        # Action clients Left
        self.move_client_left = ActionClient(self, MoveGroup, '/move_action')
        self.gripper_client_left = ActionClient(self, FollowJointTrajectory, '/left_gripper_controller/follow_joint_trajectory')

        # Action clients Right
        self.move_client_right = ActionClient(self, MoveGroup, '/move_action')
        self.gripper_client_right = ActionClient(self, FollowJointTrajectory, '/right_gripper_controller/follow_joint_trajectory')

        self.left_gripper_joint = 'openarm_left_finger_joint1'
        self.right_gripper_joint = 'openarm_right_finger_joint1'

        self.MAX_GRIPPER_RAD = 0.7854

        # Loaded reference poses
        self.sequence_poses = {}

    async def wait_for_servers(self):
        self.get_logger().info("Waiting for MoveGroup action server...")
        self.move_client_left.wait_for_server()
        self.get_logger().info("Waiting for left gripper action server...")
        self.gripper_client_left.wait_for_server()
        self.get_logger().info("Waiting for right gripper action server...")
        self.gripper_client_right.wait_for_server()
        self.get_logger().info("All servers online!")

    async def await_ros_future(self, future):
        while rclpy.ok() and not future.done():
            await asyncio.sleep(0.02)
        if not future.done():
            raise RuntimeError("ROS future did not complete before shutdown")
        return future.result()

    async def move_to_pose(self, arm_side, x, y, z, q_dict, step_name):
        self.get_logger().info(f"[{arm_side.upper()}] {step_name}: Moving to (x={x:.3f}, y={y:.3f}, z={z:.3f})")
        
        goal_msg = MoveGroup.Goal()
        req = MotionPlanRequest()
        req.workspace_parameters.header.frame_id = "world"
        
        if arm_side == "left":
            req.group_name = "left_arm"
            client = self.move_client_left
            ee_link = "openarm_left_ee_base_link"
        else:
            req.group_name = "right_arm"
            client = self.move_client_right
            ee_link = "openarm_right_ee_base_link"

        req.num_planning_attempts = 10
        req.allowed_planning_time = 10.0
        req.max_velocity_scaling_factor = 0.35
        req.max_acceleration_scaling_factor = 0.35

        constraints = Constraints()

        # Position Constraint
        pos = PositionConstraint()
        pos.header.frame_id = "world"
        pos.link_name = ee_link
        
        box = SolidPrimitive()
        box.type = SolidPrimitive.SPHERE
        box.dimensions = [0.01]  # 1cm tolerance sphere
        
        pose = Pose()
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = float(z)
        pose.orientation.w = 1.0
        
        pos.constraint_region.primitives.append(box)
        pos.constraint_region.primitive_poses.append(pose)
        pos.weight = 1.0

        # Orientation Constraint
        ori = OrientationConstraint()
        ori.header.frame_id = "world"
        ori.link_name = ee_link
        ori.orientation.x = float(q_dict["x"])
        ori.orientation.y = float(q_dict["y"])
        ori.orientation.z = float(q_dict["z"])
        ori.orientation.w = float(q_dict["w"])
        
        ori.absolute_x_axis_tolerance = 0.1
        ori.absolute_y_axis_tolerance = 0.1
        ori.absolute_z_axis_tolerance = 0.1
        ori.weight = 1.0

        constraints.position_constraints.append(pos)
        constraints.orientation_constraints.append(ori)
        
        req.goal_constraints.append(constraints)
        goal_msg.request = req

        send_goal_future = client.send_goal_async(goal_msg)
        goal_handle = await self.await_ros_future(send_goal_future)
        if not goal_handle.accepted:
            self.get_logger().error(f"[{arm_side.upper()}] MoveGroup rejected pose goal!")
            return False

        result_future = goal_handle.get_result_async()
        result = await self.await_ros_future(result_future)
        
        error_code = result.result.error_code.val
        if error_code == 1:
            return True
        else:
            self.get_logger().error(f"[{arm_side.upper()}] Pose movement failed with error_code: {error_code}")
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
        point.time_from_start.sec = 2
        point.time_from_start.nanosec = 0
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
            await asyncio.sleep(1.0) # Settle time
            return True
        else:
            self.get_logger().error(f"[{arm_side.upper()}] Gripper failed: {result.result.error_code}")
            return False

    async def execute_left_sequence(self, block_x, block_y):
        arm_side = "left_arm"
        poses = self.sequence_poses[arm_side]
        
        # Helper to get the recorded orientation for a specific step index (0 to 6)
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
        
        # Step 5: Ascend to bin
        if not await self.move_to_pose("left", 0.153, 0.31, 0.51, get_q(4), "Step 5 (To Bin)"): return
        if not await self.move_gripper("left", 20.0, "Step 5 (Gripper)"): return
        
        # Step 6: Release block
        if not await self.move_to_pose("left", 0.153, 0.31, 0.51, get_q(5), "Step 6 (Release)"): return
        if not await self.move_gripper("left", 60.0, "Step 6 (Gripper)"): return
        
        # Step 7: Home
        if not await self.move_to_pose("left", 0.154, 0.180, 0.593, get_q(6), "Step 7 (Home)"): return
        if not await self.move_gripper("left", 20.0, "Step 7 (Gripper)"): return

        self.get_logger().info("[LEFT ARM] SEQUENCE COMPLETE!")

    async def execute_right_sequence(self, block_x, block_y):
        arm_side = "right_arm"
        poses = self.sequence_poses[arm_side]
        
        # Helper to get the recorded orientation for a specific step index (0 to 6)
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
        
        # Step 5: Ascend to bin
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
            with open("sequence_poses.json", "r") as f:
                self.sequence_poses = json.load(f)
        except FileNotFoundError:
            self.get_logger().error("Could not find 'sequence_poses.json'. Please run compute_fk_sequence.py first.")
            return

        # Block locations based on the current MuJoCo XML
        left_block_x = 0.25
        left_block_y = 0.1
        
        right_block_x = 0.25
        right_block_y = -0.1

        # Run both arms concurrently
        left_task = asyncio.create_task(self.execute_left_sequence(left_block_x, left_block_y))
        right_task = asyncio.create_task(self.execute_right_sequence(right_block_x, right_block_y))

        await asyncio.gather(left_task, right_task)
        
        self.get_logger().info("========================================")
        self.get_logger().info(" BIMANUAL DYNAMIC OPERATION COMPLETE")
        self.get_logger().info("========================================")

def main(args=None):
    rclpy.init(args=args)
    node = DynamicPickAndPlace()
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
