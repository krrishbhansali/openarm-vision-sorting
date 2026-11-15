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

class CartesianPickAndPlace(Node):
    def __init__(self):
        super().__init__('bimanual_pick_place')
        
        # Action clients Left
        self.move_client_left = ActionClient(self, MoveGroup, '/move_action')
        self.gripper_client_left = ActionClient(self, FollowJointTrajectory, '/left_gripper_controller/follow_joint_trajectory')

        # Action clients Right
        self.move_client_right = ActionClient(self, MoveGroup, '/move_action')
        self.gripper_client_right = ActionClient(self, FollowJointTrajectory, '/right_gripper_controller/follow_joint_trajectory')

        self.left_gripper_joint = 'openarm_left_finger_joint1'
        self.right_gripper_joint = 'openarm_right_finger_joint1'
        
        self.GRIPPER_OPEN = 0.0
        self.GRIPPER_CLOSE_85 = 0.7854 * 0.85  # ~0.667

        # Reference poses loaded from JSON
        self.reference_poses = {}

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
        self.get_logger().info(f"[{arm_side.upper()}] {step_name}: Moving to (x={x:.2f}, y={y:.2f}, z={z:.2f})")
        
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
        
        # Give it a tiny bit of freedom to find a valid IK solution
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

    async def move_gripper(self, arm_side, target_rad, step_name):
        self.get_logger().info(f"[{arm_side.upper()}] {step_name}: Gripper -> {target_rad:.2f} rad")
        
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

    async def pick_and_place_sequence(self, arm_side, block_pose, bin_pose):
        if arm_side not in self.reference_poses:
            self.get_logger().error(f"No reference pose found for {arm_side}. Skipping.")
            return

        ref_quat = self.reference_poses[arm_side]["orientation"]
        home_pos = self.reference_poses[arm_side]["position"]
        
        # Step 1: Pre-grasp (x - 0.15)
        pre_x = block_pose["x"] - 0.15
        if not await self.move_to_pose(arm_side, pre_x, block_pose["y"], block_pose["z"], ref_quat, "Step 1 (Pre-grasp)"): return
        
        # Step 2: Open gripper
        if not await self.move_gripper(arm_side, self.GRIPPER_OPEN, "Step 2 (Open)"): return
        
        # Step 3: Slide +x
        # We slide to x=0.23 to comfortably enclose the 0.02m block
        grasp_x = block_pose["x"] - 0.02
        if not await self.move_to_pose(arm_side, grasp_x, block_pose["y"], block_pose["z"], ref_quat, "Step 3 (Slide)"): return
        
        # Step 4: Close gripper 85%
        if not await self.move_gripper(arm_side, self.GRIPPER_CLOSE_85, "Step 4 (Close)"): return
        
        # Step 5: Move to bin (x - 0.05, y - 0.05, z + 0.1)
        # Note: If right side, maybe y is negative so -0.05 pushes it further out.
        # We'll exactly follow the prompt: y - 0.05.
        drop_x = bin_pose["x"] - 0.05
        drop_y = bin_pose["y"] - 0.05
        drop_z = bin_pose["z"] + 0.1
        if not await self.move_to_pose(arm_side, drop_x, drop_y, drop_z, ref_quat, "Step 5 (Drop location)"): return
        
        # Step 6: Open gripper fully
        if not await self.move_gripper(arm_side, self.GRIPPER_OPEN, "Step 6 (Release)"): return
        
        # Step 7: Go to home position
        if not await self.move_to_pose(arm_side, home_pos["x"], home_pos["y"], home_pos["z"], ref_quat, "Step 7 (Home)"): return
        
        self.get_logger().info(f"[{arm_side.upper()}] Pick and Place SEQUENCE COMPLETE!")

    async def run_all(self):
        await self.wait_for_servers()
        
        # Load the reference Cartesian poses generated by compute_fk.py
        try:
            with open("horizontal_poses.json", "r") as f:
                self.reference_poses = json.load(f)
        except FileNotFoundError:
            self.get_logger().error("Could not find 'horizontal_poses.json'. Please run compute_fk.py first.")
            return

        # Hardcoded object locations based on the current MuJoCo XML
        left_block = {"x": 0.25, "y": 0.1, "z": 0.0}
        left_bin = {"x": 0.3, "y": 0.3, "z": -0.275}
        
        right_block = {"x": 0.25, "y": -0.1, "z": 0.0}
        right_bin = {"x": 0.3, "y": -0.3, "z": -0.275}

        # Run both arms concurrently!
        left_task = asyncio.create_task(self.pick_and_place_sequence("left_arm", left_block, left_bin))
        right_task = asyncio.create_task(self.pick_and_place_sequence("right_arm", right_block, right_bin))

        await asyncio.gather(left_task, right_task)
        
        self.get_logger().info("========================================")
        self.get_logger().info(" BIMANUAL CARTESIAN OPERATION COMPLETE")
        self.get_logger().info("========================================")

def main(args=None):
    rclpy.init(args=args)
    node = CartesianPickAndPlace()
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
