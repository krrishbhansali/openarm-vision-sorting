#!/usr/bin/env python3

import sys
import json
import asyncio
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class PlaybackBimanual(Node):
    def __init__(self):
        super().__init__('playback_bimanual')
        
        # Action clients Left
        self.move_action_client_left = ActionClient(self, MoveGroup, '/move_action')
        self.gripper_client_left = ActionClient(self, FollowJointTrajectory, '/left_gripper_controller/follow_joint_trajectory')

        # Action clients Right
        self.move_action_client_right = ActionClient(self, MoveGroup, '/move_action')
        self.gripper_client_right = ActionClient(self, FollowJointTrajectory, '/right_gripper_controller/follow_joint_trajectory')

        # Joint names
        self.left_arm_joints = [
            'openarm_left_joint1', 'openarm_left_joint2', 'openarm_left_joint3',
            'openarm_left_joint4', 'openarm_left_joint5', 'openarm_left_joint6',
            'openarm_left_joint7'
        ]
        self.left_gripper_joint = 'openarm_left_finger_joint1'

        self.right_arm_joints = [
            'openarm_right_joint1', 'openarm_right_joint2', 'openarm_right_joint3',
            'openarm_right_joint4', 'openarm_right_joint5', 'openarm_right_joint6',
            'openarm_right_joint7'
        ]
        self.right_gripper_joint = 'openarm_right_finger_joint1'
        
        # Gripper definitions
        self.GRIPPER_OPEN_RAD = 0.0
        self.GRIPPER_CLOSE_RAD = 0.78  # Max close is 0.7854

    async def wait_for_servers(self):
        self.get_logger().info("Waiting for MoveGroup action server...")
        self.move_action_client_left.wait_for_server()
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

    async def move_arm_to_joints(self, arm_side, joint_values, step_idx):
        self.get_logger().info(f"[Step {step_idx}] [{arm_side.upper()}] Moving arm to recorded joints...")
        
        goal_msg = MoveGroup.Goal()
        req = MotionPlanRequest()
        req.workspace_parameters.header.frame_id = "world"
        
        if arm_side == "left":
            req.group_name = "left_arm"
            target_joints = self.left_arm_joints
            client = self.move_action_client_left
        else:
            req.group_name = "right_arm"
            target_joints = self.right_arm_joints
            client = self.move_action_client_right

        req.num_planning_attempts = 5
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = 0.35
        req.max_acceleration_scaling_factor = 0.35

        constraints = Constraints()
        for i, j_name in enumerate(target_joints):
            jc = JointConstraint()
            jc.joint_name = j_name
            jc.position = joint_values[i]
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        
        req.goal_constraints.append(constraints)
        goal_msg.request = req

        send_goal_future = client.send_goal_async(goal_msg)
        goal_handle = await self.await_ros_future(send_goal_future)
        if not goal_handle.accepted:
            self.get_logger().error(f"[Step {step_idx}] [{arm_side.upper()}] MoveGroup rejected the goal!")
            return False

        result_future = goal_handle.get_result_async()
        result = await self.await_ros_future(result_future)
        
        error_code = result.result.error_code.val
        if error_code == 1:
            return True
        else:
            self.get_logger().error(f"[Step {step_idx}] [{arm_side.upper()}] Arm movement failed with error_code: {error_code}")
            return False

    async def move_gripper_to_val(self, arm_side, target_rad, step_idx):
        self.get_logger().info(f"[Step {step_idx}] [{arm_side.upper()}] Commanding gripper ({target_rad} rad)...")
        
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
            self.get_logger().error(f"[Step {step_idx}] [{arm_side.upper()}] Gripper rejected the goal!")
            return False
            
        result_future = goal_handle.get_result_async()
        result = await self.await_ros_future(result_future)
        
        error_code = result.result.error_code
        if error_code == 0:
            return True
        else:
            self.get_logger().error(f"[Step {step_idx}] [{arm_side.upper()}] Gripper failed with error_code: {error_code}")
            return False

    async def execute_combined(self, arm_side, joints, step_idx):
        # First 7 are arm, 8th is gripper
        arm_joints = joints[:7]
        gripper_val = joints[7]
        
        # We run both goals concurrently using asyncio.gather
        arm_task = asyncio.create_task(self.move_arm_to_joints(arm_side, arm_joints, step_idx))
        gripper_task = asyncio.create_task(self.move_gripper_to_val(arm_side, gripper_val, step_idx))
        
        results = await asyncio.gather(arm_task, gripper_task)
        return all(results)

    async def dispatch_step(self, arm_side, step, step_idx):
        if step["type"] == "combined":
            return await self.execute_combined(arm_side, step["joints"], step_idx)
        elif step["type"] == "arm":
            return await self.move_arm_to_joints(arm_side, step["joints"], step_idx)
        elif step["type"] == "gripper":
            target_rad = self.GRIPPER_OPEN_RAD if step["action"] == "open" else self.GRIPPER_CLOSE_RAD
            return await self.move_gripper_to_val(arm_side, target_rad, step_idx)
        else:
            self.get_logger().warning(f"[Step {step_idx}] [{arm_side.upper()}] Unknown step type: {step['type']}")
            return True

    async def execute_trajectories(self, left_file, right_file):
        # Load left trajectory
        try:
            with open(left_file, 'r') as f:
                left_traj = json.load(f)
        except Exception:
            self.get_logger().warning(f"Could not load '{left_file}'. Left arm will do nothing.")
            left_traj = []

        # Load right trajectory
        try:
            with open(right_file, 'r') as f:
                right_traj = json.load(f)
        except Exception:
            self.get_logger().warning(f"Could not load '{right_file}'. Right arm will do nothing.")
            right_traj = []

        max_steps = max(len(left_traj), len(right_traj))
        
        if max_steps == 0:
            self.get_logger().error("Both trajectories are empty or missing! Aborting.")
            return

        self.get_logger().info(f"Loaded Left ({len(left_traj)} steps) and Right ({len(right_traj)} steps).")
        self.get_logger().info("Beginning BIMANUAL playback...")
        
        for i in range(max_steps):
            step_idx = i + 1
            
            tasks = []
            
            # Queue left arm action
            if i < len(left_traj):
                tasks.append(asyncio.create_task(self.dispatch_step("left", left_traj[i], step_idx)))
                
            # Queue right arm action
            if i < len(right_traj):
                tasks.append(asyncio.create_task(self.dispatch_step("right", right_traj[i], step_idx)))
                
            # Execute both arms concurrently for this step!
            results = await asyncio.gather(*tasks)
            
            if not all(results):
                self.get_logger().error("PLAYBACK ABORTED due to a failure in one of the arms.")
                return
            
            # Small buffer to ensure physics settle
            await asyncio.sleep(0.5)
                
        self.get_logger().info("================================================")
        self.get_logger().info(" BIMANUAL PLAYBACK COMPLETE!")
        self.get_logger().info("================================================")


def main(args=None):
    rclpy.init(args=args)
    
    left_file = "left_arm_trajectory.json"
    right_file = "right_arm_trajectory.json"
        
    node = PlaybackBimanual()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    
    async def run_playback():
        await node.wait_for_servers()
        await node.execute_trajectories(left_file, right_file)
        rclpy.shutdown()
        
    # Run the async loop
    loop = asyncio.get_event_loop()
    try:
        loop.run_until_complete(asyncio.gather(
            run_playback(),
            loop.run_in_executor(None, executor.spin)
        ))
    except (KeyboardInterrupt, RuntimeError):
        pass

if __name__ == '__main__':
    main()
