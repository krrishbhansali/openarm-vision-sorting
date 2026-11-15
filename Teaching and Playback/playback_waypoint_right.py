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

class PlaybackWaypoints(Node):
    def __init__(self):
        super().__init__('playback_waypoints')
        
        # Action clients
        self.move_action_client = ActionClient(self, MoveGroup, '/move_action')
        self.gripper_client = ActionClient(self, FollowJointTrajectory, '/right_gripper_controller/follow_joint_trajectory')

        # Joint names
        self.arm_joints = [
            'openarm_right_joint1', 'openarm_right_joint2', 'openarm_right_joint3',
            'openarm_right_joint4', 'openarm_right_joint5', 'openarm_right_joint6',
            'openarm_right_joint7'
        ]
        self.gripper_joint = 'openarm_right_finger_joint1'
        
        # Gripper definitions (matches bimanual_script_right.py)
        self.GRIPPER_OPEN_RAD = 0.0
        self.GRIPPER_CLOSE_RAD = 0.78  # Max close is 0.7854

    async def wait_for_servers(self):
        self.get_logger().info("Waiting for MoveGroup action server...")
        self.move_action_client.wait_for_server()
        self.get_logger().info("Waiting for right gripper action server...")
        self.gripper_client.wait_for_server()
        self.get_logger().info("All servers online!")

    async def await_ros_future(self, future):
        while rclpy.ok() and not future.done():
            await asyncio.sleep(0.02)
        if not future.done():
            raise RuntimeError("ROS future did not complete before shutdown")
        return future.result()

    async def move_arm_to_joints(self, joint_values, step_idx):
        self.get_logger().info(f"[Step {step_idx}] Moving arm to recorded joints...")
        
        goal_msg = MoveGroup.Goal()
        req = MotionPlanRequest()
        req.workspace_parameters.header.frame_id = "world"
        req.group_name = "right_arm"
        req.num_planning_attempts = 5
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = 0.35
        req.max_acceleration_scaling_factor = 0.35

        constraints = Constraints()
        for i, j_name in enumerate(self.arm_joints):
            jc = JointConstraint()
            jc.joint_name = j_name
            jc.position = joint_values[i]
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        
        req.goal_constraints.append(constraints)
        goal_msg.request = req

        send_goal_future = self.move_action_client.send_goal_async(goal_msg)
        goal_handle = await self.await_ros_future(send_goal_future)
        if not goal_handle.accepted:
            self.get_logger().error(f"[Step {step_idx}] MoveGroup rejected the goal!")
            return False

        result_future = goal_handle.get_result_async()
        result = await self.await_ros_future(result_future)
        
        error_code = result.result.error_code.val
        if error_code == 1:
            self.get_logger().info(f"[Step {step_idx}] Arm movement complete.")
            return True
        else:
            self.get_logger().error(f"[Step {step_idx}] Arm movement failed with error_code: {error_code}")
            return False

    async def move_gripper(self, action, step_idx):
        target_rad = self.GRIPPER_OPEN_RAD if action == "open" else self.GRIPPER_CLOSE_RAD
        self.get_logger().info(f"[Step {step_idx}] Commanding gripper to {action.upper()} ({target_rad} rad)...")
        
        goal_msg = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = [self.gripper_joint]
        
        point = JointTrajectoryPoint()
        point.positions = [target_rad]
        point.time_from_start.sec = 2
        point.time_from_start.nanosec = 0
        traj.points.append(point)
        
        goal_msg.trajectory = traj
        
        send_goal_future = self.gripper_client.send_goal_async(goal_msg)
        goal_handle = await self.await_ros_future(send_goal_future)
        if not goal_handle.accepted:
            self.get_logger().error(f"[Step {step_idx}] Gripper rejected the goal!")
            return False
            
        result_future = goal_handle.get_result_async()
        result = await self.await_ros_future(result_future)
        
        error_code = result.result.error_code
        if error_code == 0:
            self.get_logger().info(f"[Step {step_idx}] Gripper action complete.")
            # Small delay to ensure physics settle
            await asyncio.sleep(1.0)
            return True
        else:
            self.get_logger().error(f"[Step {step_idx}] Gripper failed with error_code: {error_code}")
            return False

    async def execute_combined(self, joints, step_idx):
        # First 7 are arm, 8th is gripper
        arm_joints = joints[:7]
        gripper_val = joints[7]
        
        self.get_logger().info(f"[Step {step_idx}] Executing combined ARM and GRIPPER motion...")
        
        # We run both goals concurrently using asyncio.gather
        arm_task = asyncio.create_task(self.move_arm_to_joints(arm_joints, step_idx))
        gripper_task = asyncio.create_task(self.move_gripper_to_val(gripper_val, step_idx))
        
        results = await asyncio.gather(arm_task, gripper_task)
        return all(results)

    async def move_gripper_to_val(self, target_rad, step_idx):
        goal_msg = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = [self.gripper_joint]
        
        point = JointTrajectoryPoint()
        point.positions = [target_rad]
        point.time_from_start.sec = 2
        point.time_from_start.nanosec = 0
        traj.points.append(point)
        
        goal_msg.trajectory = traj
        
        send_goal_future = self.gripper_client.send_goal_async(goal_msg)
        goal_handle = await self.await_ros_future(send_goal_future)
        if not goal_handle.accepted:
            self.get_logger().error(f"[Step {step_idx}] Gripper rejected the goal!")
            return False
            
        result_future = goal_handle.get_result_async()
        result = await self.await_ros_future(result_future)
        
        error_code = result.result.error_code
        if error_code == 0:
            return True
        else:
            self.get_logger().error(f"[Step {step_idx}] Gripper failed with error_code: {error_code}")
            return False

    async def execute_trajectory(self, filepath):
        try:
            with open(filepath, 'r') as f:
                trajectory = json.load(f)
        except Exception as e:
            self.get_logger().error(f"Failed to load trajectory file '{filepath}': {e}")
            return

        self.get_logger().info(f"Loaded {len(trajectory)} steps from '{filepath}'. Beginning playback...")
        
        for i, step in enumerate(trajectory):
            step_idx = i + 1
            if step["type"] == "combined":
                success = await self.execute_combined(step["joints"], step_idx)
            elif step["type"] == "arm":
                success = await self.move_arm_to_joints(step["joints"], step_idx)
            elif step["type"] == "gripper":
                success = await self.move_gripper(step["action"], step_idx)
            else:
                self.get_logger().warning(f"[Step {step_idx}] Unknown step type: {step['type']}")
                continue
                
            if not success:
                self.get_logger().error("PLAYBACK ABORTED due to failure.")
                return
                
        self.get_logger().info("================================================")
        self.get_logger().info(" PLAYBACK COMPLETE! Successfully executed all steps.")
        self.get_logger().info("================================================")


def main(args=None):
    rclpy.init(args=args)
    
    if len(sys.argv) > 1:
        filepath = sys.argv[1]
    else:
        filepath = "right_arm_trajectory.json"
        
    node = PlaybackWaypoints()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    async def run_playback():
        await node.wait_for_servers()
        await node.execute_trajectory(filepath)
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
