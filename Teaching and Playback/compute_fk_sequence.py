#!/usr/bin/env python3

import json
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionFK
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState

class ComputeFKSequenceNode(Node):
    def __init__(self):
        super().__init__('compute_fk_sequence')
        self.fk_client = self.create_client(GetPositionFK, '/compute_fk')

        while not self.fk_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('/compute_fk service not available, waiting again...')

    def compute_pose(self, fk_link_name, joint_names, joint_positions):
        req = GetPositionFK.Request()
        req.header.frame_id = "world"
        req.fk_link_names = [fk_link_name]
        
        rs = RobotState()
        js = JointState()
        js.name = joint_names
        js.position = joint_positions
        rs.joint_state = js
        req.robot_state = rs
        
        future = self.fk_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            resp = future.result()
            if resp.error_code.val == 1: # SUCCESS
                return resp.pose_stamped[0].pose
            else:
                self.get_logger().error(f"FK calculation failed with error code {resp.error_code.val}")
                return None
        else:
            self.get_logger().error("Service call failed")
            return None

def main(args=None):
    rclpy.init(args=args)
    node = ComputeFKSequenceNode()

    # Load sequence joints
    filename = "sequence_joints.json"
    try:
        with open(filename, 'r') as f:
            data = json.load(f)
    except FileNotFoundError:
        node.get_logger().error(f"Could not find '{filename}'. Did you run teach_sequence.py first?")
        return

    output_sequence = {"left_arm": [], "right_arm": []}

    # Process Left Arm
    if "left_arm" in data:
        node.get_logger().info("Computing Cartesian Poses for Left Arm...")
        for i, step in enumerate(data["left_arm"]):
            pose = node.compute_pose(
                fk_link_name="openarm_left_ee_base_link",
                joint_names=step["arm_joint_names"],
                joint_positions=step["arm_positions"]
            )
            if pose:
                output_sequence["left_arm"].append({
                    "step_name": step["step_name"],
                    "position": {"x": pose.position.x, "y": pose.position.y, "z": pose.position.z},
                    "orientation": {"x": pose.orientation.x, "y": pose.orientation.y, "z": pose.orientation.z, "w": pose.orientation.w},
                    "gripper_rad": step["gripper_rad"],
                    "gripper_percent": step["gripper_percent"]
                })

    # Process Right Arm
    if "right_arm" in data:
        node.get_logger().info("Computing Cartesian Poses for Right Arm...")
        for i, step in enumerate(data["right_arm"]):
            pose = node.compute_pose(
                fk_link_name="openarm_right_ee_base_link",
                joint_names=step["arm_joint_names"],
                joint_positions=step["arm_positions"]
            )
            if pose:
                output_sequence["right_arm"].append({
                    "step_name": step["step_name"],
                    "position": {"x": pose.position.x, "y": pose.position.y, "z": pose.position.z},
                    "orientation": {"x": pose.orientation.x, "y": pose.orientation.y, "z": pose.orientation.z, "w": pose.orientation.w},
                    "gripper_rad": step["gripper_rad"],
                    "gripper_percent": step["gripper_percent"]
                })

    # Save outputs
    out_filename = "sequence_poses.json"
    with open(out_filename, 'w') as f:
        json.dump(output_sequence, f, indent=4)
        
    print("\n==================================================")
    print(f" SUCCESS! Calculated Cartesian Poses for all 14 steps.")
    print(f" Saved to '{out_filename}'.")
    print("==================================================")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
