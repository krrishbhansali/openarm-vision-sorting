#!/usr/bin/env python3

import json
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionFK
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState

class ComputeFKNode(Node):
    def __init__(self):
        super().__init__('compute_fk_node')
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
                # We only requested 1 link
                return resp.pose_stamped[0].pose
            else:
                self.get_logger().error(f"FK calculation failed with error code {resp.error_code.val}")
                return None
        else:
            self.get_logger().error("Service call failed")
            return None

def main(args=None):
    rclpy.init(args=args)
    node = ComputeFKNode()

    # Load joints
    filename = "horizontal_joints.json"
    try:
        with open(filename, 'r') as f:
            data = json.load(f)
    except FileNotFoundError:
        node.get_logger().error(f"Could not find '{filename}'. Did you run teach_horizontal.py first?")
        return

    output_poses = {}

    # Process Left Arm
    if "left_arm" in data:
        node.get_logger().info("Computing Cartesian Pose for Left Arm...")
        left_pose = node.compute_pose(
            fk_link_name="openarm_left_ee_base_link",
            joint_names=data["left_arm"]["joint_names"],
            joint_positions=data["left_arm"]["positions"]
        )
        if left_pose:
            output_poses["left_arm"] = {
                "position": {
                    "x": left_pose.position.x,
                    "y": left_pose.position.y,
                    "z": left_pose.position.z
                },
                "orientation": {
                    "x": left_pose.orientation.x,
                    "y": left_pose.orientation.y,
                    "z": left_pose.orientation.z,
                    "w": left_pose.orientation.w
                }
            }
            print(f"Left Arm Pose: {output_poses['left_arm']}")

    # Process Right Arm
    if "right_arm" in data:
        node.get_logger().info("Computing Cartesian Pose for Right Arm...")
        right_pose = node.compute_pose(
            fk_link_name="openarm_right_ee_base_link",
            joint_names=data["right_arm"]["joint_names"],
            joint_positions=data["right_arm"]["positions"]
        )
        if right_pose:
            output_poses["right_arm"] = {
                "position": {
                    "x": right_pose.position.x,
                    "y": right_pose.position.y,
                    "z": right_pose.position.z
                },
                "orientation": {
                    "x": right_pose.orientation.x,
                    "y": right_pose.orientation.y,
                    "z": right_pose.orientation.z,
                    "w": right_pose.orientation.w
                }
            }
            print(f"Right Arm Pose: {output_poses['right_arm']}")

    # Save outputs
    out_filename = "horizontal_poses.json"
    with open(out_filename, 'w') as f:
        json.dump(output_poses, f, indent=4)
        
    print("\n==================================================")
    print(f" SUCCESS! Calculated reference quaternions and saved to '{out_filename}'.")
    print(" You can now run `./bimanual_pick_and_place.py`")
    print("==================================================")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
