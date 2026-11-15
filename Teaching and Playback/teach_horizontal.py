#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import threading
import json
import sys

class TeachHorizontalNode(Node):
    def __init__(self):
        super().__init__('teach_horizontal')
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        self.latest_joints = {}
        
        self.left_joints = [
            'openarm_left_joint1', 'openarm_left_joint2', 'openarm_left_joint3',
            'openarm_left_joint4', 'openarm_left_joint5', 'openarm_left_joint6',
            'openarm_left_joint7'
        ]
        
        self.right_joints = [
            'openarm_right_joint1', 'openarm_right_joint2', 'openarm_right_joint3',
            'openarm_right_joint4', 'openarm_right_joint5', 'openarm_right_joint6',
            'openarm_right_joint7'
        ]

    def joint_state_callback(self, msg):
        for name, position in zip(msg.name, msg.position):
            self.latest_joints[name] = position

    def get_joints(self, joint_list):
        for j in joint_list:
            if j not in self.latest_joints:
                return None
        return [self.latest_joints[j] for j in joint_list]

def input_thread_func(node):
    print("==================================================")
    print(" TEACH HORIZONTAL REFERENCE POSES")
    print("==================================================")
    
    saved_data = {}

    while rclpy.ok():
        try:
            # 1. Left Arm
            print("\n--- LEFT ARM ---")
            print("1. In RViz2, drag the LEFT ARM to a horizontal grasp pose (approaching from +X side).")
            print("2. Click 'Plan & Execute' in RViz2.")
            input("3. Press [ENTER] here when the left arm is in position: ")
            
            left_pos = node.get_joints(node.left_joints)
            if left_pos is None:
                print("ERROR: Did not receive joint states for left arm yet! Please try again.")
                continue
                
            saved_data["left_arm"] = {
                "joint_names": node.left_joints,
                "positions": left_pos
            }
            print(f"-> Left arm pose saved! ({len(left_pos)} joints)")
            
            # 2. Right Arm
            print("\n--- RIGHT ARM ---")
            print("1. In RViz2, drag the RIGHT ARM to a horizontal grasp pose (approaching from +X side).")
            print("2. Click 'Plan & Execute' in RViz2.")
            input("3. Press [ENTER] here when the right arm is in position: ")
            
            right_pos = node.get_joints(node.right_joints)
            if right_pos is None:
                print("ERROR: Did not receive joint states for right arm yet! Please try again.")
                continue
                
            saved_data["right_arm"] = {
                "joint_names": node.right_joints,
                "positions": right_pos
            }
            print(f"-> Right arm pose saved! ({len(right_pos)} joints)")
            
            # 3. Save to JSON
            filename = "horizontal_joints.json"
            with open(filename, 'w') as f:
                json.dump(saved_data, f, indent=4)
                
            print("\n==================================================")
            print(f" SUCCESS! Saved both reference poses to '{filename}'.")
            print(" You can now run `./compute_fk.py`")
            print("==================================================")
            break
            
        except EOFError:
            break

    # Stop ROS spinning when input loop exits
    rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = TeachHorizontalNode()

    # Start the input thread
    input_thread = threading.Thread(target=input_thread_func, args=(node,), daemon=True)
    input_thread.start()

    # Spin the node to process ROS callbacks (joint states)
    try:
        rclpy.spin(node)
    except Exception:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
