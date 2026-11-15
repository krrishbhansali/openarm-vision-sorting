#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import threading
import json

class TeachSequenceNode(Node):
    def __init__(self):
        super().__init__('teach_sequence')
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
        self.left_gripper = 'openarm_left_finger_joint1'
        
        self.right_joints = [
            'openarm_right_joint1', 'openarm_right_joint2', 'openarm_right_joint3',
            'openarm_right_joint4', 'openarm_right_joint5', 'openarm_right_joint6',
            'openarm_right_joint7'
        ]
        self.right_gripper = 'openarm_right_finger_joint1'

    def joint_state_callback(self, msg):
        for name, position in zip(msg.name, msg.position):
            self.latest_joints[name] = position

    def get_arm_joints(self, joint_list):
        for j in joint_list:
            if j not in self.latest_joints:
                return None
        return [self.latest_joints[j] for j in joint_list]

    def get_gripper_joint(self, gripper_name):
        return self.latest_joints.get(gripper_name, 0.0)

def input_thread_func(node):
    print("==================================================")
    print(" TEACH 7-STEP SEQUENCE FOR BOTH ARMS")
    print("==================================================")
    
    saved_data = {"left_arm": [], "right_arm": []}
    
    steps = [
        "1. Pre grasp",
        "2. Gripper open",
        "3. Go near the block",
        "4. Close gripper (the block is holded)",
        "5. Go above the bin",
        "6. Open gripper (drop the block)",
        "7. Home position"
    ]

    while rclpy.ok():
        try:
            # === LEFT ARM ===
            print("\n" + "="*40)
            print(" TEACHING LEFT ARM ")
            print("="*40)
            for i, step_name in enumerate(steps):
                print(f"\n[LEFT ARM] {step_name}")
                print(" -> Adjust the arm and gripper in RViz2, then click 'Plan & Execute'.")
                input(" -> Press [ENTER] here when the robot is in position: ")
                
                arm_pos = node.get_arm_joints(node.left_joints)
                gripper_pos = node.get_gripper_joint(node.left_gripper)
                
                if arm_pos is None:
                    print("ERROR: Did not receive joint states! Run again.")
                    return
                
                # Convert gripper rad to percentage (max close is ~0.7854)
                # 0.0 = 0% (open), 0.7854 = 100% (closed)
                gripper_percent = min(100.0, max(0.0, (gripper_pos / 0.7854) * 100.0))
                
                saved_data["left_arm"].append({
                    "step_name": step_name,
                    "arm_joint_names": node.left_joints,
                    "arm_positions": arm_pos,
                    "gripper_rad": gripper_pos,
                    "gripper_percent": round(gripper_percent, 1)
                })
                print(f" ✓ Saved! (Gripper at {gripper_percent:.1f}%)")

            # === RIGHT ARM ===
            print("\n" + "="*40)
            print(" TEACHING RIGHT ARM ")
            print("="*40)
            for i, step_name in enumerate(steps):
                print(f"\n[RIGHT ARM] {step_name}")
                print(" -> Adjust the arm and gripper in RViz2, then click 'Plan & Execute'.")
                input(" -> Press [ENTER] here when the robot is in position: ")
                
                arm_pos = node.get_arm_joints(node.right_joints)
                gripper_pos = node.get_gripper_joint(node.right_gripper)
                
                if arm_pos is None:
                    print("ERROR: Did not receive joint states! Run again.")
                    return
                
                gripper_percent = min(100.0, max(0.0, (gripper_pos / 0.7854) * 100.0))
                
                saved_data["right_arm"].append({
                    "step_name": step_name,
                    "arm_joint_names": node.right_joints,
                    "arm_positions": arm_pos,
                    "gripper_rad": gripper_pos,
                    "gripper_percent": round(gripper_percent, 1)
                })
                print(f" ✓ Saved! (Gripper at {gripper_percent:.1f}%)")
            
            # Save to JSON
            filename = "sequence_joints.json"
            with open(filename, 'w') as f:
                json.dump(saved_data, f, indent=4)
                
            print("\n==================================================")
            print(f" SUCCESS! Saved all 14 steps to '{filename}'.")
            print(" You can now run `./compute_fk_sequence.py`")
            print("==================================================")
            break
            
        except EOFError:
            break

    # Stop ROS spinning when input loop exits
    rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = TeachSequenceNode()

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
