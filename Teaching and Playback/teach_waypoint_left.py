#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import threading
import json
import sys

class TeachWaypointsNode(Node):
    def __init__(self):
        super().__init__('teach_waypoints')
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        self.latest_arm_joints = {}
        self.trajectory = []
        
        # We record the arm joints AND the gripper joint
        self.target_joints = [
            'openarm_left_joint1',
            'openarm_left_joint2',
            'openarm_left_joint3',
            'openarm_left_joint4',
            'openarm_left_joint5',
            'openarm_left_joint6',
            'openarm_left_joint7',
            'openarm_left_finger_joint1'
        ]

    def joint_state_callback(self, msg):
        for name, position in zip(msg.name, msg.position):
            if name in self.target_joints:
                self.latest_arm_joints[name] = position

    def get_current_waypoint(self):
        if len(self.latest_arm_joints) != len(self.target_joints):
            return None
        # Return an ordered list of joint values
        return [self.latest_arm_joints[j] for j in self.target_joints]

def input_thread_func(node):
    print("==================================================")
    print(" TEACH MODE ACTIVE")
    print("==================================================")
    print("Instructions:")
    print("  1. In RViz2, drag the left_arm OR change the left_gripper.")
    print("  2. Click 'Plan & Execute' in RViz2 so the robot moves.")
    print("  3. Press [ENTER] here to save the arm AND gripper position together.")
    print("  4. Type 'save' when you are done to export the sequence.")
    print("==================================================")

    while rclpy.ok():
        try:
            cmd = input("\n[Teach] Press Enter to record pose, or type 'save': ").strip().lower()
            
            if cmd == 'save':
                if not node.trajectory:
                    print("[Teach] Sequence is empty! Exiting without saving.")
                    break
                
                filename = "left_arm_trajectory.json"
                with open(filename, 'w') as f:
                    json.dump(node.trajectory, f, indent=4)
                print(f"[Teach] SAVED {len(node.trajectory)} steps to '{filename}'. You can now run the playback script!")
                break
                
            elif cmd == '':
                waypoint = node.get_current_waypoint()
                if waypoint is None:
                    print("[Teach] ERROR: Haven't received joint states for all joints yet. Make sure the robot is running.")
                else:
                    node.trajectory.append({"type": "combined", "joints": waypoint})
                    print(f"[Teach] Recorded ARM + GRIPPER POSE. (Step {len(node.trajectory)})")
            
            else:
                print(f"[Teach] Unknown command: '{cmd}'")
                
        except EOFError:
            break

    # Stop ROS spinning when input loop exits
    rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = TeachWaypointsNode()

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
