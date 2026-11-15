#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Image
from cv_bridge import CvBridge
import mujoco
import mujoco.viewer
import threading
import time
from pathlib import Path

class MujocoRosBridge(Node):
    def __init__(self, m, d):
        super().__init__('mujoco_ros_bridge')
        self.m = m
        self.d = d
        self.bridge = CvBridge()
        self.pub_ceiling = self.create_publisher(Image, '/camera/ceiling/image_raw', 10)
        self.sub_joints = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        self.get_logger().info("ROS 2 Network Node Online: Listening for MoveIt Trajectories.")

    def joint_callback(self, msg):
        # Inject ROS 2 trajectory angles into MuJoCo motor controllers
        for i, name in enumerate(msg.name):
            jnt_id = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_JOINT, name)
            if jnt_id != -1:
                for act_id in range(self.m.nu):
                    if self.m.actuator_trnid[act_id, 0] == jnt_id:
                        self.d.ctrl[act_id] = msg.position[i]
                        break

def main(args=None):
    rclpy.init(args=args)
    
    # Load the scene bundled with this project.
    xml_path = str(
        Path(__file__).resolve().parent
        / "src/openarm_mujoco/v2/openarm_bimanual.xml"
    )
    m = mujoco.MjModel.from_xml_path(xml_path)
    d = mujoco.MjData(m)
    
    # 1. Start the ROS 2 Network in the background
    bridge_node = MujocoRosBridge(m, d)
    executor_thread = threading.Thread(target=rclpy.spin, args=(bridge_node,), daemon=True)
    executor_thread.start()
    
    # 2. Initialize the Graphics Renderer strictly in the MAIN thread
    width = min(960, m.vis.global_.offwidth)
    height = min(600, m.vis.global_.offheight)
    renderer = mujoco.Renderer(m, height, width)
    
    cam_fps = 30
    last_cam_time = time.time()
    
    # 3. Launch Physics Viewer (Main Thread)
    viewer = mujoco.viewer.launch_passive(m, d)
    print("MuJoCo Physics Engine Online. Gravity Active.")
    
    while viewer.is_running():
        step_start = time.time()
        
        # Step Physics
        with viewer.lock():
            mujoco.mj_step(m, d)
            
        # Render and Publish Camera (Main Thread Safe)
        if time.time() - last_cam_time > (1.0 / cam_fps):
            try:
                renderer.update_scene(d, camera="camera_ceiling")
                pixels = renderer.render()
                img_msg = bridge_node.bridge.cv2_to_imgmsg(pixels, encoding="rgb8")
                bridge_node.pub_ceiling.publish(img_msg)
                last_cam_time = time.time()
            except Exception:
                pass
                
        # Sync the GUI
        viewer.sync()
        
        # Keep real-time pace
        elapsed = time.time() - step_start
        if elapsed < m.opt.timestep:
            time.sleep(m.opt.timestep - elapsed)
            
    rclpy.shutdown()

if __name__ == '__main__':
    main()
