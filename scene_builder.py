#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from pathlib import Path

class SceneBuilder(Node):
    def __init__(self):
        super().__init__('scene_builder')
        # Connect to the MoveIt 2 Planning Scene
        self.publisher_ = self.create_publisher(CollisionObject, '/collision_object', 10)
        
        self.get_logger().info("Waiting for MoveIt 2 to connect...")
        while self.publisher_.get_subscription_count() == 0:
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info("MoveIt 2 Connected! Constructing physical bounds...")
        
        # Build the environment using the EXACT coordinates from your MuJoCo XML
        # Shifted -0.1m in X
        self.add_table()
        self.add_bin("bin_red_left", 0.3, 0.3, 0.425)
        self.add_bin("bin_blue_right", 0.3, -0.3, 0.425)

    def add_table(self):
        obj = CollisionObject()
        obj.id = "work_table"
        obj.header.frame_id = "world" # The base mathematical frame of your robot

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        # MuJoCo size was 0.2, 0.4, 0.025. MoveIt requires FULL size (x2)
        box.dimensions = [0.4, 0.8, 0.05]

        pose = Pose()
        # Shifted -0.1m in X
        pose.position.x = 0.3
        pose.position.y = 0.0
        pose.position.z = 0.4

        obj.primitives.append(box)
        obj.primitive_poses.append(pose)
        obj.operation = CollisionObject.ADD

        self.publisher_.publish(obj)
        self.get_logger().info("Virtual Table deployed.")

    def add_bin(self, name, x, y, z):
        obj = CollisionObject()
        obj.id = name
        obj.header.frame_id = "world"

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        # MuJoCo size was 0.1, 0.1, 0.01. MoveIt requires FULL size (x2)
        box.dimensions = [0.2, 0.2, 0.02]

        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z

        obj.primitives.append(box)
        obj.primitive_poses.append(pose)
        obj.operation = CollisionObject.ADD

        self.publisher_.publish(obj)
        self.get_logger().info(f"Virtual Bin '{name}' deployed.")

def main(args=None):
    rclpy.init(args=args)
    node = SceneBuilder()
    # Spin briefly to ensure the network has time to transmit the messages
    rclpy.spin_once(node, timeout_sec=1.0)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()









# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from moveit_msgs.msg import CollisionObject
# from shape_msgs.msg import SolidPrimitive
# from geometry_msgs.msg import Pose

# class SceneBuilder(Node):
#     def __init__(self):
#         super().__init__('scene_builder')
#         # Connect to the MoveIt 2 Planning Scene
#         self.publisher_ = self.create_publisher(CollisionObject, '/collision_object', 10)
        
#         self.get_logger().info("Waiting for MoveIt 2 to connect...")
#         while self.publisher_.get_subscription_count() == 0:
#             rclpy.spin_once(self, timeout_sec=0.1)

#         self.get_logger().info("MoveIt 2 Connected! Constructing physical bounds...")
        
#         # Build the environment using the EXACT coordinates from your MuJoCo XML
#         self.add_table()
#         self.add_bin("bin_red_left", 0.3, 0.3, 0.425)
#         self.add_bin("bin_blue_right", 0.3, -0.3, 0.425)

#     def add_table(self):
#         obj = CollisionObject()
#         obj.id = "work_table"
#         obj.header.frame_id = "world" # The base mathematical frame of your robot

#         box = SolidPrimitive()
#         box.type = SolidPrimitive.BOX
#         # MuJoCo size was 0.2, 0.4, 0.025. MoveIt requires FULL size (x2)
#         box.dimensions = [0.4, 0.8, 0.05]

#         pose = Pose()
#         pose.position.x = 0.3
#         pose.position.y = 0.0
#         pose.position.z = 0.4

#         obj.primitives.append(box)
#         obj.primitive_poses.append(pose)
#         obj.operation = CollisionObject.ADD

#         self.publisher_.publish(obj)
#         self.get_logger().info("Virtual Table deployed.")

#     def add_bin(self, name, x, y, z):
#         obj = CollisionObject()
#         obj.id = name
#         obj.header.frame_id = "world"

#         box = SolidPrimitive()
#         box.type = SolidPrimitive.BOX
#         # MuJoCo size was 0.1, 0.1, 0.01. MoveIt requires FULL size (x2)
#         box.dimensions = [0.2, 0.2, 0.02]

#         pose = Pose()
#         pose.position.x = x
#         pose.position.y = y
#         pose.position.z = z

#         obj.primitives.append(box)
#         obj.primitive_poses.append(pose)
#         obj.operation = CollisionObject.ADD

#         self.publisher_.publish(obj)
#         self.get_logger().info(f"Virtual Bin '{name}' deployed.")

# def main(args=None):
#     rclpy.init(args=args)
#     node = SceneBuilder()
#     # Spin briefly to ensure the network has time to transmit the messages
#     rclpy.spin_once(node, timeout_sec=1.0)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
