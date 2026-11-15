#!/usr/bin/env python3
"""
Continuous ceiling-camera block target publisher.

The node detects the nearest movable red and blue blocks from
/camera/ceiling/image_raw and continuously publishes:

  /left_arm_target   geometry_msgs/Point  -> nearest red block as (X, Y, 0.0)
  /right_arm_target  geometry_msgs/Point  -> nearest blue block as (X, Y, 0.0)

It also publishes /camera/ceiling/annotated for debugging in rqt_image_view.
"""

import math
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from rclpy.node import Node
from sensor_msgs.msg import Image


class CameraModel:
    def __init__(self, xml_path: str):
        self.fx = 0.0
        self.fy = 0.0
        self.cx = 0.0
        self.cy = 0.0
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_z = 0.0
        
        self._load_from_xml(xml_path)

    def _load_from_xml(self, xml_path: str):
        tree = ET.parse(xml_path)
        root = tree.getroot()
        for cam in root.iter('camera'):
            if cam.get('name') == 'camera_ceiling':
                # Example: pos="0.245 0 0.6"
                pos = cam.get('pos').split()
                self.pos_x = float(pos[0])
                self.pos_y = float(pos[1])
                self.pos_z = float(pos[2])
                
                # Example: resolution="960 600"
                res = cam.get('resolution').split()
                width = float(res[0])
                height = float(res[1])
                
                # Example: fovy="66"
                self.fovy_rad = math.radians(float(cam.get('fovy')))
                return
        raise ValueError("camera_ceiling not found in XML")

    def update_resolution(self, width: int, height: int):
        """Update intrinsics based on the true arriving image resolution."""
        if self.cx == width / 2.0 and self.cy == height / 2.0:
            return  # Already updated
            
        self.cx = width / 2.0
        self.cy = height / 2.0
        self.fy = (height / 2.0) / math.tan(self.fovy_rad / 2.0)
        self.fx = self.fy  # square pixels

    def pixel_to_world(self, u: float, v: float, object_z: float = 0.0) -> Tuple[float, float]:
        """
        Shoots a ray from the pinhole out into the world.
        u: pixel column (X)
        v: pixel row (Y)
        object_z: height of the table/block plane
        """
        # Distance from camera lens to the object plane
        depth = self.pos_z - object_z
        
        # Ray in pure Camera Frame
        y_cam = (v - self.cy) / self.fy
        x_cam = (u - self.cx) / self.fx
        
        # Apply Extrinsic Rotation: Camera looks down (-Z). Image Top is World +X, Image Right is World -Y.
        world_x = self.pos_x - (depth * y_cam)
        world_y = self.pos_y - (depth * x_cam)
        
        return world_x, world_y


@dataclass
class BlockDetection:
    color_name: str
    pixel_u: int
    pixel_v: int
    world_x: float
    world_y: float
    bbox: Tuple[int, int, int, int]
    area: float


class BlockTargetPublisher(Node):
    def __init__(self):
        super().__init__("block_target_publisher")
        self.bridge = CvBridge()
        self.latest_image: Optional[np.ndarray] = None

        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("min_block_area_px", 80.0)
        self.declare_parameter("max_block_area_px", 2500.0)
        self.declare_parameter("min_block_width_px", 8)
        self.declare_parameter("max_block_width_px", 70)
        self.declare_parameter("min_block_height_px", 8)
        self.declare_parameter("max_block_height_px", 70)
        self.declare_parameter("max_aspect_ratio", 2.2)
        self.declare_parameter("min_solidity", 0.55)
        self.declare_parameter("log_targets", True)
        self.declare_parameter(
            "xml_path",
            str(
                Path(__file__).resolve().parent
                / "src/openarm_mujoco/v2/openarm_bimanual.xml"
            ),
        )

        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.min_block_area_px = float(self.get_parameter("min_block_area_px").value)
        self.max_block_area_px = float(self.get_parameter("max_block_area_px").value)
        self.min_block_width_px = int(self.get_parameter("min_block_width_px").value)
        self.max_block_width_px = int(self.get_parameter("max_block_width_px").value)
        self.min_block_height_px = int(self.get_parameter("min_block_height_px").value)
        self.max_block_height_px = int(self.get_parameter("max_block_height_px").value)
        self.max_aspect_ratio = float(self.get_parameter("max_aspect_ratio").value)
        self.min_solidity = float(self.get_parameter("min_solidity").value)
        self.log_targets = bool(self.get_parameter("log_targets").value)
        self.last_log_time = 0.0
        
        xml_path = self.get_parameter("xml_path").value
        self.camera_model = CameraModel(xml_path)
        self.get_logger().info(f"Loaded Camera Model: Fx={self.camera_model.fx:.2f}, Z={self.camera_model.pos_z:.2f}")

        # Approximate arm bases in world coordinates. Used only to decide which
        # same-color contour is "nearest" when there are multiple blocks.
        self.left_arm_base_xy = (0.0, 0.031)
        self.right_arm_base_xy = (0.0, -0.031)

        self.color_ranges = {
            "red": [
                (np.array([0, 100, 60]), np.array([12, 255, 255])),
                (np.array([168, 100, 60]), np.array([180, 255, 255])),
            ],
            "blue": [
                (np.array([95, 80, 45]), np.array([145, 255, 255])),
            ],
        }
        self.draw_colors = {
            "red": (0, 0, 255),
            "blue": (255, 0, 0),
        }

        self.image_sub = self.create_subscription(
            Image,
            "/camera/ceiling/image_raw",
            self.image_callback,
            10,
        )
        self.left_target_pub = self.create_publisher(Point, "/left_arm_target", 10)
        self.right_target_pub = self.create_publisher(Point, "/right_arm_target", 10)
        self.annotated_pub = self.create_publisher(
            Image,
            "/camera/ceiling/annotated",
            10,
        )

        timer_period = 1.0 / max(self.publish_rate_hz, 1.0)
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info(
            "Block detector online: publishing red -> /left_arm_target, blue -> /right_arm_target"
        )

    def image_callback(self, msg: Image) -> None:
        self.camera_model.update_resolution(msg.width, msg.height)
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warn(f"Failed to convert camera frame: {exc}")

    def timer_callback(self) -> None:
        if self.latest_image is None:
            cv2.waitKey(1)
            return

        image = self.latest_image.copy()
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        red_blocks = self.find_color_blocks(hsv, "red")
        blue_blocks = self.find_color_blocks(hsv, "blue")

        red_target = self.nearest_block(red_blocks, self.left_arm_base_xy)
        blue_target = self.nearest_block(blue_blocks, self.right_arm_base_xy)

        if red_target is not None:
            self.publish_point(self.left_target_pub, red_target)
            self.draw_detection(image, red_target, "LEFT TARGET")

        if blue_target is not None:
            self.publish_point(self.right_target_pub, blue_target)
            self.draw_detection(image, blue_target, "RIGHT TARGET")

        for block in red_blocks + blue_blocks:
            if block is red_target or block is blue_target:
                continue
            self.draw_detection(image, block, "candidate", thickness=1)

        self.log_current_targets(red_target, blue_target)
        self.annotated_pub.publish(self.bridge.cv2_to_imgmsg(image, encoding="bgr8"))
        cv2.waitKey(1)

    def find_color_blocks(self, hsv: np.ndarray, color_name: str) -> List[BlockDetection]:
        mask = self.build_mask(hsv, color_name)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        blocks: List[BlockDetection] = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < self.min_block_area_px or area > self.max_block_area_px:
                continue

            x, y, w, h = cv2.boundingRect(contour)
            if (
                w < self.min_block_width_px
                or w > self.max_block_width_px
                or h < self.min_block_height_px
                or h > self.max_block_height_px
            ):
                continue

            aspect_ratio = max(w / max(h, 1), h / max(w, 1))
            if aspect_ratio > self.max_aspect_ratio:
                continue

            hull_area = cv2.contourArea(cv2.convexHull(contour))
            if hull_area <= 0.0 or (area / hull_area) < self.min_solidity:
                continue

            pixel_u = x + (w // 2)
            pixel_v = y + (h // 2)
            
            # The table is at Z=-0.3 with height 0.025 (top is -0.275).
            # The blocks have half-height 0.02, so their top surface is at Z = -0.235.
            world_x, world_y = self.camera_model.pixel_to_world(pixel_u, pixel_v, object_z=-0.235)

            blocks.append(
                BlockDetection(
                    color_name=color_name,
                    pixel_u=pixel_u,
                    pixel_v=pixel_v,
                    world_x=world_x,
                    world_y=world_y,
                    bbox=(x, y, w, h),
                    area=area,
                )
            )

        return blocks

    def build_mask(self, hsv: np.ndarray, color_name: str) -> np.ndarray:
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lower, upper in self.color_ranges[color_name]:
            mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lower, upper))

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        return mask

    def nearest_block(
        self,
        blocks: List[BlockDetection],
        arm_base_xy: Tuple[float, float],
    ) -> Optional[BlockDetection]:
        if not blocks:
            return None

        base_x, base_y = arm_base_xy
        return min(
            blocks,
            key=lambda block: math.hypot(block.world_x - base_x, block.world_y - base_y),
        )

    def publish_point(self, publisher, block: BlockDetection) -> None:
        msg = Point()
        msg.x = block.world_x
        msg.y = block.world_y
        msg.z = 0.0
        publisher.publish(msg)

    def draw_detection(
        self,
        image: np.ndarray,
        block: BlockDetection,
        prefix: str,
        thickness: int = 2,
    ) -> None:
        x, y, w, h = block.bbox
        color = self.draw_colors[block.color_name]
        cv2.rectangle(image, (x, y), (x + w, y + h), color, thickness)
        cv2.circle(image, (block.pixel_u, block.pixel_v), 4, color, -1)

        label = (
            f"{prefix} {block.color_name.upper()} "
            f"X={block.world_x:.3f}, Y={block.world_y:.3f}"
        )
        cv2.putText(
            image,
            label,
            (max(0, x - 20), max(18, y - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            color,
            2,
            cv2.LINE_AA,
        )

    def log_current_targets(
        self,
        red_target: Optional[BlockDetection],
        blue_target: Optional[BlockDetection],
    ) -> None:
        if not self.log_targets:
            return

        now = time.time()
        if now - self.last_log_time < 1.0:
            return

        red_text = "none"
        blue_text = "none"
        if red_target is not None:
            red_text = f"({red_target.world_x:.3f}, {red_target.world_y:.3f})"
        if blue_target is not None:
            blue_text = f"({blue_target.world_x:.3f}, {blue_target.world_y:.3f})"

        self.get_logger().info(
            f"targets: left/red={red_text}, right/blue={blue_text}"
        )
        self.last_log_time = now


def main(args=None):
    rclpy.init(args=args)
    node = BlockTargetPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
