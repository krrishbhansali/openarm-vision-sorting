#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    workspace_root = LaunchConfiguration("workspace_root")

    moveit_demo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("openarm_bimanual_moveit_config"),
                "launch",
                "demo.launch.py",
            )
        )
    )

    mujoco_bridge = ExecuteProcess(
        cmd=[
            "python3",
            PathJoinSubstitution([workspace_root, "mujoco_ros2_bridge.py"]),
        ],
        name="mujoco_ros2_bridge",
        output="screen",
    )

    block_detector = ExecuteProcess(
        cmd=[
            "python3",
            PathJoinSubstitution([workspace_root, "block_detector.py"]),
        ],
        name="block_detector",
        output="screen",
    )

    delayed_scene_builder = TimerAction(
        period=4.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "python3",
                    PathJoinSubstitution([workspace_root, "scene_builder.py"]),
                ],
                name="scene_builder",
                output="screen",
            )
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "workspace_root",
                default_value=os.path.expanduser("~/ros2_ws"),
                description="Path to the ROS 2 workspace containing the custom Python scripts.",
            ),
            moveit_demo_launch,
            mujoco_bridge,
            block_detector,
            delayed_scene_builder,
        ]
    )
