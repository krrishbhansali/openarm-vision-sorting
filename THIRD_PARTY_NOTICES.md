# Third-party notices and project attribution

This project builds on OpenArm by Enactic, Inc., ROS 2, MoveIt 2, and MuJoCo.
The OpenArm robot design, descriptions, meshes, base configurations, and simulator
models are upstream work. This repository does not claim authorship of them.

## Project code and license

Project-authored code and documentation are provided under the Apache License,
Version 2.0; see [LICENSE](LICENSE). Third-party files retain their original
copyright notices and license terms. File-specific notices take precedence over
this general project statement.

The application work includes the ceiling-camera ROS bridge, color-based block
detection and planar coordinate mapping, tabletop sorting-scene integration,
teaching/playback experiments, and the autonomous pick-and-place orchestration.
These components use existing ROS 2 interfaces, MoveIt kinematics and motion
generation, and MuJoCo simulation; they do not implement those frameworks.

## Bundled OpenArm source snapshots

The directories below are source snapshots copied from the development workspace.
They include the local changes described below; the recorded commits identify
their upstream bases, rather than commits containing all project modifications.
Upstream Git histories are not bundled.

| Directory | Upstream repository | Base commit | Preserved license |
|---|---|---|---|
| `src/openarm_ros2/` | [enactic/openarm_ros2](https://github.com/enactic/openarm_ros2) | `01b9f3815222b490a593f01379d8e2106c8da6a0` | [Apache-2.0](src/openarm_ros2/LICENSE) |
| `src/openarm_description/` | [enactic/openarm_description](https://github.com/enactic/openarm_description) | `2a29691e2ff9f48f735cd6d765368c4d989087da` | [Apache-2.0](src/openarm_description/LICENSE.txt) |
| `src/openarm_mujoco/` | [enactic/openarm_mujoco](https://github.com/enactic/openarm_mujoco) | `5c6e1b4c71cbe27c6a4b58c7f198f080449e1b9c` | [Apache-2.0](src/openarm_mujoco/LICENSE) |

OpenArm upstream notices identify Enactic, Inc., including copyright years 2025
and 2026. Some individual files identify additional contributors. Those original
notices are retained in the bundled source and must be preserved with those files.

The machine-readable base-revision list is in
[docs/upstream_versions.txt](docs/upstream_versions.txt).

## Changes included in the snapshots

### `openarm_ros2`

Relative to its recorded upstream base, the snapshot contains changes to:

- `openarm_bimanual_moveit_config/config/openarm_v2.0/kinematics.yaml`
- `openarm_bimanual_moveit_config/config/openarm_v2.0/moveit.rviz`
- `openarm_bimanual_moveit_config/config/openarm_v2.0/moveit_controllers.yaml`
- `openarm_bimanual_moveit_config/config/openarm_v2.0/openarm_bimanual.srdf`
- `openarm_bimanual_moveit_config/config/openarm_v2.0/ros2_controllers.yaml`

These adapt kinematics timeouts, RViz configuration, controller mappings, and
gripper/end-effector configuration. It also includes two project additions:

- `openarm_bimanual_moveit_config/launch/master_bimanual.launch.py`, which starts
  the MoveIt demo and custom simulation, detection, and scene-building processes.
- `openarm_hardware/COLCON_IGNORE`, which excludes the hardware package from
  automatic colcon discovery in this simulation workspace.

### `openarm_description`

No tracked modifications to the upstream base were reported at snapshot review.
The copied directory includes the robot description and associated assets.

### `openarm_mujoco`

- `v2/openarm_bimanual.xml` adds the sorting table, colored destination regions,
  colored blocks, and the ceiling camera to the upstream robot scene.
- `setup.py` contains local Python packaging changes.

## External runtime dependencies

ROS 2, MoveIt 2, ros2_control, MuJoCo, OpenCV, NumPy, and their dependencies are
external projects governed by their respective licenses. The project-level
license does not replace their licenses or imply endorsement by their authors.

## Snapshot scope

Earlier teaching/playback experiments may retain paths and assumptions from the
original learning workspace. The supported autonomous workflow and its tested
environment should be identified in the project README. The three active resource
paths in the application were made relative to their script locations during
GitHub preparation on 15 November 2026.
