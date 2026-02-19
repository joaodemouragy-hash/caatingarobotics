# caatingarobotics

experimental fork for jazzy/ gazebo harmonic

ROS 2 workspace for precision agriculture robotics, combining simulation, navigation,
computer vision inference, and field traceability analytics.

## Project Pitch

This project addresses a practical agriculture challenge: inspect large crop areas,
detect infestation signals, and support targeted intervention with fewer inputs.

The solution integrates:

- a ROS 2 simulation stack for robot operation and mission flow,
- a YOLO-based vision pipeline for on-board detection,
- analytics and traceability outputs to support agronomic decisions.

## Architecture Overview

The workspace is organized in two main ROS 2 packages:

- `agro_robot_sim`: robot model, Gazebo worlds, SLAM/navigation launch flows, mission scripts.
- `caatinga_vision`: camera source, YOLO inference, infestation analytics, photo capture pipeline.

High-level data flow:

1. Sensor/camera data is published.
2. Vision nodes run inference and publish detections.
3. Analytics nodes aggregate detections and generate status/recommendations.
4. Navigation and mission layers consume maps/localization for route execution.

## Tech Stack

- ROS 2 jazzy
- Gazebo harmonic + Nav2 + SLAM Toolbox
- YOLO (Ultralytics), OpenCV, `cv_bridge`
- Python (`rclpy`) and CMake (`ament_cmake`)
- PyQt5 operational panel tooling

## Repository Structure

```text
.
├── datasets/
│   └── agro_v1/
│       └── data.yaml
├── src/
│   ├── agro_robot_sim/
│   │   ├── launch/
│   │   ├── config/
│   │   ├── maps/
│   │   ├── urdf/
│   │   └── worlds/
│   └── caatinga_vision/
│       ├── caatinga_vision/
│       ├── launch/
│       ├── config/
│       └── models/
└── README.md
```

## Quick Start

### 1. Prerequisites

- Ubuntu Noble
- ROS 2 jazzy
- `colcon`
- `rosdep`

### 2. Build

```bash
cd /home/joaodemoura/agro_robot_ws
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

### 3. Run `agro_robot_sim` (simulation and navigation)

Start full farm simulation/navigation launch:

```bash
source /opt/ros/humble/setup.bash
source /home/joaodemoura/agro_robot_ws/install/setup.bash
ros2 launch agro_robot_sim fazenda_completa.launch.py
```

Optional SLAM mode:

```bash
ros2 launch agro_robot_sim fazenda_completa.launch.py slam:=True
```

### 4. Run `caatinga_vision` (inference and analytics)

Start the IA pipeline:

```bash
source /opt/ros/humble/setup.bash
source /home/joaodemoura/agro_robot_ws/install/setup.bash
ros2 launch caatinga_vision ia_pipeline.launch.py model_path:=~/models/best.pt
```

Photo capture pipeline:

```bash
ros2 launch caatinga_vision photo_capture.launch.py session_id:=sess_demo
```

## Required External Assets

This repository intentionally excludes large artifacts from version control.

You must provide locally:

- Dataset metadata path: `datasets/agro_v1/data.yaml` (already included)
- Dataset images/labels in local storage when training
- YOLO model weights, for example: `~/models/best.pt`

## Demo and Results

Use this section to attach media when showcasing the project:

- Simulation + navigation demo GIF placeholder: `docs/media/sim_navigation.gif`
- Vision inference overlay screenshot placeholder: `docs/media/inference_overlay.png`
- Analytics/traceability output screenshot placeholder: `docs/media/traceability_status.png`

How to generate media:

1. Run simulation and IA pipelines together.
2. Record screen using OBS or GNOME Screen Recorder.
3. Export short GIFs/screenshots and save them under `docs/media/`.
4. Update this README with embedded markdown images.

## Limitations

- Full field validation with physical hardware is not part of this repository.
- Runtime quality depends on camera conditions and YOLO model quality.
- Some launch files are tailored to ROS 2 Humble and current package versions.

## Next Steps

- Add repeatable benchmark scenarios for navigation and detection quality.
- Add automated unit/integration tests for critical nodes.
- Publish curated demo media and performance metrics per release.

## Contact

- GitHub: `joaodemouragy-hash`
- Email: `joaodemouragy@gmail.com`
