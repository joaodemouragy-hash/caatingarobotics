# caatingarobotics

ROS 2 workspace with the following packages:

- `src/agro_robot_sim`
- `src/caatinga_vision`

## Requirements

- ROS 2 installed
- `colcon`

## Build

```bash
cd /home/joaodemoura/agro_robot_ws
colcon build
```

## Run

Use each package launch files according to your target workflow.

## Dataset and model weights

This repository includes source code and essential configuration only.
Large files (images, labels, model weights such as `yolo11n.pt`) stay outside GitHub.

To train or run inference locally, add:

- the dataset under `datasets/agro_v1/`
- model weights at the paths expected by your scripts/launch files
