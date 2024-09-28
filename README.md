# Egg Collector Robot

This robot is used to collect eggs in scattered duck houses. It is able to use SLAM to navigate itself to the designated places and search for eggs in the nearby area.

## Algorithms Used
- **SLAM Algorithm**: Cartographer
- **Object Detection Algorithm**: YOLOv5
- **Arm Planning**: MoveIt!

There isn't much original algorithm. Before running, you'll need to install the requirements first.

## Current Status
This robot still has a lot of drawbacks and will not be maintained by the author anymore. If anyone wants to push through, contact the author or publish issues.

## Main Folder
Contains only essential packages.

## Usage

### Mapping
Use `mapping.launch` to start mapping. You may use `wheeltec_robot_rc` to remotely control the robot to explore around and `script_name.sh` to save the map to `/map`. The robot arm will also be activated; it should be set to retracted before you start mapping.

### Navigation
Use `navigation.launch` to launch and use the saved map as the navigation map file.

### Egg Collection
Use `yolov5ros yolonode.launch` to start the collecting and search node.
