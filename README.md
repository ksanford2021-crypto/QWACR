# QWACR - Autonomous Ground Robot

FPL Autonomous Robot Repo

## Overview
This repository contains the ROS workspace for QWACR (Quadruped Walking Autonomous Cognitive Robot), an autonomous ground robot development project.

## Repository Structure
```
qwacr_ws/          # ROS workspace directory
  src/             # Source packages for robot development
```

## Getting Started

### Prerequisites
- ROS (Robot Operating System)
- Python 3.x
- VS Code (recommended for development)

### Workspace Setup
The `qwacr_ws` directory contains the ROS workspace for robot development. To build the workspace:

```bash
cd qwacr_ws
catkin_make  # or catkin build if using catkin_tools
source devel/setup.bash
```

## Development
This workspace is actively being developed for autonomous ground robot capabilities including:
- Navigation
- Perception
- Control systems
- Autonomy

## Contributing
This is an active development repository. Changes should be committed regularly to preserve work progress.
