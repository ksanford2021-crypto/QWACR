# QWACR Workspace

This is the ROS workspace for the QWACR autonomous ground robot project.

## Workspace Structure
```
qwacr_ws/
├── src/                 # ROS packages source code
├── build/              # Build artifacts (not tracked in git)
├── devel/              # Development space (not tracked in git)
├── install/            # Install space (not tracked in git)
└── README.md           # This file
```

## Building the Workspace

After cloning this repository, build the workspace:

```bash
cd qwacr_ws
catkin_make
# or if using catkin_tools:
# catkin build
```

## Sourcing the Workspace

After building, source the workspace to use the packages:

```bash
source devel/setup.bash
```

To automatically source on terminal startup, add to your `~/.bashrc`:
```bash
echo "source ~/path/to/qwacr_ws/devel/setup.bash" >> ~/.bashrc
```

## Development Workflow

1. Create or modify packages in the `src/` directory
2. Build the workspace: `catkin_make` or `catkin build`
3. Source the workspace: `source devel/setup.bash`
4. Test your changes
5. Commit source code (build artifacts are excluded via .gitignore)

## Notes
- Build, devel, install, and log directories are excluded from version control
- Only source code in the `src/` directory should be committed
- VS Code workspace files are also excluded from version control
