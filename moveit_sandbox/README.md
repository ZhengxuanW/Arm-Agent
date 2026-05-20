# MoveIt Sandbox

This folder is isolated from `mycobot_sorting_package`. It is only for testing MoveIt path planning before deciding whether to integrate it into the robot HTTP service.

## Scope

- Test MoveIt2 planning in a disposable ROS2 Humble environment.
- Do not connect to the physical myCobot.
- Do not import or modify `motion_service.py` or `robot_service.py`.
- Do not reuse this as production control code until the service boundary is designed explicitly.

## Requirements

- Docker Desktop on macOS, or Docker on Linux.
- Internet access for the first image build.

## Build

```bash
cd moveit_sandbox
./scripts/build.sh
```

## Start A Shell

```bash
cd moveit_sandbox
./scripts/shell.sh
```

Inside the container:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Headless Panda Demo

Use the standard Panda MoveIt config first. This verifies the MoveIt stack before adding a myCobot model.

Terminal 1 inside the container:

```bash
source /opt/ros/humble/setup.bash
ros2 launch moveit_resources_panda_moveit_config demo.launch.py rviz:=false
```

Terminal 2 inside the container:

```bash
source /opt/ros/humble/setup.bash
source /sandbox/ws/install/setup.bash
ros2 run moveit_probe wait_for_move_group.py
```

Expected result: the probe reports that `/move_action` is available.

## Next Step For myCobot

After the Panda demo works, add a separate myCobot URDF/SRDF/config package under this sandbox only. Keep the output as a planning-only service until we explicitly map MoveIt trajectories back into the existing HTTP service safety model.
