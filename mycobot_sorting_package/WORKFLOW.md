# WORKFLOW.md - Current Robot Workflow

This document records the current tested workflow.

## Authority Boundary

The agent is allowed to control the robot through `robot_service.py` HTTP APIs.

The agent must not bypass HTTP by importing `MotionService`, running hardware scripts, or using deleted local-agent workflows.

If the user asks for a physical task, execute it through HTTP APIs unless a safety check fails.

## Current State

Current valid setup:

```text
robot service: http://127.0.0.1:5050
robot port: /dev/cu.usbserial-1130
end camera: 640x480, index 0
overhead camera: 1920x1080, index 1
calibration: global_camera_calib_4aruco.json
```

Environment setup and service launch:

```bash
scripts/setup_env.sh
scripts/run_robot_service.sh
```

The service must be run with project `.venv/bin/python` so `pymycobot`, Flask, OpenCV, and NumPy are all available in the same environment.

Current fixed robot pose for scan, detection, and grasp work:

```text
z = 150
rx = -180
ry = 0
rz = -45
```

Do not change these pose values during a grasp task unless the user explicitly approves. The current end-camera centering behavior and gripper offset are only validated under this pose.

Current heights:

```text
z=150 scan/camera
z=140 hover
z=95 grasp
z=105 release
```

Current gripper offset:

```text
grasp_x = camera_center_x - 30
grasp_y = camera_center_y
```

This `-30mm` X offset is the current verified grasp correction.

## Clean Workflow Model

There are no default target areas and no automatic placement zones.

The system does not maintain:

```text
color-to-target mapping
default trash bins
automatic place-by-color
```

After a successful grasp, the robot must hold at `z=140` until the user gives either an explicit next action or explicit release coordinates.

## Start Of Any Task

1. Check service:

```bash
curl -sS -X POST http://127.0.0.1:5050/api/v1/status
```

2. Read actual physical position:

```bash
curl -sS -X POST http://127.0.0.1:5050/api/v1/position
```

3. If the arm may block overhead vision or state is uncertain, call `init`:

```bash
curl -sS -X POST http://127.0.0.1:5050/api/v1/init
```

Do not infer current position from logs.

## Overhead Detection

Use `global_detect` to get a rough robot XY from the overhead camera:

```bash
curl -sS -X POST http://127.0.0.1:5050/api/v1/global_detect \
  -H "Content-Type: application/json" -d '{"color":"yellow"}'
```

Experience from the latest tests:

- Yellow was detected at about `[57.4, -200.1]`.
- Green was near the lower edge of the overhead image, so ROI had to include `v` up to `900`.
- Overhead detection is a rough starting point, not a grasp point.

## End-Effector Centering

Move to the overhead rough point at `z=150, rz=-45`, capture or detect with the end camera, then center the object.

Goal:

```text
abs(offset_x) <= 20 px
abs(offset_y) <= 20 px
```

Current `rz=-45` centering experience:

- Target in image right/down: first try increasing X/Y.
- Target in image left/up: first try decreasing X/Y.
- Use small steps near the center; tiny target changes may not move the real robot because of mechanical resolution and position error.
- If the correction makes the offset larger, reverse direction immediately.

Green block threshold experience:

```text
end camera green HSV: [50, 120, 40] to [85, 255, 120]
```

The current green cube is dark. If `/api/v1/detect {"color":"green"}` fails but the cube is visible in `capture`, use this HSV range for local segmentation and continue centering from the measured pixel offset. The API currently does not support per-call HSV overrides.

Overhead ROI experience:

```text
GLOBAL_ROI = u[500,1200], v[300,900]
```

The green cube has appeared near `v≈855`. If `global_detect green` returns `no contours in ROI`, use `global_capture` and run full-image or expanded-ROI segmentation instead of assuming the object is missing. The API currently does not support per-call ROI overrides.

If the object appears too large at `z=150`, do not automatically move to `z=200` or another height. That changes the validated geometry. Stop and report the issue, or ask the user before running a new height experiment.

## Grasp Workflow

Prerequisite: object centered by end-effector camera.

1. Read or use the current camera-centered robot XY.
2. Apply current gripper offset:

```text
grasp_x = camera_center_x - 30
grasp_y = camera_center_y
```

3. Execute grasp:

```bash
POST /api/v1/move {"x": grasp_x, "y": grasp_y, "z": 140, "rx": -180, "ry": 0, "rz": -45}
POST /api/v1/move {"x": grasp_x, "y": grasp_y, "z": 95,  "rx": -180, "ry": 0, "rz": -45}
POST /api/v1/gripper {"action": "close"}
POST /api/v1/move {"x": grasp_x, "y": grasp_y, "z": 140, "rx": -180, "ry": 0, "rz": -45}
```

Always check every `move.result.success`.

If centering fails because the object fills the image, do not infer a grasp point from unstable offsets. Stop and report the failure unless the user approves a different-height experiment.

## Explicit Release Workflow

Only release when the user provides explicit release coordinates or asks for a spatial relation that can be computed from current detections.

Example from the verified task: place green near yellow.

1. Detect yellow overhead.
2. Choose an explicit nearby release point, offset from yellow enough to avoid collision.
3. Execute release:

```bash
POST /api/v1/move {"x": release_x, "y": release_y, "z": 140, "rx": -180, "ry": 0, "rz": -45}
POST /api/v1/move {"x": release_x, "y": release_y, "z": 105, "rx": -180, "ry": 0, "rz": -45}
POST /api/v1/gripper {"action": "open"}
POST /api/v1/move {"x": release_x, "y": release_y, "z": 140, "rx": -180, "ry": 0, "rz": -45}
```

Verified result:

```text
yellow final approx [57.4, -200.1]
green final approx [87.4, -160.5]
distance approx 49.7 mm
```

## 4-ArUco Calibration Workflow

Current calibration is complete. Valid file:

```text
global_camera_calib_4aruco.json
```

Fit quality from the completed calibration:

```text
mean error: 1.801 mm
max error: 1.82 mm
```

Recalibration workflow:

1. `init` so the arm does not block overhead view.
2. Overhead capture must see ArUco ids `0,1,2,3`.
3. Read scan points from `aruco_scan_grid_selection.json` or `/dashboard/api/scan-grid`.
4. For each point, run `init -> move(z=150, rz=-45) -> capture -> init`.
5. For each detected marker, center it with the end camera and record actual robot XY from `/api/v1/position`.
6. After four markers are recorded, call `init` again.
7. Capture one final overhead frame and detect all four marker centers in that same image.
8. Match by marker id and compute the overhead pixel to robot XY affine matrix.
9. Save only `global_camera_calib_4aruco.json`.

## Files To Keep

Runtime and current configuration files:

```text
robot_service.py
motion_service.py
vision.py
config.py
MANUAL.md
WORKFLOW.md
global_camera_calib_4aruco.json
aruco_scan_grid_selection.json
grasp_offset.json
```

Calibration audit data may be kept:

```text
aruco_calibration_session_20260519.json
local_agent_logs/
robot_state.log
```

## Service And Camera Recovery

If `capture`, `global_capture`, `/dashboard/api/status`, or camera health reports a failure:

1. Stop the task.
2. Report the exact failed endpoint and response.
3. Do not kill processes or restart `robot_service.py` without explicit user approval.

Only one `robot_service.py` process should run at a time. If the user approves a restart, verify before continuing:

```text
POST /api/v1/status
GET /dashboard/api/status
POST /api/v1/init
POST /api/v1/position
```
