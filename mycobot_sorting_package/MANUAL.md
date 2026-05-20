# MANUAL.md - Robot Service API

This is the API reference for the current robot workflow. Use `robot_service.py` as the only hardware entrypoint.

## Service

Prepare the project virtual environment before running the robot service:

```bash
scripts/setup_env.sh
```

This creates `.venv`, installs project dependencies, and verifies imports for Flask, OpenCV, NumPy, and `pymycobot`.

Start service:

```bash
scripts/run_robot_service.sh
```

The run script uses `.venv/bin/python` and defaults to:

```text
MYCobot_PORT=/dev/cu.usbserial-1130
ROBOT_SERVICE_HOST=127.0.0.1
ROBOT_SERVICE_PORT=5050
```

Do not start the service with a different Python environment unless the user explicitly approves it.

Base URL used in current tests:

```text
http://127.0.0.1:5050
```

All robot actions are HTTP requests. Do not run old local scripts or instantiate `MotionService` directly.

## Control Boundary

Allowed:

- Control the robot through `robot_service.py` HTTP APIs.
- Move the arm, control the gripper, and capture images through `/api/v1/*` endpoints.

Not allowed:

- Import or instantiate `MotionService` directly.
- Run hardware scripts outside `robot_service.py`.
- Use deleted workflows, target zones, or local hardware agents.

If the user requests a physical task, execute it through HTTP APIs unless a safety check fails.

## Current Constants

```text
scan/camera z = 150
hover z = 140
grasp z = 95
release z = 105
rx = -180
ry = 0
rz = -45
grasp offset = [-30, 0]
```

Current valid calibration file:

```text
global_camera_calib_4aruco.json
```

There are no built-in target areas. After grasping, the default behavior is to hold at hover height and wait for an explicit next command or explicit release coordinates.

Do not change `z`, `rx`, `ry`, or `rz` during grasp workflow unless the user explicitly approves. The current centering and grasp offset are only validated for `z=150, rx=-180, ry=0, rz=-45` before applying the grasp offset.

## Endpoints

### `POST /api/v1/status`

Returns service and robot connection status.

Typical response:

```json
{
  "status": "ok",
  "action": "status",
  "connected": true,
  "log_dir": ".../local_agent_logs/20260519_212201"
}
```

### `POST /api/v1/position`

Returns current physical robot coordinates and joint angles. Use this to know the real state; logs are not a substitute.

Typical response:

```json
{
  "status": "ok",
  "action": "position",
  "coords": [53.1, -49.1, 408.4, -92.57, 0.02, -71.34],
  "angles": [-0.7, -0.79, -0.96, -0.96, 19.33, -0.87]
}
```

### `POST /api/v1/init`

Opens gripper and returns the arm to init. Use before overhead vision so the arm does not block the camera.

Typical response:

```json
{
  "status": "ok",
  "action": "init",
  "result": {
    "success": true,
    "target_angles": [0, 0, 0, 0, 20, 0]
  }
}
```

### `POST /api/v1/move`

Moves to Cartesian coordinates.

Parameters:

| name | required | default | notes |
|---|---:|---:|---|
| `x` | yes | - | robot X mm |
| `y` | yes | - | robot Y mm |
| `z` | no | `150` | height mm |
| `rx` | no | `-180` | fixed current pose |
| `ry` | no | `0` | fixed current pose |
| `rz` | no | `-45` | fixed current pose |
| `speed` | no | `20` | motion speed |
| `wait` | no | `true` | wait until done |

Always check `result.success`.

Example:

```json
{"x": -55.9, "y": 191.3, "z": 150, "rx": -180, "ry": 0, "rz": -45, "speed": 30}
```

### `POST /api/v1/capture`

Captures an end-effector camera image.

Typical response:

```json
{
  "status": "ok",
  "action": "capture",
  "image_path": ".../cap_222820_757.jpg",
  "image_url": "/images/20260519_212201/cap_222820_757.jpg",
  "shape": [480, 640, 3]
}
```

### `POST /api/v1/detect`

Captures with the end-effector camera and detects a color.

Request:

```json
{"color": "green"}
```

Typical success response:

```json
{
  "status": "ok",
  "action": "detect",
  "found": true,
  "color": "green",
  "pixel": [325, 216],
  "area": 90871,
  "offset_from_center": [5, -24]
}
```

Supported colors come from `config.py`: `red`, `blue`, `green`, `yellow`, `orange` for end camera; `red`, `blue`, `green`, `yellow` for overhead camera.

Current green end-camera HSV is tuned for the dark green cube used in testing:

```text
green HSV lower = [50, 120, 40]
green HSV upper = [85, 255, 120]
```

If `/api/v1/detect {"color":"green"}` fails while the cube is visible in `capture`, inspect the raw image and run local segmentation with this range. The API currently does not expose per-request HSV overrides.

### `POST /api/v1/global_capture`

Captures an overhead camera image.

Use this when `global_detect` ROI or thresholding needs manual inspection.

### `POST /api/v1/global_detect`

Detects a color in the overhead camera and converts the pixel to robot XY using `global_camera_calib_4aruco.json`.

Request:

```json
{"color": "yellow"}
```

Typical response:

```json
{
  "status": "ok",
  "action": "global_detect",
  "found": true,
  "color": "yellow",
  "pixel": [912, 399],
  "world_xy": [57.4, -200.1],
  "area": 1270
}
```

Overhead coordinates are rough. Use end-effector centering before grasping.

Current overhead ROI in `config.py`:

```text
u_min = 500
u_max = 1200
v_min = 300
v_max = 900
```

The lower edge was expanded to `v_max=900` because the green cube appeared near `v≈855`. If `global_detect green` returns `no contours in ROI`, call `global_capture` and run full-image or wider-ROI local segmentation before deciding the object is absent. The API currently does not expose per-request ROI overrides.

### `POST /api/v1/gripper`

Controls gripper.

Request:

```json
{"action": "open"}
```

or

```json
{"action": "close"}
```

### `GET /dashboard`

Browser dashboard with end and overhead camera streams and ArUco coarse scan controls.

### `GET /dashboard/api/status`

Returns camera status and frame health.

### `GET|POST /dashboard/api/scan-grid`

Reads or saves the ArUco coarse scan grid. Current grid is stored in `aruco_scan_grid_selection.json`.

### `POST /dashboard/api/coarse-scan`

Runs selected ArUco coarse scan points. Each point uses `init -> move(z=150, rz=-45) -> capture -> init`.

## Safety

Check these before any physical task:

1. `status.connected == true`
2. `position` is reasonable
3. gripper is open before approach
4. every `move.result.success == true`
5. if uncertain, call `init`

## Service Recovery Boundary

If a camera or service is unavailable:

1. Stop the physical task.
2. Report the observed problem.
3. Do not kill processes or restart services unless the user explicitly approves.

Only one `robot_service.py` process should run at a time. If the service is restarted, always verify in this order before continuing:

```text
POST /api/v1/status
GET /dashboard/api/status
POST /api/v1/init
POST /api/v1/position
```

If an object appears too large in the end camera at the validated `z=150`, do not automatically switch to `z=200` or any other height. Stop and report the issue, or ask for permission to run a new calibration/centering experiment at a different height.

Safe workspace in config:

```text
X [-280, 280]
Y [-280, 280]
Z [30, 400]
```
