# MANUAL.md — Robot Service API 说明书

## 概述

本项目提供一个 HTTP 服务 `robot_service.py`，暴露机械臂的所有原子化操作。AI 通过阅读本文档，使用标准的 HTTP 请求控制机械臂。

**服务地址**: `http://localhost:5000`（默认）
**API 前缀**: `/api/v1/`
**通信格式**: JSON

AI 不应直接 import 本项目中的任何 Python 模块，也不应执行项目中的 .py 文件。AI 应该只通过 HTTP 调用此服务。

---

## API 列表

所有 API 统一为 `POST` 请求，Content-Type 为 `application/json`。

### 1. init — 初始化

**端点**: `POST /api/v1/init`

**参数**: 无

**功能**: 连接机械臂，回零位，张开夹爪。每个任务开始前必须调用。

**成功返回**:
```json
{
  "status": "ok",
  "action": "init",
  "result": {
    "success": true,
    "target_angles": [0, 0, 0, 0, 20, 0],
    "actual_angles": [0.1, -0.2, 0.1, 0, 20.1, 0],
    "elapsed_sec": 2.3
  }
}
```

---

### 2. shutdown — 安全关闭

**端点**: `POST /api/v1/shutdown`

**参数**: 无

**功能**: 回零位，张开夹爪，断开机械臂连接。

**成功返回**:
```json
{
  "status": "ok",
  "action": "shutdown",
  "result": {...}
}
```

---

### 3. move — 移动

**端点**: `POST /api/v1/move`

**参数**:
| 参数 | 类型 | 必填 | 默认值 | 说明 |
|------|------|------|--------|------|
| x | float | 是 | - | X 坐标 (mm) |
| y | float | 是 | - | Y 坐标 (mm) |
| z | float | 否 | 200 | Z 坐标 (mm) |
| rx | float | 否 | -180 | RX 角度 |
| ry | float | 否 | 0 | RY 角度 |
| rz | float | 否 | 45 | RZ 角度 |
| speed | int | 否 | 20 | 速度 (1-50) |
| wait | bool | 否 | true | 是否等待到位 |

**高度语义**:
- `z=200`: 扫描/拍照高度
- `z=140`: 接近/悬停高度
- `z=95`: 抓取高度
- `z=105`: 放置高度

**成功返回**:
```json
{
  "status": "ok",
  "action": "move",
  "target": [170, 0, 200, -180, 0, 45],
  "result": {
    "success": true,
    "actual": [169.8, 0.1, 200.2, -180, 0, 45.1],
    "elapsed_sec": 1.5
  }
}
```

**错误返回**:
```json
{
  "status": "error",
  "action": "move",
  "reason": "坐标 (300, 0, 200) 超出安全工作空间!"
}
```

---

### 4. capture — 拍照

**端点**: `POST /api/v1/capture`

**参数**: 无

**功能**: 用末端相机拍照。

**成功返回**:
```json
{
  "status": "ok",
  "action": "capture",
  "image_path": "/path/to/cap_163052_123.jpg",
  "image_url": "/images/20260513_163052/cap_163052_123.jpg",
  "shape": [480, 640, 3]
}
```

AI 可以通过 `image_url` 访问图片：
```
GET http://localhost:5000/images/20260513_163052/cap_163052_123.jpg
```

---

### 5. detect — 颜色检测

**端点**: `POST /api/v1/detect`

**参数**:
| 参数 | 类型 | 必填 | 说明 |
|------|------|------|------|
| color | string | 是 | 颜色名称: "red", "blue", "green", "yellow" |

**功能**: 在当前位置拍照，检测指定颜色物品。

**成功返回（找到）**:
```json
{
  "status": "ok",
  "action": "detect",
  "found": true,
  "color": "red",
  "pixel": [320, 240],
  "area": 4521,
  "offset_from_center": [15, -8],
  "debug_image_url": "/images/20260513_163052/detect_red_163052.jpg"
}
```

**成功返回（未找到）**:
```json
{
  "status": "ok",
  "action": "detect",
  "found": false,
  "color": "red",
  "reason": "no contours",
  "debug_image_url": "/images/.../detect_red_163052.jpg"
}
```

**重要**: `offset_from_center` 表示目标像素相对于画面中心的偏移。
- `[+15, -8]` 表示目标在画面中心右偏 15px、上偏 8px

---

### 5.5 global_detect — 全局摄像头颜色检测

**端点**: `POST /api/v1/global_detect`

**参数**:
| 参数 | 类型 | 必填 | 说明 |
|------|------|------|------|
| color | string | 是 | 颜色名称: "red", "blue", "green", "yellow" |

**功能**: 使用全局摄像头(BRIO)检测颜色，直接返回世界坐标。不需要先移动机械臂。

**成功返回（找到）**:
```json
{
  "status": "ok",
  "action": "global_detect",
  "found": true,
  "color": "red",
  "pixel": [717, 587],
  "world_xy": [182.7, 78.6],
  "area": 2046,
  "raw_image_url": "/images/.../global_raw_xxx.jpg",
  "debug_image_url": "/images/.../global_detect_red_xxx.jpg"
}
```

**成功返回（未找到）**:
```json
{
  "status": "ok",
  "action": "global_detect",
  "found": false,
  "color": "red",
  "reason": "no contours in ROI",
  "raw_image_url": "/images/.../global_raw_xxx.jpg"
}
```

**重要**: 
- `pixel` 是全局摄像头画面中的像素坐标（1920x1080）
- `world_xy` 是转换后的机械臂世界坐标（mm），可直接用于 `move`
- 全局检测精度较低，通常需要配合末端相机精对准

---

### 6. gripper — 夹爪控制

**端点**: `POST /api/v1/gripper`

**参数**:
| 参数 | 类型 | 必填 | 说明 |
|------|------|------|------|
| action | string | 是 | "open" 或 "close" |

**成功返回**:
```json
{
  "status": "ok",
  "action": "gripper",
  "state": "open"
}
```

---

### 7. position — 获取当前位置

**端点**: `POST /api/v1/position`

**参数**: 无

**成功返回**:
```json
{
  "status": "ok",
  "action": "position",
  "coords": [170.0, 0.0, 200.0, -180.0, 0.0, 45.0],
  "angles": [0.0, -20.0, 40.0, 0.0, 20.0, 45.0]
}
```

---

### 8. scan — 蛇形扫描

**端点**: `POST /api/v1/scan`

**参数**:
| 参数 | 类型 | 必填 | 默认值 | 说明 |
|------|------|------|--------|------|
| x_range | list | 否 | [140, 200] | X 范围 [min, max] |
| y_range | list | 否 | [-80, 80] | Y 范围 [min, max] |
| steps | int | 否 | 3 | 每轴步数 |
| z | float | 否 | 200 | 扫描高度 |

**功能**: 在指定区域内蛇形移动并拍照。

**成功返回**:
```json
{
  "status": "ok",
  "action": "scan",
  "images": [
    {"filename": "scan_140_-80.jpg", "url": "/images/.../scan_140_-80.jpg"},
    {"filename": "scan_170_-80.jpg", "url": "/images/.../scan_170_-80.jpg"}
  ],
  "count": 2
}
```

---

### 9. status — 服务状态

**端点**: `POST /api/v1/status`

**参数**: 无

**功能**: 检查服务是否连接了机械臂。

**返回**:
```json
{
  "status": "ok",
  "action": "status",
  "connected": true,
  "log_dir": "local_agent_logs/20260513_163052"
}
```

---

## 坐标系

### 图像坐标系（像素）
- 画面中心 = 图像中心 `(width/2, height/2)`
- `offset_from_center[0] > 0`: 目标在画面中心右侧
- `offset_from_center[0] < 0`: 目标在画面中心左侧
- `offset_from_center[1] > 0`: 目标在画面中心下方
- `offset_from_center[1] < 0`: 目标在画面中心上方

### 机械臂坐标系（笛卡尔，单位 mm）
- **X 轴**: 前后方向（正 = 远离底座，即画面上方）
- **Y 轴**: 左右方向（正 = 左侧）
- **Z 轴**: 上下方向（正 = 上方）

**坐标映射规则**（Z=200mm 时）:
```
dX = -offset_y_px * SCALE * RATIO
dY = -offset_x_px * SCALE * RATIO
```
其中 `SCALE = 0.32 mm/px`, `RATIO = 0.6`

### 夹爪偏移（抓取修正）
相机光心与夹爪中心存在固定偏移：
```
grasp_x = camera_x + 30  # mm
grasp_y = camera_y + 0   # mm
```
此偏移仅在 Z=95 时有效。

---

## 安全规则

1. **每次任务开始前必须调用 `init`**
2. **抓取前必须确认夹爪已张开**（调用 `gripper` action="open"）
3. **安全边界**: 超出安全工作空间的坐标会被拒绝
   - X: [-280, 280]
   - Y: [-280, 280]
   - Z: [30, 400]
4. **奇异点警告**: J5 接近 0° 时运动可能不稳定，可忽略但需注意
5. **关键动作建议**: `gripper` action="close" 前，建议先 `capture` 确认对准正确

---

## 目标区域坐标

### Zone 1（颜色标记区，Y正方向）

| 标记 | 颜色 | 相机对准坐标 | 释放坐标 |
|------|------|-------------|---------|
| B | 蓝色 | (-90, 230, 105) | (-60, 230, 105) |
| G | 绿色 | (-30, 230, 105) | (0, 230, 105) |
| R | 红色 | (30, 230, 105) | (60, 230, 105) |
| Y | 黄色 | (90, 230, 105) | (120, 230, 105) |

**释放坐标 = 相机对准坐标 + (30, 0)**

### Zone 2（垃圾桶区，Y负方向）

| 类型 | 颜色 | 相机对准坐标 | 释放坐标 |
|------|------|-------------|---------|
| 可回收 | 蓝色 | (-90, -205, 105) | (-60, -205, 105) |
| 厨余 | 绿色 | (-30, -205, 105) | (0, -205, 105) |
| 有害 | 红色 | (30, -205, 105) | (60, -205, 105) |
| 其他 | 灰色 | (90, -205, 105) | (120, -205, 105) |

---

## 完整任务示例

### 示例 1: 抓取红色物品放到 R 区

```python
import requests

BASE = "http://localhost:5000/api/v1"

# 1. 初始化
requests.post(f"{BASE}/init")

# 2. 移动到扫描位置
requests.post(f"{BASE}/move", json={"x": 170, "y": 0, "z": 200})

# 3. 检测红色
r = requests.post(f"{BASE}/detect", json={"color": "red"}).json()
if not r["found"]:
    print("未找到红色物品")
    requests.post(f"{BASE}/shutdown")
    exit()

offset = r["offset_from_center"]

# 4. 迭代对准（根据偏移修正位置）
# 偏移 (+15, -8) -> 需要微调
# dX = -(-8) * 0.32 * 0.6 = +1.5
# dY = -(+15) * 0.32 * 0.6 = -2.9
# camera_x = 170 + 1.5 = 171.5
# camera_y = 0 + (-2.9) = -2.9
camera_x, camera_y = 171.5, -2.9
requests.post(f"{BASE}/move", json={"x": camera_x, "y": camera_y, "z": 200})

# 再次检测确认
r = requests.post(f"{BASE}/detect", json={"color": "red"}).json()
if abs(r["offset_from_center"][0]) > 20 or abs(r["offset_from_center"][1]) > 20:
    print("对准未收敛，继续修正")
    # ... 继续迭代

# 5. 抓取
grasp_x = camera_x + 30  # 201.5
grasp_y = camera_y        # -2.9

requests.post(f"{BASE}/move", json={"x": grasp_x, "y": grasp_y, "z": 140})  # 悬停
requests.post(f"{BASE}/move", json={"x": grasp_x, "y": grasp_y, "z": 95})   # 降下
requests.post(f"{BASE}/gripper", json={"action": "close"})                    # 闭合
requests.post(f"{BASE}/move", json={"x": grasp_x, "y": grasp_y, "z": 140})  # 抬升

# 6. 移动到 R 区
requests.post(f"{BASE}/move", json={"x": 60, "y": 230, "z": 140})   # 上方
requests.post(f"{BASE}/move", json={"x": 60, "y": 230, "z": 105})    # 降下
requests.post(f"{BASE}/gripper", json={"action": "open"})              # 释放
requests.post(f"{BASE}/move", json={"x": 60, "y": 230, "z": 140})    # 撤离

# 7. 结束
requests.post(f"{BASE}/shutdown")
```

### 示例 2: 扫描桌面

```python
# 蛇形扫描 3x3 区域
r = requests.post(f"{BASE}/scan", json={
    "x_range": [140, 200],
    "y_range": [-80, 80],
    "steps": 3,
    "z": 200
}).json()

for img in r["images"]:
    print(f"图片: {img['url']}")
```

---

## 启动服务

### 前台运行
```bash
python3 robot_service.py
```

### 指定端口
```bash
python3 robot_service.py --port 8080
```

### 后台运行
```bash
python3 robot_service.py --daemon
```

---

## 原子操作调用规范（AI 必须遵守）

Service 只暴露原子操作（move、capture、gripper 等），**抓取、放置等流程必须由 AI 逐条发送 HTTP 请求组装**。AI 作为调用方，必须对每一步的执行结果负责。

### 1. 检查每个 API 的返回状态

所有 API 返回 JSON，必须同时检查：
1. HTTP 状态码是否为 200
2. JSON 中的 `"status"` 是否为 `"ok"`
3. 对于 `move` API，额外检查 `"result.success"` 是否为 `true`

**正确的判断逻辑**：
```python
response = requests.post("http://localhost:5000/api/v1/move", json={"x": 234, "y": 40, "z": 95})
data = response.json()

if response.status_code != 200:
    raise Exception(f"HTTP 错误: {response.status_code}")

if data.get("status") != "ok":
    raise Exception(f"业务错误: {data.get('reason')}")

# move 操作额外检查实际执行结果
if data["action"] == "move" and not data["result"]["success"]:
    raise Exception(f"运动失败: 目标={data['target']}, 实际={data['result'].get('actual')}")
```

### 2. 失败立即停止（Fail-Fast）

**绝不能在某个步骤失败后继续执行下一步。** 例如：

❌ **错误做法**（会导致空中夹爪）：
```python
requests.post("/api/v1/move", json={"x": 234, "y": 40, "z": 140})  # 可能失败
requests.post("/api/v1/move", json={"x": 234, "y": 40, "z": 95})   # 不检查就继续
requests.post("/api/v1/gripper", json={"action": "close"})           # 空中闭合！
```

✅ **正确做法**：
```python
def move_or_die(x, y, z):
    r = requests.post("/api/v1/move", json={"x": x, "y": y, "z": z}).json()
    if r.get("status") != "ok" or not r["result"]["success"]:
        raise Exception(f"Move failed: {r.get('reason', 'unknown')}")
    return r

move_or_die(234, 40, 140)
move_or_die(234, 40, 95)
requests.post("/api/v1/gripper", json={"action": "close"})
```

### 3. move API 的特殊检查

`move` 返回的 `result` 字段：
```json
{
  "status": "ok",
  "action": "move",
  "target": [234, 40, 95, -180, 0, 45],
  "result": {
    "success": true,
    "actual": [232.1, 42.3, 93.5, -178.2, 1.4, 45.1],
    "elapsed_sec": 1.5
  }
}
```

注意：
- `"status": "ok"` 只表示 API 调用成功，不代表机械臂已经到位
- `"result.success": true` 表示运动正常完成
- `"result.actual"` 是实际到达的位置，可能与 target 有 1-3mm 误差（正常）
- 如果 `"result.success": false`，说明运动被中断或超时

### 4. 建议的代码模板

```python
import requests

BASE = "http://localhost:5000/api/v1"

def api(action, **params):
    """调用 API，失败时抛出异常"""
    r = requests.post(f"{BASE}/{action}", json=params)
    data = r.json()
    
    if r.status_code != 200:
        raise Exception(f"HTTP {r.status_code}: {data}")
    
    if data.get("status") != "ok":
        raise Exception(f"API error: {data.get('reason')}")
    
    # move 操作额外检查
    if action == "move" and not data["result"]["success"]:
        raise Exception(f"Move failed to target {data['target']}")
    
    return data

# 使用示例
try:
    api("init")
    api("move", x=170, y=0, z=200)
    result = api("detect", color="red")
    # ... 根据结果计算修正 ...
    api("move", x=234, y=40, z=140)   # 悬停
    api("move", x=234, y=40, z=95)    # 降下
    api("gripper", action="close")      # 抓取
    api("move", x=234, y=40, z=140)   # 抬升
    api("shutdown")
except Exception as e:
    print(f"任务失败: {e}")
    api("shutdown")  # 无论如何都要安全关闭
```

---

## 状态日志 API

Service 会自动记录所有 API 调用到 `robot_state.log`。AI 可以通过此 API 读取历史操作，了解机械臂之前做了什么、当前可能在哪里。

### 10. log — 读取操作日志

**端点**: `GET /api/v1/log` 或 `POST /api/v1/log`

**参数**:
| 参数 | 类型 | 必填 | 默认值 | 说明 |
|------|------|------|--------|------|
| limit | int | 否 | 50 | 返回最近 N 条日志 |
| clear | bool | 否 | false | 是否清空日志（谨慎使用）|

**功能**: 读取服务启动以来的所有操作记录。

**成功返回**:
```json
{
  "status": "ok",
  "action": "log",
  "count": 4,
  "entries": [
    {
      "timestamp": "2026-05-13T19:57:41.126963",
      "action": "status",
      "params": {},
      "result_summary": {
        "status": "ok"
      }
    },
    {
      "timestamp": "2026-05-13T19:57:52.063007",
      "action": "init",
      "params": {},
      "result_summary": {
        "move_success": true,
        "status": "ok"
      }
    },
    {
      "timestamp": "2026-05-13T19:57:56.554048",
      "action": "move",
      "params": {
        "x": 170.0,
        "y": 0.0,
        "z": 200.0
      },
      "result_summary": {
        "actual": [169.4, -0.9, 194.0, -177.59, 1.97, 43.51],
        "move_success": true,
        "status": "ok"
      }
    },
    {
      "timestamp": "2026-05-13T19:57:56.613532",
      "action": "detect",
      "params": {
        "color": "red"
      },
      "result_summary": {
        "found": true,
        "offset_from_center": [-254, -127],
        "pixel": [66, 113],
        "status": "ok"
      }
    }
  ]
}
```

**日志内容说明**:
- `timestamp`: ISO 8601 格式时间戳
- `action`: 调用的 API 名称
- `params`: 传入的关键参数（x, y, z, color 等）
- `result_summary`: 结果摘要
  - `status`: "ok" 或 "error"
  - `move_success`: move 操作是否成功到达
  - `actual`: move 后实际到达的坐标
  - `found`: detect 是否找到目标
  - `offset_from_center`: detect 的像素偏移
  - `world_xy`: global_detect 的世界坐标

**使用场景**:

1. **AI 重启后恢复上下文**:
```bash
# AI 刚启动，不知道之前做了什么
GET /api/v1/log?limit=10

# 从日志发现：
# - 最后一次 move 到了 (234, 40, 140)
# - 夹爪状态是 close
# - 当前可能在抓取后的抬升位置
```

2. **排查失败原因**:
```bash
# 任务失败了，查看日志找哪一步出错
GET /api/v1/log?limit=50

# 发现某次 move 的 move_success=false
# 原因可能是超出安全边界
```

3. **清空日志（谨慎）**:
```bash
POST /api/v1/log
{"clear": true}
```

**注意事项**:
- 日志文件保存在项目根目录 `robot_state.log`
- 服务重启后日志不丢失（追加模式）
- 日志为 NDJSON 格式（每行一条 JSON）
- 建议 AI 在任务开始时读取日志了解当前状态

---

## 错误码

| HTTP 状态码 | 含义 |
|------------|------|
| 200 | 操作成功（但业务逻辑可能返回 status: error） |
| 404 | 端点不存在 |
| 500 | 服务端执行错误 |

所有响应都包含 JSON body，通过 `"status": "ok"` 或 `"status": "error"` 判断业务结果。

---

*最后更新: 2026-05-13*
