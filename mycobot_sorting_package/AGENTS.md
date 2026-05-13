# AGENTS.md: myCobot 280 智能体操作手册

本文件定义 **AI-Local Agent-机械臂** 三层协作架构，是 Agent 和 Skill 迁移的核心指南。

---

## 0. 快速开始（AI 直接控制）

AI（如 OpenCode / ChatGPT / Claude）通过 **HTTP 请求** 直接控制机械臂，无需用户逐条复制粘贴。

**核心原则**：AI 只阅读 `MANUAL.md`（API 说明书）和 `WORKFLOW.md`（流程记录），不阅读、不执行任何代码文件。

### 前提

确保 `robot_service.py` 已启动：
```bash
python3 robot_service.py
# 服务默认运行在 http://localhost:5000
```

### 方式一：HTTP 请求（推荐）

```bash
# 1. 获取状态
curl -X POST http://localhost:5000/api/v1/status

# 2. 读取当前实际位置（不能凭日志推断）
curl -X POST http://localhost:5000/api/v1/position

# 3. （可选）读取历史日志，了解之前做了什么
curl -X POST http://localhost:5000/api/v1/log

# 4. 初始化（复位）
curl -X POST http://localhost:5000/api/v1/init

# 5. 全局检测红色
curl -X POST http://localhost:5000/api/v1/global_detect \
  -H "Content-Type: application/json" -d '{"color": "red"}'

# 6. 移动到目标位置
curl -X POST http://localhost:5000/api/v1/move \
  -H "Content-Type: application/json" -d '{"x": 170, "y": 0, "z": 200}'

# 7. 末端检测
curl -X POST http://localhost:5000/api/v1/detect \
  -H "Content-Type: application/json" -d '{"color": "red"}'

# 8. 控制夹爪
curl -X POST http://localhost:5000/api/v1/gripper \
  -H "Content-Type: application/json" -d '{"action": "close"}'

# 9. 安全关闭
curl -X POST http://localhost:5000/api/v1/shutdown
```

### 方式二：交互式 Local Agent（调试/标定）

```bash
python3 local_agent.py
# LOCAL> {"cmd": "go_home"}
# LOCAL> capture
```

---

## 1. 架构设计（重构后）

```
┌─────────────────────────────────────────────────────────────┐
│  Layer 3: AI (大语言模型，如 ChatGPT/Claude/OpenCode)         │
│  - 理解用户自然语言指令                                       │
│  - 调用 agent_api.py 直接控制机械臂                            │
│  - 或通过 ai_agent_bridge.py 发送 JSON 指令                   │
│  - 根据执行结果（照片/坐标/状态）调整策略                        │
│  ⚠️ 绝不直接实例化 MotionService                               │
└──────────────────────┬──────────────────────────────────────┘
                       │ Python API / JSON / 命令行
                       ↓
┌─────────────────────────────────────────────────────────────┐
│  Layer 2: AI Agent Bridge (ai_agent_bridge.py)              │
│  - 自动管理 Local Agent 守护进程                               │
│  - 原子文件通信（解决旧版轮询竞态问题）                         │
│  - 超时重试、异常恢复                                         │
│  - 人类可在关键动作前介入（可选）                               │
└──────────────────────┬──────────────────────────────────────┘
                       │ command.json / result.json
                       ↓
┌─────────────────────────────────────────────────────────────┐
│  Layer 1: Local Agent (local_agent.py)                       │
│  - 连接机械臂串口 (/dev/cu.usbserial-1110)                     │
│  - 连接摄像头（末端相机 + BRIO 全局相机）                        │
│  - 接收 JSON 指令，执行原子操作                                │
│  - 返回结构化结果：坐标、照片路径、检测状态                       │
│  - 保存完整执行日志到 local_agent_logs/                         │
└──────────────────────┬──────────────────────────────────────┘
                       │ pymycobot / OpenCV
                       ↓
┌─────────────────────────────────────────────────────────────┐
│  Hardware: myCobot 280 + 末端 USB 相机 + Logitech BRIO        │
└─────────────────────────────────────────────────────────────┘
```

### 架构改进点

| 旧问题 | 新解决方案 |
|---|---|
| AI 无法直接控制，需用户复制粘贴 | `agent_api.py` 提供 Python 高级 API |
| 文件轮询 IPC 有原子性/竞态问题 | `ai_agent_bridge.py` 原子 rename + mtime 校验 |
| 颜色检测逻辑在 5+ 个文件中重复 | 统一迁移到 `vision.py` |
| 魔法数字散落各处 | 全部集中到 `config.py` 统一管理 |
| 双单例冲突（MotionService） | 移除全局单例，强制通过 Local Agent 访问 |
| 安全边界检查被 pass | 恢复边界检查，超限直接拒绝运动 |
| robot_agent.py 直接操作硬件 | 已废弃该路径，AI 统一走 bridge |

---

## 2. Agent API 详细说明

### 2.1 初始化与关闭

```python
from agent_api import RobotAPI

api = RobotAPI(auto_connect=True)   # 连接 + go_home + gripper_open
# ... 执行任务 ...
api.shutdown()                       # go_home + gripper_open + 断开
```

### 2.2 原子操作（底层）

```python
from agent_api import move_to, detect, capture, gripper_open, gripper_close

move_to(170, 0, 200)                 # 移动
detect("red")                        # 检测颜色
capture()                            # 拍照
gripper_open()                       # 开爪
gripper_close()                      # 闭爪
```

### 2.3 高级语义操作（推荐）

```python
# 迭代对准（自动修正直到目标在画面中心）
result = api.align(start_x=170, start_y=0, color="red")
# -> {"status": "ok", "camera_x": 172.3, "camera_y": -1.2, "iterations": 3}

# 抓取（自动应用 +30mm X 偏移）
api.pick(camera_x=172.3, camera_y=-1.2)

# 放置
api.place(release_x=60, release_y=230)

# 完整抓取放置（最常用）
result = api.pick_and_place("red", "R")
# -> {"status": "ok", "target_color": "red", "destination": "R", ...}
```

### 2.4 全局视觉粗定位 + 精对准

```python
# 用 BRIO 全局摄像头粗定位 -> 末端精对准 -> 抓取 -> 放置
result = api.global_pick_and_place("red", "R")
```

---

## 3. Local Agent 使用指南（底层调试）

### 3.1 启动守护进程

```bash
# 手动启动
python3 local_agent.py --daemon

# 或由 bridge 自动管理
python3 ai_agent_bridge.py start
```

### 3.2 指令格式

所有指令为 **JSON**，必须包含 `"cmd"` 字段：

```json
{"cmd": "go_home"}
{"cmd": "move_to", "x": 170, "y": 0, "z": 200}
{"cmd": "capture"}
{"cmd": "detect", "color": "red"}
{"cmd": "gripper_open"}
{"cmd": "gripper_close"}
{"cmd": "get_position"}
{"cmd": "scan", "x_range": [140,200], "y_range": [-80,80], "steps": 3}
```

简写（交互模式下不需要大括号）：
```
home      → {"cmd": "go_home"}
capture   → {"cmd": "capture"}
open      → {"cmd": "gripper_open"}
close     → {"cmd": "gripper_close"}
pos       → {"cmd": "get_position"}
```

### 3.3 返回格式

Local Agent 返回 **JSON**：

**成功（拍照）**：
```json
{
  "status": "ok",
  "action": "capture",
  "image_path": "local_agent_logs/20260513_163052/cap_163052_123.jpg",
  "shape": [480, 640, 3]
}
```

**成功（颜色检测）**：
```json
{
  "status": "ok",
  "action": "detect",
  "found": true,
  "color": "red",
  "pixel": [320, 240],
  "area": 4521,
  "offset_from_center": [15, -8],
  "debug_image": "local_agent_logs/.../detect_red_163052.jpg"
}
```

**失败**：
```json
{"status": "error", "action": "detect", "reason": "no target in work area"}
```

---

## 4. 核心约定（Agent 必须遵守）

### 4.1 安全规则
1. **每次任务开始**：`RobotAPI.init()` 会自动 `go_home` + `gripper_open`
2. **抓取前**：确认夹爪已张开
3. **奇异点 WARN** (J5 ~ 1deg)：可忽略
4. **关键动作人在回路**：AI 调用 `pick()` 前，应确认对准（可让用户检查照片）
5. **安全边界已恢复**：`motion_service.py` 会拒绝超出工作空间的坐标

### 4.2 坐标系速查
- 画面上方 → X增加, 画面下方 → X减少
- 画面左方 → Y增加, 画面右方 → Y减少
- Z=200: 扫描, Z=140: 悬停, Z=95: 抓取, Z=105: 放置

### 4.3 夹爪偏移（黄金法则）
```python
grasp_x = camera_center_x + 30  # mm
grasp_y = camera_center_y + 0   # mm
```
仅在 Z=95 验证有效。偏移值可在 `config.py` 或 `grasp_offset.json` 中调整。

### 4.4 迭代对准
- 每次修正 60% 偏移量（`ALIGN_RATIO = 0.6`）
- 收敛条件：|dx_px| < 20 且 |dy_px| < 20
- SCALE_Z200 = 0.32 mm/px

---

## 5. 目标区域坐标

### Zone 1（颜色标记区，Y正方向）
B(蓝色): (-90, 230, 105) | G(绿色): (-30, 230, 105) | R(红色): (30, 230, 105) | Y(黄色): (90, 230, 105)

### Zone 2（垃圾桶区，Y负方向）
蓝色(可回收): (-90, -205, 105) → 释放: (-60, -205)
绿色(厨余):   (-30, -205, 105) → 释放: (0,   -205)
红色(有害):   ( 30, -205, 105) → 释放: (60,  -205)
灰色(其他):   ( 90, -205, 105) → 释放: (120, -205)

**放置偏移**：相机对准坐标 + 30mm X = 实际释放坐标

---

## 6. 文件速查（重构后）

| 文件 | 层级 | 用途 |
|------|------|------|
| **MANUAL.md** | Layer 3 | AI 必读：Robot Service HTTP API 完整说明书 |
| **WORKFLOW.md** | Layer 3 | 抓取流程记录（实际测试验证的最佳实践）|
| **AGENTS.md** | Layer 3 | AI 操作手册（本文件） |
| **SKILL.md** | Layer 3 | 完整技术文档、SOP、移植指南 |
| **robot_service.py** | Layer 2 | HTTP 服务，暴露所有原子操作 API |
| **local_agent.py** | Layer 1 | 本地执行代理（唯一有硬件权限） |
| **motion_service.py** | Layer 1 | 机械臂核心服务（边界检查已恢复） |
| **vision.py** | Layer 1 | 统一视觉模块（颜色检测/相机封装） |
| **config.py** | 全层 | 统一配置（所有参数单一事实来源） |
| **robot_state.log** | 全层 | 服务自动记录的操作日志（AI 可读） |
| **global_camera_calib.json** | Layer 1 | 全局摄像头(BRIO)标定矩阵 |
| **marker_map.json** | Layer 1 | 放置目标坐标 |
| **grasp_offset.json** | Layer 1 | 夹爪偏移参数 |
| **local_agent_logs/** | Layer 1 | 执行日志与照片 |
| **legacy/** | - | 旧脚本归档（自动分拣、旧版视觉、测试脚本等）|

---

## 7. 移植到新环境

### 7.1 必须更新的配置（全部在 config.py 中）
1. 串口地址：`ROBOT_PORT`（或环境变量 `MYCobot_PORT`）
2. 相机索引：`END_CAMERA_INDEX` / `GLOBAL_CAMERA_INDEX`（或环境变量 `MYCobot_END_CAM` / `MYCobot_GLOBAL_CAM`）
3. 重新标定夹爪-相机偏移 → 更新 `grasp_offset.json`
4. 更新 `marker_map.json` 坐标
5. 调整扫描路径范围（`DEFAULT_SCAN_X_RANGE` / `DEFAULT_SCAN_Y_RANGE`）
6. 测量抓取高度 Z（`Z_GRAB`）

### 7.2 标定流程
1. 放置 3 个 AprilTag (36h11) 在固定位置
2. 用 `measure_apriltag_with_arm.py` 测量世界坐标
3. 用 `interactive_calibrate.py` 调整摄像头角度，按空格标定
4. 保存 `global_camera_calib.json`
5. 更新 SKILL.md 中的坐标记录

---

## 8. 最后更新

2026-05-13
- 架构重构完成
- 新增 `config.py` 统一配置
- 新增 `vision.py` 统一视觉
- 新增 `agent_api.py` — AI 可直接调用的 Python API
- 重构 `ai_agent_bridge.py` — 自动守护进程管理 + 原子 IPC
- 修复 `motion_service.py` — 恢复安全边界检查，移除双单例
- `local_agent.py` 精简为纯硬件访问层
- 全局摄像头标定完成 (3个AprilTag)
- 全局视觉分拣验证通过 ✅
