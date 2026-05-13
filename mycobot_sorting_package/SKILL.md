# SKILL: myCobot 280 桌面垃圾分拣（端到端）

## 版本
- Created: 2026-05-11
- Updated: 2026-05-11（两轮实测验证通过）
- Author: OpenCode Agent
- Hardware: myCobot 280 + 末端 USB 相机 (640×480)

---

## 1. 能力概述

本 SKILL 描述一套完整的**视觉引导桌面分拣系统**，包含：
- 蛇形扫描覆盖待分拣区域
- 人工/自动识别物品类型
- 迭代居中对准 + 夹爪偏移修正
- 抓取 → 放置到目标区域
- 全流程已验证成功率 100%（2/2 轮）

---

## 2. 硬件配置（可移植修改）

| 项目 | 当前值 | 移植时需确认 |
|------|--------|-------------|
| 机械臂 | myCobot 280 | 波特率/协议兼容 |
| 串口 | `/dev/cu.usbserial-120` | 新环境端口 |
| 波特率 | 1000000 | 同上 |
| 末端相机 | Camera[0], 640×480 | index/分辨率 |
| 夹爪最大开度 | ~55 mm | 测量确认 |
| 物品尺寸 | ~29 mm 立方体 | 调整抓取高度 |

---

## 3. 坐标系定义（核心，不可改）

### 3.1 笛卡尔世界坐标
- 从 `go_init` `[0,0,0,0,20,0]` 建立
- **X+**: 机械臂朝前（远离身体）
- **X-**: 机械臂朝后（靠近身体）
- **Y+**: 机械臂朝左
- **Y-**: 机械臂朝右

### 3.2 相机画面 → 世界坐标映射（已验证）

| 画面方向 | 世界方向 | 机械臂动作 |
|---------|---------|-----------|
| 画面上方 | X 正方向 | X 增加 |
| 画面下方 | X 负方向 | X 减少 |
| 画面左方 | Y 正方向 | Y 增加 |
| 画面右方 | Y 负方向 | Y 减少 |

换算公式（Z=200mm）：
```python
delta_X_mm = -delta_y_px * 0.32   # 画面上移 = X增加
delta_Y_mm = -delta_x_px * 0.32   # 画面左移 = Y增加
```

### 3.3 最优视角
- **rz = 45°**：文字正向可读，图案清晰，阴影最少
- RX=-180°, RY=0° 固定（末端相机朝下）

---

## 4. 核心校准参数（移植后必须重新标定）

### 4.1 夹爪-相机偏移 ⭐最关键

当相机光轴对准物品中心时，夹爪实际位于物品的 **X- 方向 30mm 处**。

```python
grasp_x = detect_x + 30   # mm
grasp_y = detect_y + 0    # mm
```

**验证数据**：
- 相机对准: `(164.6, -1.9)` → 夹空（西瓜在夹爪 X+30mm）
- 修正后: `(194.6, -1.9)` → **抓取成功**

> ⚠️ 此偏移仅在 **Z=95mm 抓取高度** 验证。若更改高度，需重新校准。

### 4.2 像素比例
| 高度 Z | 比例 mm/px |
|--------|-----------|
| 200 mm | 0.32 |
| 120 mm | 0.20（近距离不稳定，不推荐） |

### 4.3 高度配置
| 用途 | Z 值 | 说明 |
|------|------|------|
| 扫描/检测 | 200 mm | 视野覆盖好，检测稳定 |
| 悬停过渡 | 140 mm | 移动缓冲，避免碰撞 |
| 抓取 | **95 mm** | 29mm 立方体已验证 |
| 放置 | 105 mm | 略高以防压坏 |

---

## 5. 标准操作程序（SOP）

### 5.1 阶段0: 安全初始化
```python
ms = MotionService()
ms.connect()
ms.gripper_open()          # 释放残留物品
time.sleep(0.5)
ms.go_init(speed=10)       # 回零位 [0,0,0,0,20,0]
time.sleep(0.5)
```

### 5.2 阶段1: 蛇形扫描
```python
with open("path_sorting_area_rz45.json") as f:
    data = json.load(f)

for wp in data["waypoints"]:
    ms.move_to(x=wp["x"], y=wp["y"], z=200,
               rx=-180, ry=0, rz=45, speed=20)
    time.sleep(2.0)
    frame = ms.capture()
    cv2.imwrite(f"scan_{wp['name']}.jpg", frame)
```

扫描范围：X=140~200, Y=-80~80，共 9 路点（3行×3列，蛇形）。
> 2026-05-12 更新：从 20 点精简为 9 点，Y 范围从 ±120 缩至 ±80，更聚焦实际工作区。实测橙/红色方块在前排/中排区域被 5/9 点位拍到。无遗漏。

### 5.3 阶段2: 物品识别
当前策略：**人工审查 scan 照片** → 判断物品类型 → 确定目标区域。

颜色→目标区映射（默认 Zone 1 标记区）：
| 物品颜色 | 默认目标区 | Zone 1 坐标 |
|---------|-----------|------------|
| 蓝色 | B (蓝色标记) | (-90, 230, 105) |
| 绿色 | G (绿色标记) | (-30, 230, 105) |
| 红色 | R (红色标记) | (30, 230, 105) |
| 黄色 | Y (黄色标记) | (90, 230, 105) |

> **原则：物品颜色 → 对应颜色标记区**。不再强制语义映射（如"黄色=电池盒=有害"）。
>
> 用户可手动指定 Zone 2（垃圾桶区）或其他目标。

### 5.4 阶段3: 迭代居中对准 ⭐关键
```python
def iterative_center(ms, start_x, start_y, color_lower, color_upper,
                     max_iter=5, tol_px=20, scale=0.32):
    """迭代微调使物品进入画面中心"""
    x, y = start_x, start_y
    for i in range(max_iter):
        ms.move_to(x=x, y=y, z=200, rx=-180, ry=0, rz=45, speed=15)
        time.sleep(2.5)
        frame = ms.capture()

        # 检测物品像素中心
        cx, cy = detect_color_center(frame, color_lower, color_upper)
        h, w = frame.shape[:2]
        dx = cx - w//2
        dy = cy - h//2

        if abs(dx) < tol_px and abs(dy) < tol_px:
            return x, y  # CENTERED

        # 保守修正：只走计算值的 60%
        x += (-dy) * scale * 0.6
        y += (-dx) * scale * 0.6

    return x, y  # best effort
```

**经验**：每次走满计算偏移量容易过度修正，建议只走 **50-70%**。

### 5.5 阶段4: 应用偏移
```python
center_x, center_y = iterative_center(...)   # 相机对准坐标
grasp_x = center_x + 30
grasp_y = center_y + 0
```

### 5.6 阶段5: 抓取
```python
# 悬停
ms.move_to(x=grasp_x, y=grasp_y, z=140, rx=-180, ry=0, rz=45, speed=15)
time.sleep(2.0)
# 降下
ms.move_to(x=grasp_x, y=grasp_y, z=95, rx=-180, ry=0, rz=45, speed=10)
time.sleep(1.5)
# 夹取
ms.gripper_close()
time.sleep(1.0)
# 抬升
ms.move_to(x=grasp_x, y=grasp_y, z=140, rx=-180, ry=0, rz=45, speed=15)
time.sleep(1.5)
```

### 5.7 阶段6: 放置 ⭐ 同样需要偏移

放置时夹爪相对于相机同样有 +30mm X 偏移。因此：
- 文档中记录的是**相机对准坐标**（标记在画面中心）
- 实际释放坐标需要 **+30mm X**

```python
target = marker_map[category]   # 查表得到相机对准坐标
release_x = target["x"] + 30   # 放置偏移！
release_y = target["y"] + 0

# 移动到目标上方（使用释放坐标）
ms.move_to(x=release_x, y=release_y, z=140,
           rx=-180, ry=0, rz=45, speed=20)
time.sleep(2.5)
# 降下放置
ms.move_to(x=release_x, y=release_y, z=105,
           rx=-180, ry=0, rz=45, speed=15)
time.sleep(1.5)
# 释放
ms.gripper_open()
time.sleep(0.5)
# 抬升
ms.move_to(x=release_x, y=release_y, z=140,
           rx=-180, ry=0, rz=45, speed=20)
time.sleep(1.5)
```

### 5.8 阶段7: 回零
```python
ms.go_init(speed=10)
time.sleep(2.0)
```

---

## 6. 目标区域坐标

### Zone 1（颜色标记区，Y 正方向）
已 2026-05-11 实际验证：

```json
{
  "B": {"x": -90, "y": 230, "z": 105, "name": "蓝色标记"},
  "G": {"x": -30, "y": 230, "z": 105, "name": "绿色标记"},
  "R": {"x":  30, "y": 230, "z": 105, "name": "红色标记"},
  "Y": {"x":  90, "y": 230, "z": 105, "name": "黄色标记"},
  "1": {"x": -90, "y": 200, "z": 105, "name": "1号位置"},
  "2": {"x": -30, "y": 200, "z": 105, "name": "2号位置"},
  "3": {"x":  30, "y": 200, "z": 105, "name": "3号位置"},
  "4": {"x":  90, "y": 200, "z": 105, "name": "4号位置"}
}
```

### Zone 2（垃圾桶区，Y 负方向）
```json
{
  "blue":  {"x": -90, "y": -205, "z": 105, "name": "可回收"},
  "green": {"x": -30, "y": -205, "z": 105, "name": "厨余"},
  "red":   {"x":  30, "y": -205, "z": 105, "name": "有害"},
  "gray":  {"x":  90, "y": -205, "z": 105, "name": "其他"}
}
```

---

## 7. 速度配置

| 阶段 | 速度 | 说明 |
|------|------|------|
| go_init | 10 | 安全回零 |
| 扫描 | 20 | 稳定，减少抖动 |
| 大范围移动 | 20-30 | 快速就位 |
| 接近/对准 | 15 | 精细控制 |
| 抓取降下 | 10 | 最慢，避免碰撞 |
| 夹爪动作 | - | 开/关后等待 0.5-1.0s |

---

## 8. 已知限制

1. **奇异点**: J5≈0° 时出现 WARN，运动仍可执行
2. **安全边界**: 已禁用（`motion_service.py` 中改为 `pass`），需人工确保坐标安全
3. **视觉检测**: 当前依赖人工审查照片，自动颜色检测在特定光照下不可靠
4. **角度识别**: rz=45° 固定，不做物品角度调整（绿色边框+阴影干扰）
5. **物品需摆正**: 若物品严重歪斜，抓取可能失败

---

## 9. 文件清单

| 文件 | 用途 | 移植必需 |
|------|------|---------|
| `motion_service.py` | 机械臂运动核心 | ✅ |
| `camera_vision.py` | 相机封装 | ✅ |
| `path_sorting_area_rz45.json` | 蛇形扫描路径 | ✅ |
| `marker_map.json` | 目标区域坐标 | ✅ 需按实际修改 |
| `grasp_offset.json` | 夹爪-相机偏移 | ✅ 需重新标定 |
| `ROBOT_NOTES.md` | 开发记录/校准日志 | 推荐保留 |

---

## 10. 移植检查清单

- [ ] 修改串口地址 (`/dev/cu.usbserial-120`)
- [ ] 确认相机 index (Camera[0])
- [ ] 测量并更新夹爪最大开度
- [ ] 测量物品尺寸，调整抓取高度 Z
- [ ] **重新标定夹爪-相机偏移**（方法：相机对准物品→夹取测试→记录偏移量）
- [ ] 根据实际桌面布局更新 `marker_map.json`
- [ ] 调整扫描路径范围（`path_sorting_area_rz45.json`）
- [ ] 在 `motion_service.py` 中确认/调整安全边界

---

*最后验证: 2026-05-11 22:48，两轮分拣 100% 成功*
