# AGENTS.md: myCobot 280 桌面垃圾分拣

本文件为 Agent 专用指南，记录项目结构、关键约定和操作速查。

## 1. 项目概述

基于 myCobot 280 + 末端 USB 相机的桌面垃圾分拣系统。
已实现端到端流程：扫描 -> 识别 -> 对准 -> 抓取 -> 放置。
两轮实测 100% 成功 (2026-05-11)。

## 2. 快速启动

依赖: pymycobot, opencv-python, numpy

单轮分拣最简流程:
1. ms = MotionService(); ms.connect()
2. ms.gripper_open(); ms.go_init(speed=10)
3. 执行蛇形扫描 (path_sorting_area_rz45.json)
4. 人工识别照片 -> 确定物品类型和坐标
5. 迭代居中对准 (见 SKILL.md)
6. 应用 +30mm X 偏移 -> 抓取
7. 放置到目标区 (marker_map.json)
8. ms.go_init()

## 3. 项目结构

- AGENTS.md: 本文件
- SKILL.md: 完整技能文档/移植指南
- ROBOT_NOTES.md: 开发记录/校准日志
- grasp_offset.json: 夹爪-相机偏移
- marker_map.json: 目标区域坐标
- motion_service.py: 核心运动服务
- camera_vision.py: 相机封装
- vision_detector.py: 颜色检测(不稳定)
- path_sorting_area_rz45.json: 20路点扫描路径
- auto_center.py: 迭代居中对准脚本
- run_YYYYMMDD_HHMMSS/: 运行照片记录

## 4. 关键约定

### 4.1 安全规则 (必须遵守)
1. 每次运行前: gripper_open() -> go_init()
2. 先 connect() 才能操作
3. 奇异点 WARN 可忽略 (J5 ~ 1deg)

### 4.2 坐标系速查
画面上方 -> X增加, 画面下方 -> X减少
画面左方 -> Y增加, 画面右方 -> Y减少

### 4.3 高度速查
Z=200: 扫描检测, Z=140: 悬停过渡
Z=95: 抓取(已验证), Z=105: 放置

### 4.4 夹爪偏移 (黄金法则)
grasp_x = center_x + 30
grasp_y = center_y + 0
仅在 Z=95 验证有效。

### 4.5 迭代对准经验
每次只走计算偏移的 50-70%，避免过度修正。
收敛条件: |dx_px| < 20 且 |dy_px| < 20

### 4.6 速度配置
扫描: speed=20, 对准: speed=15, 抓取降下: speed=10
go_init: speed=10

## 5. 目标区域坐标

### Zone 1（颜色标记区，Y正方向，已验证）
B(蓝色标记): (-90,  230, 105)
G(绿色标记): (-30,  230, 105)
R(红色标记): ( 30,  230, 105)
Y(黄色标记): ( 90,  230, 105)

### Zone 2（垃圾桶区，Y负方向，已验证）
蓝色(可回收): (-90, -205, 105) → 释放: (-60, -205)
绿色(厨余):   (-30, -205, 105) → 释放: (0,   -205)
红色(有害):   ( 30, -205, 105) → 释放: (60,  -205)
灰色(其他):   ( 90, -205, 105) → 释放: (120, -205)

### ⚠️ 放置偏移（2026-05-11 实测）
以上坐标是**相机对准坐标**（标记在画面中心）。
放置时需要 **+30mm X 偏移**才能将物品放到标记正上方：
```python
release_x = target_x + 30   # 放置偏移
release_y = target_y + 0
```
例如：R 区相机坐标 (30, 230) → 实际释放坐标 (60, 230)

## 6. 物品识别映射

核心原则：物品颜色 -> 对应颜色标记区（Zone 1）

黄色 -> 黄色标记区 Y (90, 230)
蓝色 -> 蓝色标记区 B (-90, 230)
绿色 -> 绿色标记区 G (-30, 230)
红色 -> 红色标记区 R (30, 230)

用户可手动指定放 Zone 2 或其他区域。

## 7. 移植到新环境时需修改

1. 串口地址 (/dev/cu.usbserial-120)
2. 相机 index (Camera[0])
3. 重新标定夹爪-相机偏移
4. 更新 marker_map.json 坐标
5. 调整扫描路径范围
6. 测量并更新抓取高度 Z

## 8. 相关文件速查

| 文件 | 用途 |
|------|------|
| SKILL.md | 完整文档、SOP、移植指南 |
| ROBOT_NOTES.md | 校准记录、运行日志 |
| motion_service.py | 核心服务，所有动作入口 |
| marker_map.json | 放置目标坐标 |
| path_sorting_area_rz45.json | 扫描路径定义 |

最后更新: 2026-05-11
