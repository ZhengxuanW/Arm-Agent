#!/usr/bin/env python3
"""
pickup_and_place.py
===================
R区 -> B区 单方块抓取放置脚本。

流程:
1. 连接并初始化 (gripper_open + go_init)
2. 移动到 R区上方扫描并迭代居中对准
3. 应用 +30mm X 偏移 -> 抓取
4. 移动到 B区 -> 放置
5. 回零并断开
"""

from __future__ import annotations

import json
import logging
import os
import sys
import time
from datetime import datetime

import cv2
import numpy as np

# 将项目目录加入路径
PROJECT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, PROJECT_DIR)

from motion_service import MotionService

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
)
logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# 配置参数
# ---------------------------------------------------------------------------

# 串口（AGENTS.md 中更新为 /dev/cu.usbserial-1120）
PORT = "/dev/cu.usbserial-1120"
BAUD = 1000000

# 视角固定
RX, RY, RZ = -180.0, 0.0, 45.0

# R区起始扫描坐标（方块大概位置）
R_SCAN_X, R_SCAN_Y = 60.0, 230.0

# B区目标坐标
B_CAMERA_X, B_CAMERA_Y = -90.0, 230.0   # 相机对准坐标
B_RELEASE_X, B_RELEASE_Y = -60.0, 230.0  # 实际释放坐标 (+30mm X)

# 高度
Z_SCAN = 200
Z_HOVER = 140
Z_GRASP = 95
Z_PLACE = 105

# 速度
SPEED_INIT = 10
SPEED_SCAN = 20
SPEED_APPROACH = 15
SPEED_GRASP_DOWN = 10
SPEED_PLACE_DOWN = 15

# 像素比例 (Z=200)
SCALE_MM_PER_PX = 0.32

# 迭代对准参数
MAX_ITER = 5
TOL_PX = 20
CORRECTION_RATIO = 0.6

# 红色/橙色 HSV 范围（合并 red + red2 + orange）
HSV_RED_LOWER1 = np.array([0, 120, 70])
HSV_RED_UPPER1 = np.array([10, 255, 255])
HSV_RED_LOWER2 = np.array([170, 120, 70])
HSV_RED_UPPER2 = np.array([180, 255, 255])
HSV_ORANGE_LOWER = np.array([10, 100, 100])
HSV_ORANGE_UPPER = np.array([25, 255, 255])

# 最小/最大检测面积
MIN_AREA = 500
MAX_AREA = 100000

# ---------------------------------------------------------------------------
# 视觉检测
# ---------------------------------------------------------------------------

def detect_red_orange_center(frame: np.ndarray) -> tuple | None:
    """
    在图像中检测红色/橙色方块的像素中心 (cx, cy)。
    返回 None 表示未检测到。
    """
    if frame is None:
        return None

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # 构建掩码: red1 + red2 + orange
    mask1 = cv2.inRange(hsv, HSV_RED_LOWER1, HSV_RED_UPPER1)
    mask2 = cv2.inRange(hsv, HSV_RED_LOWER2, HSV_RED_UPPER2)
    mask3 = cv2.inRange(hsv, HSV_ORANGE_LOWER, HSV_ORANGE_UPPER)
    mask = cv2.bitwise_or(mask1, mask2)
    mask = cv2.bitwise_or(mask, mask3)

    # 形态学去噪
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # 找轮廓
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None

    # 选最大轮廓
    largest = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(largest)
    if not (MIN_AREA <= area <= MAX_AREA):
        return None

    M = cv2.moments(largest)
    if M["m00"] == 0:
        return None

    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])

    return cx, cy


def save_debug_image(frame: np.ndarray, name: str, cx: int = None, cy: int = None):
    """保存调试用图像，可选标记中心点。"""
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"debug_{name}_{timestamp}.jpg"
    filepath = os.path.join(PROJECT_DIR, filename)
    vis = frame.copy()
    if cx is not None and cy is not None:
        cv2.drawMarker(vis, (cx, cy), (0, 255, 0), cv2.MARKER_CROSS, 20, 2)
        h, w = vis.shape[:2]
        cv2.drawMarker(vis, (w // 2, h // 2), (0, 0, 255), cv2.MARKER_CROSS, 20, 2)
    cv2.imwrite(filepath, vis)
    logger.info("Saved debug image: %s", filepath)
    return filepath


# ---------------------------------------------------------------------------
# 迭代居中对准
# ---------------------------------------------------------------------------

def iterative_center(ms: MotionService, start_x: float, start_y: float) -> tuple[float, float]:
    """
    迭代微调使红色/橙色方块进入画面中心。
    返回最终相机对准坐标 (center_x, center_y)。
    """
    x, y = start_x, start_y
    h, w = 480, 640  # 相机分辨率

    for i in range(MAX_ITER):
        logger.info("[对准迭代 %d/%d] 移动到 (%.1f, %.1f, %d)", i + 1, MAX_ITER, x, y, Z_SCAN)
        ms.move_to(x=x, y=y, z=Z_SCAN, rx=RX, ry=RY, rz=RZ, speed=SPEED_APPROACH)
        time.sleep(2.5)

        frame = ms.capture()
        if frame is None:
            logger.warning("[对准迭代 %d] 拍照失败，跳过本次", i + 1)
            continue

        center = detect_red_orange_center(frame)
        if center is None:
            logger.warning("[对准迭代 %d] 未检测到红色/橙色方块", i + 1)
            save_debug_image(frame, f"iter{i+1}_nodetect")
            continue

        cx, cy = center
        dx = cx - w // 2
        dy = cy - h // 2

        logger.info("[对准迭代 %d] 像素中心=(%d,%d), 画面中心=(%d,%d), dx=%d, dy=%d",
                    i + 1, cx, cy, w // 2, h // 2, dx, dy)
        save_debug_image(frame, f"iter{i+1}_dx{dx}_dy{dy}", cx, cy)

        if abs(dx) < TOL_PX and abs(dy) < TOL_PX:
            logger.info("[对准迭代 %d] 已收敛！|dx|=%d, |dy|=%d < %d", i + 1, abs(dx), abs(dy), TOL_PX)
            return x, y

        # 坐标修正: delta_X = -dy * scale * 0.6, delta_Y = -dx * scale * 0.6
        # 画面上方 -> X+, 画面左方 -> Y+
        delta_x = -dy * SCALE_MM_PER_PX * CORRECTION_RATIO
        delta_y = -dx * SCALE_MM_PER_PX * CORRECTION_RATIO

        x += delta_x
        y += delta_y

        logger.info("[对准迭代 %d] 修正: delta_x=%.2f, delta_y=%.2f -> 新坐标=(%.1f, %.1f)",
                    i + 1, delta_x, delta_y, x, y)

    logger.info("[对准迭代] 达到最大迭代次数，返回最佳估计坐标=(%.1f, %.1f)", x, y)
    return x, y


# ---------------------------------------------------------------------------
# 主流程
# ---------------------------------------------------------------------------

def main():
    logger.info("=" * 60)
    logger.info("R区 -> B区 单方块抓取放置任务开始")
    logger.info("=" * 60)

    ms = MotionService(port=PORT, baud=BAUD)

    # ------------------------------------------------------------------
    # 步骤 1: 连接并初始化
    # ------------------------------------------------------------------
    logger.info("[步骤1] 连接机械臂和相机...")
    if not ms.connect():
        logger.error("连接失败，退出")
        return 1

    logger.info("[步骤1] 打开夹爪并回零...")
    ms.gripper_open()
    time.sleep(0.5)
    ms.go_init(speed=SPEED_INIT)
    time.sleep(1.0)
    logger.info("[步骤1] 初始化完成")

    # ------------------------------------------------------------------
    # 步骤 2: 移动到 R 区上方并迭代居中对准
    # ------------------------------------------------------------------
    logger.info("[步骤2] 移动到 R 区上方扫描位置 (%.1f, %.1f, %d)...", R_SCAN_X, R_SCAN_Y, Z_SCAN)
    ms.move_to(x=R_SCAN_X, y=R_SCAN_Y, z=Z_SCAN, rx=RX, ry=RY, rz=RZ, speed=SPEED_SCAN)
    time.sleep(2.0)

    # 拍一张初始照片确认
    frame_init = ms.capture()
    if frame_init is not None:
        save_debug_image(frame_init, "r_zone_initial")

    # 迭代居中对准
    logger.info("[步骤2] 开始迭代居中对准...")
    center_x, center_y = iterative_center(ms, R_SCAN_X, R_SCAN_Y)
    logger.info("[步骤2] 对准完成: 相机中心坐标=(%.1f, %.1f)", center_x, center_y)

    # 应用 +30mm X 抓取偏移
    grasp_x = center_x + 30.0
    grasp_y = center_y + 0.0
    logger.info("[步骤2] 抓取坐标: grasp_x=%.1f, grasp_y=%.1f (应用 +30mm X 偏移)", grasp_x, grasp_y)

    # ------------------------------------------------------------------
    # 步骤 3: 抓取方块
    # ------------------------------------------------------------------
    logger.info("[步骤3] 开始抓取...")

    # 悬停
    logger.info("[步骤3a] 悬停至 z=%d...", Z_HOVER)
    ms.move_to(x=grasp_x, y=grasp_y, z=Z_HOVER, rx=RX, ry=RY, rz=RZ, speed=SPEED_APPROACH)
    time.sleep(2.0)

    # 降下
    logger.info("[步骤3b] 降下至抓取高度 z=%d...", Z_GRASP)
    ms.move_to(x=grasp_x, y=grasp_y, z=Z_GRASP, rx=RX, ry=RY, rz=RZ, speed=SPEED_GRASP_DOWN)
    time.sleep(1.5)

    # 夹取
    logger.info("[步骤3c] 闭合夹爪...")
    ms.gripper_close()
    time.sleep(1.0)

    # 抬升
    logger.info("[步骤3d] 抬升至 z=%d...", Z_HOVER)
    ms.move_to(x=grasp_x, y=grasp_y, z=Z_HOVER, rx=RX, ry=RY, rz=RZ, speed=SPEED_APPROACH)
    time.sleep(1.5)
    logger.info("[步骤3] 抓取完成")

    # ------------------------------------------------------------------
    # 步骤 4: 移动到 B 区放置
    # ------------------------------------------------------------------
    logger.info("[步骤4] 移动到 B 区放置...")
    logger.info("[步骤4] B区相机对准坐标=(%.1f, %.1f), 实际释放坐标=(%.1f, %.1f)",
                B_CAMERA_X, B_CAMERA_Y, B_RELEASE_X, B_RELEASE_Y)

    # 移动到 B 区上方
    logger.info("[步骤4a] 移动到 B 区上方 (%.1f, %.1f, %d)...", B_RELEASE_X, B_RELEASE_Y, Z_HOVER)
    ms.move_to(x=B_RELEASE_X, y=B_RELEASE_Y, z=Z_HOVER, rx=RX, ry=RY, rz=RZ, speed=SPEED_SCAN)
    time.sleep(2.5)

    # 降下放置
    logger.info("[步骤4b] 降下至放置高度 z=%d...", Z_PLACE)
    ms.move_to(x=B_RELEASE_X, y=B_RELEASE_Y, z=Z_PLACE, rx=RX, ry=RY, rz=RZ, speed=SPEED_PLACE_DOWN)
    time.sleep(1.5)

    # 释放
    logger.info("[步骤4c] 打开夹爪...")
    ms.gripper_open()
    time.sleep(0.5)

    # 抬升
    logger.info("[步骤4d] 抬升至 z=%d...", Z_HOVER)
    ms.move_to(x=B_RELEASE_X, y=B_RELEASE_Y, z=Z_HOVER, rx=RX, ry=RY, rz=RZ, speed=SPEED_SCAN)
    time.sleep(1.5)
    logger.info("[步骤4] 放置完成")

    # ------------------------------------------------------------------
    # 步骤 5: 收尾
    # ------------------------------------------------------------------
    logger.info("[步骤5] 回零并断开连接...")
    ms.go_init(speed=SPEED_INIT)
    time.sleep(2.0)
    ms.close()

    # ------------------------------------------------------------------
    # 结果报告
    # ------------------------------------------------------------------
    logger.info("=" * 60)
    logger.info("任务完成!")
    logger.info("方块初始大概位置: (%.1f, %.1f, %.1f)", R_SCAN_X, R_SCAN_Y, 105.0)
    logger.info("迭代对准后相机坐标: (%.1f, %.1f)", center_x, center_y)
    logger.info("实际抓取坐标: (%.1f, %.1f, %.1f)", grasp_x, grasp_y, Z_GRASP)
    logger.info("B区放置坐标: (%.1f, %.1f, %.1f)", B_RELEASE_X, B_RELEASE_Y, Z_PLACE)
    logger.info("=" * 60)

    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        logger.info("用户中断")
        sys.exit(1)
    except Exception as e:
        logger.exception("发生异常: %s", e)
        sys.exit(1)
