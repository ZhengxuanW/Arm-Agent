#!/usr/bin/env python3
"""
global_vision_sort.py
=====================
全局摄像头粗定位 + 末端相机精扫描 + 抓取

流程:
1. 全局摄像头(BRIO)检测物品颜色，获取像素坐标
2. 用标定矩阵转换为世界坐标(粗略)
3. 机械臂移动到世界坐标上方
4. 末端相机精细扫描:
   - 第1轮: 3点紧凑扫描 (目标±20mm)
   - 第2轮: 6点扩大扫描 (目标±40mm)
5. 找到物品 → 迭代对准 → 抓取 → 放置
"""

import json
import logging
import os
import time
from datetime import datetime

import cv2
import numpy as np

from motion_service import MotionService

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
logger = logging.getLogger(__name__)

# ---------- 配置 ----------
Z_SCAN = 200
Z_APPROACH = 140
Z_GRAB = 95
Z_RELEASE = 105

SCALE_Z200 = 0.32
ALIGN_RATIO = 0.6
ALIGN_THRESH_PX = 20
MAX_ALIGN_ITERS = 8

# 紧凑扫描3点 (相对于目标位置)
TIGHT_SCAN = [(0, 0), (0, -25), (0, 25)]
# 扩大扫描6点
WIDE_SCAN = [
    (0, 0), (40, 0), (-40, 0),
    (0, 40), (0, -40), (40, 40), (-40, -40)
]

RUN_ID = datetime.now().strftime("%Y%m%d_%H%M%S")
OUT_DIR = f"global_sort_{RUN_ID}"
os.makedirs(OUT_DIR, exist_ok=True)

# ---------- 加载全局摄像头标定 ----------
def load_calibration(path="global_camera_calib.json"):
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    matrix = np.array(data["affine_matrix"], dtype=np.float32)
    return matrix

# ---------- 全局摄像头 ----------
class GlobalCamera:
    def __init__(self, index=1):
        self.index = index
        self._cap = None

    def open(self):
        self._cap = cv2.VideoCapture(self.index)
        if not self._cap.isOpened():
            return False
        for _ in range(3):
            self._cap.read()
        return True

    def grab(self, retries=5):
        if self._cap is None:
            return None
        best = None
        best_mean = 0
        for _ in range(retries):
            ret, frame = self._cap.read()
            if ret and frame is not None:
                m = np.mean(frame)
                if m > best_mean:
                    best_mean = m
                    best = frame
        return best

    def close(self):
        if self._cap:
            self._cap.release()

# ---------- 颜色检测 (全局摄像头) ----------
def find_color_global(frame, color_name):
    """在全局摄像头画面中检测指定颜色物品，返回 (cx, cy, area) 或 None"""
    if frame is None:
        return None
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    if color_name == "red":
        lower1, upper1 = np.array([0, 100, 80]), np.array([12, 255, 255])
        lower2, upper2 = np.array([160, 100, 80]), np.array([180, 255, 255])
        mask = cv2.inRange(hsv, lower1, upper1) | cv2.inRange(hsv, lower2, upper2)
    elif color_name == "blue":
        lower, upper = np.array([90, 100, 50]), np.array([130, 255, 255])
        mask = cv2.inRange(hsv, lower, upper)
    elif color_name == "green":
        lower, upper = np.array([35, 80, 50]), np.array([85, 255, 255])
        mask = cv2.inRange(hsv, lower, upper)
    elif color_name == "yellow":
        lower, upper = np.array([18, 120, 80]), np.array([35, 255, 255])
        mask = cv2.inRange(hsv, lower, upper)
    else:
        return None

    kernel = np.ones((7, 7), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # 限制在工作区域（全局画面中的待分拣区域）
    # 工作区域约: u=500~1200, v=300~800
    h, w = mask.shape
    roi_mask = np.zeros_like(mask)
    roi_mask[300:800, 500:1200] = mask[300:800, 500:1200]
    mask = roi_mask

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None

    # 选最大轮廓
    best = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(best)
    if area < 500:  # 全局画面中过滤小噪点
        return None

    M = cv2.moments(best)
    if M["m00"] == 0:
        return None
    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])
    return cx, cy, area


# ---------- 颜色检测 (末端相机) ----------
def find_color_local(frame, color_name):
    """在末端相机画面中检测指定颜色，返回 (cx, cy) 或 None"""
    if frame is None:
        return None
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    if color_name == "red":
        lower1, upper1 = np.array([0, 80, 80]), np.array([12, 255, 255])
        lower2, upper2 = np.array([160, 80, 80]), np.array([180, 255, 255])
        mask = cv2.inRange(hsv, lower1, upper1) | cv2.inRange(hsv, lower2, upper2)
    elif color_name == "blue":
        lower, upper = np.array([100, 150, 0]), np.array([140, 255, 255])
        mask = cv2.inRange(hsv, lower, upper)
    elif color_name == "green":
        lower, upper = np.array([40, 100, 100]), np.array([80, 255, 255])
        mask = cv2.inRange(hsv, lower, upper)
    elif color_name == "yellow":
        lower, upper = np.array([20, 100, 100]), np.array([35, 255, 255])
        mask = cv2.inRange(hsv, lower, upper)
    else:
        return None

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None

    candidates = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if 300 < area < 40000:
            M = cv2.moments(cnt)
            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                candidates.append((area, cx, cy))

    if not candidates:
        return None
    candidates.sort(reverse=True)
    _, cx, cy = candidates[0]
    return cx, cy


# ---------- 像素到世界坐标 ----------
def pixel_to_world(u, v, affine_matrix):
    pt = affine_matrix @ np.array([[u], [v], [1.0]])
    return float(pt[0, 0]), float(pt[1, 0])


# ---------- 保存调试图 ----------
def save_debug(frame, name, obj=None, color=(0, 255, 0)):
    vis = frame.copy()
    h, w = vis.shape[:2]
    cv2.drawMarker(vis, (w // 2, h // 2), (0, 0, 255), cv2.MARKER_CROSS, 30, 2)
    if obj:
        cx, cy = obj[:2]
        cv2.circle(vis, (cx, cy), 8, color, 2)
        info = f"({cx},{cy})"
        if len(obj) > 2:
            info += f" a={obj[2]}"
        cv2.putText(vis, info, (cx + 10, cy - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
    path = os.path.join(OUT_DIR, name)
    cv2.imwrite(path, vis)
    logger.info("已保存 %s", path)


# ---------- 精细扫描 ----------
def fine_scan(ms, target_x, target_y, color_name):
    """
    在目标位置附近精细扫描。
    第1轮: 3点紧凑扫描
    第2轮: 6点扩大扫描
    返回 (found_x, found_y) 或 None
    """
    logger.info("[FINE_SCAN] 目标位置: (%.1f, %.1f), 颜色=%s", target_x, target_y, color_name)

    # 第1轮: 紧凑扫描3点
    logger.info("  -> 第1轮: 紧凑扫描 (3点)")
    for dx, dy in TIGHT_SCAN:
        x, y = target_x + dx, target_y + dy
        logger.info("     扫描 (%.1f, %.1f)", x, y)
        ms.move_to(x=x, y=y, z=Z_SCAN, rx=-180, ry=0, rz=45, speed=20, wait=True)
        time.sleep(0.8)
        frame = ms.capture()
        obj = find_color_local(frame, color_name)
        save_debug(frame, f"fine1_{x:.0f}_{y:.0f}.jpg", obj)
        if obj:
            logger.info("     [OK] 第1轮发现物品!")
            return x, y, obj[0], obj[1]

    # 第2轮: 扩大扫描6点
    logger.info("  -> 第2轮: 扩大扫描 (6点)")
    for dx, dy in WIDE_SCAN:
        x, y = target_x + dx, target_y + dy
        logger.info("     扫描 (%.1f, %.1f)", x, y)
        ms.move_to(x=x, y=y, z=Z_SCAN, rx=-180, ry=0, rz=45, speed=20, wait=True)
        time.sleep(0.8)
        frame = ms.capture()
        obj = find_color_local(frame, color_name)
        save_debug(frame, f"fine2_{x:.0f}_{y:.0f}.jpg", obj)
        if obj:
            logger.info("     [OK] 第2轮发现物品!")
            return x, y, obj[0], obj[1]

    logger.warning("  [FAIL] 精细扫描未找到物品")
    return None


# ---------- 迭代对准 ----------
def iterative_align(ms, start_x, start_y, color_name):
    """迭代对准，返回 (camera_x, camera_y) 或 None"""
    cam_x, cam_y = start_x, start_y
    for i in range(MAX_ALIGN_ITERS):
        logger.info("  对准迭代 %d/%d: (%.1f, %.1f, %d)", i+1, MAX_ALIGN_ITERS, cam_x, cam_y, Z_SCAN)
        ms.move_to(x=cam_x, y=cam_y, z=Z_SCAN, rx=-180, ry=0, rz=45, speed=15, wait=True)
        time.sleep(1.0)

        frame = ms.capture()
        if frame is None:
            time.sleep(0.5)
            continue

        obj = find_color_local(frame, color_name)
        save_debug(frame, f"align_iter{i+1}.jpg", obj)

        if obj is None:
            logger.warning("    未检测到颜色")
            if i == 0:
                # 周边搜索
                for dx, dy in [(0, -30), (0, 30), (-30, 0), (30, 0)]:
                    try_x, try_y = cam_x + dx, cam_y + dy
                    ms.move_to(x=try_x, y=try_y, z=Z_SCAN, rx=-180, ry=0, rz=45, speed=15, wait=True)
                    time.sleep(0.8)
                    f = ms.capture()
                    o = find_color_local(f, color_name)
                    if o:
                        cam_x, cam_y = try_x, try_y
                        logger.info("    在 (%.1f, %.1f) 重新发现", try_x, try_y)
                        break
                else:
                    return None
            else:
                return None

        cx, cy = obj
        h, w = frame.shape[:2]
        dx_px = cx - w // 2
        dy_px = cy - h // 2
        logger.info("    偏移 dx=%d, dy=%d", dx_px, dy_px)

        if abs(dx_px) < ALIGN_THRESH_PX and abs(dy_px) < ALIGN_THRESH_PX:
            logger.info("    收敛!")
            return cam_x, cam_y

        dX = -dy_px * SCALE_Z200 * ALIGN_RATIO
        dY = -dx_px * SCALE_Z200 * ALIGN_RATIO
        cam_x += dX
        cam_y += dY
        logger.info("    修正 dX=%.1f, dY=%.1f -> (%.1f, %.1f)", dX, dY, cam_x, cam_y)

    return cam_x, cam_y


# ---------- 抓取与放置 ----------
def pick_and_place(ms, camera_x, camera_y, release_x, release_y):
    """抓取并放置"""
    grasp_x = camera_x + 30
    grasp_y = camera_y + 0

    logger.info("[PICK] 抓取位置: (%.1f, %.1f, %d)", grasp_x, grasp_y, Z_GRAB)

    # 悬停
    ms.move_to(x=grasp_x, y=grasp_y, z=Z_APPROACH, rx=-180, ry=0, rz=45, speed=15, wait=True)
    time.sleep(0.5)
    # 降下
    ms.move_to(x=grasp_x, y=grasp_y, z=Z_GRAB, rx=-180, ry=0, rz=45, speed=10, wait=True)
    time.sleep(0.3)
    # 闭合
    ms.gripper_close()
    time.sleep(0.5)
    # 抬升
    ms.move_to(x=grasp_x, y=grasp_y, z=Z_APPROACH, rx=-180, ry=0, rz=45, speed=15, wait=True)
    time.sleep(0.5)

    # 移动到释放区
    logger.info("[PLACE] 释放位置: (%.1f, %.1f, %d)", release_x, release_y, Z_RELEASE)
    ms.move_to(x=release_x, y=release_y, z=Z_APPROACH, rx=-180, ry=0, rz=45, speed=20, wait=True)
    time.sleep(0.5)
    ms.move_to(x=release_x, y=release_y, z=Z_RELEASE, rx=-180, ry=0, rz=45, speed=15, wait=True)
    time.sleep(0.5)
    ms.gripper_open()
    time.sleep(0.3)
    ms.move_to(x=release_x, y=release_y, z=Z_APPROACH, rx=-180, ry=0, rz=45, speed=20, wait=True)


# ---------- 主流程 ----------
def main():
    import argparse
    parser = argparse.ArgumentParser(description="全局摄像头粗定位 + 末端精扫描分拣")
    parser.add_argument("--color", required=True, choices=["red", "blue", "green", "yellow"],
                        help="目标物品颜色")
    parser.add_argument("--release-x", type=float, default=60, help="释放区X坐标")
    parser.add_argument("--release-y", type=float, default=230, help="释放区Y坐标")
    args = parser.parse_args()

    logger.info("=" * 70)
    logger.info("全局摄像头粗定位 + 末端精扫描分拣")
    logger.info("目标颜色: %s -> 释放区: (%.1f, %.1f)", args.color, args.release_x, args.release_y)
    logger.info("=" * 70)

    # 加载标定
    try:
        affine_matrix = load_calibration()
        logger.info("[OK] 已加载全局摄像头标定")
    except Exception as e:
        logger.error("加载标定失败: %s", e)
        return

    # 连接机械臂
    ms = MotionService()
    logger.info("[INFO] 连接机械臂...")
    if not ms.connect(warmup=True):
        logger.error("连接失败")
        return

    # 初始化
    ms.gripper_open()
    time.sleep(0.5)
    ms.go_init(speed=10)
    time.sleep(1.0)

    # 打开全局摄像头
    logger.info("[INFO] 打开全局摄像头 (BRIO)...")
    global_cam = GlobalCamera(index=1)
    if not global_cam.open():
        logger.error("全局摄像头打开失败")
        ms.close()
        return

    try:
        # 1. 全局摄像头检测
        logger.info("[STEP 1] 全局摄像头检测 %s 物品...", args.color)
        frame = global_cam.grab()
        if frame is None:
            logger.error("全局摄像头拍照失败")
            return

        # 保存全局图
        cv2.imwrite(os.path.join(OUT_DIR, "global_view.jpg"), frame)

        obj = find_color_global(frame, args.color)
        if obj is None:
            logger.error("全局摄像头未检测到 %s 物品", args.color)
            return

        cx, cy, area = obj
        logger.info("[OK] 全局检测到 %s: 像素=(%d, %d), 面积=%d", args.color, cx, cy, area)

        # 2. 转换为世界坐标
        world_x, world_y = pixel_to_world(cx, cy, affine_matrix)
        logger.info("[STEP 2] 世界坐标估算: (%.1f, %.1f)", world_x, world_y)

        # 3. 关闭全局摄像头
        global_cam.close()

        # 4. 机械臂移动到估算位置
        logger.info("[STEP 3] 机械臂移动到 (%.1f, %.1f, %d)...", world_x, world_y, Z_SCAN)
        ms.move_to(x=world_x, y=world_y, z=Z_SCAN, rx=-180, ry=0, rz=45, speed=20, wait=True)
        time.sleep(1.0)

        # 5. 末端相机精细扫描
        logger.info("[STEP 4] 末端相机精细扫描...")
        scan_result = fine_scan(ms, world_x, world_y, args.color)
        if scan_result is None:
            logger.error("[FAIL] 末端扫描未找到物品，任务终止")
            return

        scan_x, scan_y, _, _ = scan_result
        logger.info("[OK] 末端扫描发现物品在: (%.1f, %.1f)", scan_x, scan_y)

        # 6. 迭代对准
        logger.info("[STEP 5] 迭代对准...")
        aligned = iterative_align(ms, scan_x, scan_y, args.color)
        if aligned is None:
            logger.error("[FAIL] 对准失败，任务终止")
            return

        camera_x, camera_y = aligned
        logger.info("[OK] 对准完成: (%.1f, %.1f)", camera_x, camera_y)

        # 7. 抓取与放置
        logger.info("[STEP 6] 抓取并放置...")
        pick_and_place(ms, camera_x, camera_y, args.release_x, args.release_y)

        logger.info("[SUCCESS] 任务完成! %s 物品已放置到 (%.1f, %.1f)",
                    args.color, args.release_x, args.release_y)

    except Exception as e:
        logger.error("异常: %s", e, exc_info=True)

    finally:
        logger.info("[INFO] 回到零位...")
        ms.gripper_open()
        ms.go_init(speed=10)
        time.sleep(1.0)
        ms.close()
        global_cam.close()


if __name__ == "__main__":
    main()
