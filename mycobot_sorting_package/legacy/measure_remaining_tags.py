#!/usr/bin/env python3
"""
measure_remaining_tags.py
=========================
用扩展扫描路径测量剩余 AprilTag 的世界坐标。
"""

import json
import os
import time
from datetime import datetime

import cv2
import numpy as np

from motion_service import MotionService

# ---------- 配置 ----------
SCALE_Z200 = 0.32
ALIGN_RATIO = 0.5
ALIGN_THRESH_PX = 15
MAX_ALIGN_ITERS = 10
Z_SCAN = 200
Z_GRAB = 95

# 扩展扫描路径（覆盖更大范围）
EXTENDED_SCAN_POINTS = [
    # 原范围
    ("前左",   140, -80), ("前中左", 170, -80), ("前右",   200, -80),
    ("中右",   200,   0), ("中中",   170,   0), ("中左",   140,   0),
    ("后左",   140,  80), ("后中左", 170,  80), ("后右",   200,  80),
    # 扩展范围（X 更小，更靠近底座）
    ("深前左", 110, -80), ("深前中", 110,   0), ("深前右", 110,  80),
    ("深中左",  80, -80), ("深中中",  80,   0), ("深中右",  80,  80),
]

RUN_ID = datetime.now().strftime("%Y%m%d_%H%M%S")
OUT_DIR = f"apriltag_measure_{RUN_ID}"
os.makedirs(OUT_DIR, exist_ok=True)

# AprilTag 检测器
APRILTAG_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
APRILTAG_DETECTOR = cv2.aruco.ArucoDetector(APRILTAG_DICT)


def detect_apriltags(frame: np.ndarray):
    corners, ids, _ = APRILTAG_DETECTOR.detectMarkers(frame)
    result = {}
    if ids is not None:
        for i, marker_id in enumerate(ids.flatten()):
            c = corners[i][0]
            cx = int(np.mean(c[:, 0]))
            cy = int(np.mean(c[:, 1]))
            area = cv2.contourArea(np.array(c, dtype=np.int32))
            result[int(marker_id)] = (cx, cy, area)
    return result


def save_debug(frame, name, tag_info=None):
    vis = frame.copy()
    h, w = vis.shape[:2]
    cv2.drawMarker(vis, (w // 2, h // 2), (0, 0, 255), cv2.MARKER_CROSS, 30, 2)
    if tag_info:
        tag_id, cx, cy, area = tag_info
        cv2.circle(vis, (cx, cy), 8, (0, 255, 0), 2)
        cv2.putText(vis, f"ID={tag_id} ({cx},{cy})", (cx + 10, cy - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    path = os.path.join(OUT_DIR, name)
    cv2.imwrite(path, vis)
    print(f"  [DEBUG] 已保存 {path}")


def iterative_align(ms, start_x, start_y, target_tag_id):
    cam_x, cam_y = start_x, start_y
    for i in range(MAX_ALIGN_ITERS):
        print(f"\n  [INFO] 对准迭代 {i + 1}/{MAX_ALIGN_ITERS}: ({cam_x:.1f}, {cam_y:.1f}, {Z_SCAN})")
        ms.move_to(x=cam_x, y=cam_y, z=Z_SCAN, rx=-180, ry=0, rz=45, speed=15, wait=True)
        time.sleep(1.0)

        frame = ms.capture()
        if frame is None:
            print("  [WARN] 拍照失败")
            time.sleep(0.5)
            continue

        tags = detect_apriltags(frame)
        save_debug(frame, f"align_{target_tag_id}_iter{i + 1}.jpg",
                   tag_info=(target_tag_id, *tags.get(target_tag_id, (0, 0, 0))) if target_tag_id in tags else None)

        if target_tag_id not in tags:
            print(f"  [WARN] 未检测到 Tag ID={target_tag_id}")
            if i == 0:
                for dx, dy in [(0, -30), (0, 30), (-30, 0), (30, 0)]:
                    try_x, try_y = cam_x + dx, cam_y + dy
                    ms.move_to(x=try_x, y=try_y, z=Z_SCAN, rx=-180, ry=0, rz=45, speed=15, wait=True)
                    time.sleep(0.8)
                    f = ms.capture()
                    t = detect_apriltags(f)
                    if target_tag_id in t:
                        cam_x, cam_y = try_x, try_y
                        print(f"  [OK] 在 ({try_x:.1f}, {try_y:.1f}) 重新发现")
                        break
                else:
                    return None
            else:
                return None

        cx, cy, area = tags[target_tag_id]
        h, w = frame.shape[:2]
        dx_px = cx - w // 2
        dy_px = cy - h // 2
        print(f"  [INFO] 偏移 dx_px={dx_px}, dy_px={dy_px}")

        if abs(dx_px) < ALIGN_THRESH_PX and abs(dy_px) < ALIGN_THRESH_PX:
            print("  [INFO] 收敛")
            return cam_x, cam_y

        dx_mm = -dy_px * SCALE_Z200 * ALIGN_RATIO
        dy_mm = -dx_px * SCALE_Z200 * ALIGN_RATIO
        cam_x += dx_mm
        cam_y += dy_mm
        print(f"  [INFO] 修正: dX={dx_mm:.1f}, dY={dy_mm:.1f} -> ({cam_x:.1f}, {cam_y:.1f})")

    return cam_x, cam_y


def scan_for_tags(ms):
    found = {}
    print("\n[INFO] 开始扩展扫描...")
    for name, x, y in EXTENDED_SCAN_POINTS:
        print(f"\n  -> {name}: ({x}, {y})")
        ms.move_to(x=x, y=y, z=Z_SCAN, rx=-180, ry=0, rz=45, speed=20, wait=True)
        time.sleep(0.8)
        frame = ms.capture()
        save_debug(frame, f"scan_{name}.jpg")
        tags = detect_apriltags(frame)
        for tag_id, (cx, cy, area) in tags.items():
            if tag_id not in found:
                found[tag_id] = (x, y, cx, cy)
                print(f"     发现 Tag ID={tag_id}")
    return found


def measure_tag(ms, tag_id, start_x, start_y):
    print(f"\n{'='*60}\n[MEASURE] Tag ID={tag_id}\n{'='*60}")
    
    aligned = iterative_align(ms, start_x, start_y, tag_id)
    if aligned is None:
        return None
    
    align_x, align_y = aligned
    grasp_x = align_x + 30
    grasp_y = align_y + 0
    
    print(f"[INFO] 抓取位置: ({grasp_x:.1f}, {grasp_y:.1f}, {Z_GRAB})")
    ms.move_to(x=grasp_x, y=grasp_y, z=140, rx=-180, ry=0, rz=45, speed=15, wait=True)
    time.sleep(0.5)
    ms.move_to(x=grasp_x, y=grasp_y, z=Z_GRAB, rx=-180, ry=0, rz=45, speed=10, wait=True)
    time.sleep(0.3)
    ms.gripper_close()
    time.sleep(0.5)
    
    coords = ms.robot.get_coords()
    actual_x, actual_y = coords[0], coords[1]
    print(f"\n  夹爪已闭合，坐标: ({actual_x:.1f}, {actual_y:.1f})")
    print(f"  5秒观察时间...")
    time.sleep(5)
    
    result = {
        "tag_id": tag_id,
        "camera_x": round(float(align_x), 1),
        "camera_y": round(float(align_y), 1),
        "grasp_x": round(float(actual_x), 1),
        "grasp_y": round(float(actual_y), 1),
        "validated": True,
    }
    print(f"  [OK] 记录 Tag ID={tag_id}: ({actual_x:.1f}, {actual_y:.1f})")
    
    ms.gripper_open()
    time.sleep(0.3)
    ms.move_to(x=grasp_x, y=grasp_y, z=Z_SCAN, rx=-180, ry=0, rz=45, speed=15, wait=True)
    time.sleep(0.3)
    return result


def main():
    print("=" * 70)
    print("AprilTag 扩展扫描测量")
    print("=" * 70)

    ms = MotionService()
    print("\n[INFO] 连接机械臂...")
    if not ms.connect(warmup=True):
        print("[ERROR] 连接失败")
        return

    ms.gripper_open()
    time.sleep(0.5)
    ms.go_init(speed=10)
    time.sleep(1.0)

    # 扫描
    found_tags = scan_for_tags(ms)
    print(f"\n[INFO] 发现 {len(found_tags)} 个 Tag: {list(found_tags.keys())}")

    if not found_tags:
        ms.close()
        return

    # 测量（跳过已测过的，或者全部重测）
    results = []
    for tag_id in sorted(found_tags.keys()):
        scan_x, scan_y, _, _ = found_tags[tag_id]
        result = measure_tag(ms, tag_id, scan_x, scan_y)
        if result:
            results.append(result)

    # 收尾
    print("\n[INFO] 回到零位...")
    ms.gripper_open()
    ms.go_init(speed=10)
    time.sleep(1.0)
    ms.close()

    # 保存
    if results:
        # 合并之前的结果
        prev_files = sorted([f for f in os.listdir('.') if f.startswith('apriltag_measure_') and f.endswith('.json')], reverse=True)
        all_results = results
        
        output_path = os.path.join(OUT_DIR, "apriltag_world_coords.json")
        with open(output_path, "w", encoding="utf-8") as f:
            json.dump({
                "created_at": datetime.now().isoformat(),
                "tags": all_results,
            }, f, indent=2, ensure_ascii=False)
        print(f"\n[SUCCESS] 结果: {output_path}")
        for r in all_results:
            print(f"  Tag ID={r['tag_id']}: ({r['grasp_x']}, {r['grasp_y']})")


if __name__ == "__main__":
    main()
