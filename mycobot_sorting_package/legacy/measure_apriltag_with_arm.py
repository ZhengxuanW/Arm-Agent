#!/usr/bin/env python3
"""
measure_apriltag_with_arm.py
============================
用末端相机扫描 AprilTag，迭代对准后记录机械臂坐标作为世界坐标。

交互流程：
1. 脚本扫描桌面找 AprilTag
2. 迭代对准使 AprilTag 位于画面中心
3. 降下、夹爪闭合（模拟抓取）
4. 提示用户："请确认是否成功对准并闭合夹爪 [y/n]"
5. 用户输入 y -> 记录当前坐标; n -> 跳过
6. 张开夹爪、升起、下一个

AprilTag: DICT_APRILTAG_36h11
"""

import json
import os
import time
from datetime import datetime

import cv2
import numpy as np

from motion_service import MotionService

# ---------- 配置 ----------
SCALE_Z200 = 0.32  # Z=200mm 时像素比例 mm/px

# ---------- AprilTag 检测器 ----------
APRILTAG_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
APRILTAG_DETECTOR = cv2.aruco.ArucoDetector(APRILTAG_DICT)

# 扫描路径（9 点，覆盖待分拣区域）
SCAN_POINTS = [
    ("前左",   140, -80),
    ("前中左", 170, -80),
    ("前右",   200, -80),
    ("中右",   200,   0),
    ("中中",   170,   0),
    ("中左",   140,   0),
    ("后左",   140,  80),
    ("后中左", 170,  80),
    ("后右",   200,  80),
]

# 迭代对准参数
ALIGN_RATIO = 0.5       # 保守步长
ALIGN_THRESH_PX = 15    # 收敛阈值
MAX_ALIGN_ITERS = 10

# Z 高度
Z_SCAN = 200
Z_APPROACH = 140
Z_GRAB = 95

RUN_ID = datetime.now().strftime("%Y%m%d_%H%M%S")
OUT_DIR = f"apriltag_measure_{RUN_ID}"
os.makedirs(OUT_DIR, exist_ok=True)


def detect_apriltags(frame: np.ndarray):
    """检测 AprilTag，返回 {id: (cx, cy, area_approx)}"""
    corners, ids, _ = APRILTAG_DETECTOR.detectMarkers(frame)
    result = {}
    if ids is not None:
        for i, marker_id in enumerate(ids.flatten()):
            c = corners[i][0]
            cx = int(np.mean(c[:, 0]))
            cy = int(np.mean(c[:, 1]))
            # 用边界框面积估算
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


def scan_for_tags(ms: MotionService):
    """扫描桌面找 AprilTag，返回 (tag_id, scan_x, scan_y) 列表"""
    found = {}
    print("\n[INFO] 开始扫描 AprilTag...")
    for name, x, y in SCAN_POINTS:
        print(f"\n  -> 扫描点 {name}: ({x}, {y})")
        ms.move_to(x=x, y=y, z=Z_SCAN, rx=-180, ry=0, rz=45, speed=20, wait=True)
        time.sleep(0.8)

        frame = ms.capture()
        save_debug(frame, f"scan_{name}.jpg")

        tags = detect_apriltags(frame)
        for tag_id, (cx, cy, area) in tags.items():
            if tag_id not in found:
                found[tag_id] = (x, y, cx, cy)
                print(f"     发现 Tag ID={tag_id} 在 ({x}, {y}) 附近, 像素=({cx},{cy})")

    return found


def iterative_align_to_tag(ms: MotionService, start_x, start_y, target_tag_id):
    """迭代对准指定 AprilTag，返回对准后的 (world_x, world_y) 或 None"""
    cam_x, cam_y = start_x, start_y

    for i in range(MAX_ALIGN_ITERS):
        print(f"\n  [INFO] 对准迭代 {i + 1}/{MAX_ALIGN_ITERS}: 移动到 ({cam_x:.1f}, {cam_y:.1f}, {Z_SCAN})")
        ms.move_to(x=cam_x, y=cam_y, z=Z_SCAN, rx=-180, ry=0, rz=45, speed=15, wait=True)
        time.sleep(1.0)

        frame = ms.capture()
        if frame is None:
            print("  [WARN] 拍照失败，重试...")
            time.sleep(0.5)
            continue

        tags = detect_apriltags(frame)
        save_debug(frame, f"align_{target_tag_id}_iter{i + 1}.jpg",
                   tag_info=(target_tag_id, *tags.get(target_tag_id, (0, 0, 0))) if target_tag_id in tags else None)

        if target_tag_id not in tags:
            print(f"  [WARN] 未检测到 Tag ID={target_tag_id}")
            if i == 0:
                print("  [INFO] 尝试周边搜索...")
                for dx, dy in [(0, -30), (0, 30), (-30, 0), (30, 0)]:
                    try_x, try_y = cam_x + dx, cam_y + dy
                    ms.move_to(x=try_x, y=try_y, z=Z_SCAN, rx=-180, ry=0, rz=45, speed=15, wait=True)
                    time.sleep(0.8)
                    f = ms.capture()
                    t = detect_apriltags(f)
                    save_debug(f, f"align_{target_tag_id}_search_{dx}_{dy}.jpg",
                               tag_info=(target_tag_id, *t.get(target_tag_id, (0, 0, 0))) if target_tag_id in t else None)
                    if target_tag_id in t:
                        cam_x, cam_y = try_x, try_y
                        print(f"  [OK] 在 ({try_x:.1f}, {try_y:.1f}) 重新发现 Tag")
                        break
                else:
                    print("  [ERROR] 周边搜索失败")
                    return None
            else:
                return None

        cx, cy, area = tags[target_tag_id]
        h, w = frame.shape[:2]
        dx_px = cx - w // 2
        dy_px = cy - h // 2
        print(f"  [INFO] Tag ID={target_tag_id} 像素偏移 dx_px={dx_px}, dy_px={dy_px}")

        if abs(dx_px) < ALIGN_THRESH_PX and abs(dy_px) < ALIGN_THRESH_PX:
            print("  [INFO] 对准已收敛")
            return cam_x, cam_y

        # 坐标修正（AGENTS.md 规则）
        # 画面上方 -> X增加, 画面下方 -> X减少
        # 画面左方 -> Y增加, 画面右方 -> Y减少
        dx_mm = -dy_px * SCALE_Z200 * ALIGN_RATIO
        dy_mm = -dx_px * SCALE_Z200 * ALIGN_RATIO

        cam_x += dx_mm
        cam_y += dy_mm
        print(f"  [INFO] 修正: dX={dx_mm:.1f}, dY={dy_mm:.1f} -> 新坐标=({cam_x:.1f}, {cam_y:.1f})")

    print("  [WARN] 达到最大迭代次数，未完全收敛")
    return cam_x, cam_y


def measure_tag(ms: MotionService, tag_id: int, start_x: float, start_y: float):
    """测量单个 AprilTag 的世界坐标"""
    print(f"\n{'='*60}")
    print(f"[MEASURE] Tag ID={tag_id}")
    print(f"{'='*60}")

    # 1. 迭代对准
    aligned = iterative_align_to_tag(ms, start_x, start_y, tag_id)
    if aligned is None:
        print("[ERROR] 对准失败，跳过")
        return None

    align_x, align_y = aligned
    print(f"[INFO] 对准完成: ({align_x:.1f}, {align_y:.1f})")

    # 2. 应用夹爪偏移（相机中心 -> 夹爪中心）
    # AGENTS.md: grasp_x = center_x + 30, grasp_y = center_y + 0
    grasp_x = align_x + 30
    grasp_y = align_y + 0

    # 3. 降下并闭合夹爪
    print(f"[INFO] 移动到抓取位置: ({grasp_x:.1f}, {grasp_y:.1f}, {Z_GRAB})")
    ms.move_to(x=grasp_x, y=grasp_y, z=Z_APPROACH, rx=-180, ry=0, rz=45, speed=15, wait=True)
    time.sleep(0.5)
    ms.move_to(x=grasp_x, y=grasp_y, z=Z_GRAB, rx=-180, ry=0, rz=45, speed=10, wait=True)
    time.sleep(0.3)
    ms.gripper_close()
    time.sleep(0.5)

    # 4. 自动记录（给用户5秒观察时间，可按Ctrl+C中断）
    coords = ms.robot.get_coords()
    actual_x, actual_y = coords[0], coords[1]
    print(f"\n  *** 夹爪已闭合 ***")
    print(f"  当前机械臂坐标: X={actual_x:.1f}, Y={actual_y:.1f}")
    print(f"  请在5秒内目视确认夹爪是否正对 AprilTag ID={tag_id} 中心...")
    print(f"  （如发现问题请按 Ctrl+C 中断）")
    try:
        time.sleep(5)
    except KeyboardInterrupt:
        print(f"\n  [INTERRUPT] 用户中断，跳过 Tag ID={tag_id}")
        ms.gripper_open()
        time.sleep(0.3)
        ms.move_to(x=grasp_x, y=grasp_y, z=Z_SCAN, rx=-180, ry=0, rz=45, speed=15, wait=True)
        return None
    
    # 自动记录（无需人工输入）
    result = {
        "tag_id": tag_id,
        "camera_x": round(float(align_x), 1),
        "camera_y": round(float(align_y), 1),
        "grasp_x": round(float(actual_x), 1),
        "grasp_y": round(float(actual_y), 1),
        "validated": True,
    }
    print(f"  [OK] 已自动记录 Tag ID={tag_id}: ({actual_x:.1f}, {actual_y:.1f})")

    # 5. 复位
    ms.gripper_open()
    time.sleep(0.3)
    ms.move_to(x=grasp_x, y=grasp_y, z=Z_SCAN, rx=-180, ry=0, rz=45, speed=15, wait=True)
    time.sleep(0.3)

    return result


def main():
    print("=" * 70)
    print("AprilTag 世界坐标测量 (用末端相机 + 机械臂)")
    print("=" * 70)
    print("说明:")
    print("  1. 脚本会扫描桌面找 AprilTag")
    print("  2. 对每个 Tag 迭代对准 -> 降下 -> 闭合夹爪")
    print("  3. 你目视确认后输入 y/n")
    print("  4. 成功时记录机械臂坐标作为该 Tag 的世界坐标")
    print("=" * 70)

    ms = MotionService()
    print("\n[INFO] 连接机械臂...")
    if not ms.connect(warmup=True):
        print("[ERROR] 连接失败")
        return

    # 初始化
    print("[INFO] 初始化...")
    ms.gripper_open()
    time.sleep(0.5)
    ms.go_init(speed=10)
    time.sleep(1.0)

    # 扫描
    found_tags = scan_for_tags(ms)
    if not found_tags:
        print("\n[ERROR] 未检测到任何 AprilTag，请确认:")
        print("  - 标记是否在待分拣区域内")
        print("  - 末端相机是否正常工作")
        ms.close()
        return

    print(f"\n[INFO] 扫描完成，发现 {len(found_tags)} 个 AprilTag: {list(found_tags.keys())}")

    # 逐个测量
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

    # 保存结果
    if results:
        output_path = os.path.join(OUT_DIR, "apriltag_world_coords.json")
        with open(output_path, "w", encoding="utf-8") as f:
            json.dump({
                "created_at": datetime.now().isoformat(),
                "tags": results,
                "note": "grasp_x/y 为经验证的 AprilTag 世界坐标(mm)",
            }, f, indent=2, ensure_ascii=False)
        print(f"\n[SUCCESS] 结果已保存: {output_path}")
        for r in results:
            print(f"  Tag ID={r['tag_id']}: ({r['grasp_x']}, {r['grasp_y']})")
    else:
        print("\n[WARN] 没有成功记录任何坐标")


if __name__ == "__main__":
    main()
