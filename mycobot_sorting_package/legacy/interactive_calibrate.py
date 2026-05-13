#!/usr/bin/env python3
"""
interactive_calibrate.py
=========================
交互式全局摄像头标定工具。

用法:
    python3 interactive_calibrate.py

界面说明:
    - 实时显示全局摄像头画面
    - 绿色方框 = 检测到的 AprilTag
    - 左上角 = 操作说明
    - 左下角 = 当前检测状态

按键:
    [SPACE] 空格键 - 立即标定: 记录当前 Tag 像素位置，重新计算标定矩阵
    [M]     测量模式 - 用机械臂重新测量三个 Tag 的世界坐标
    [S]     保存截图 - 保存当前画面到文件
    [R]     刷新显示 - 重新检测
    [Q/ESC] 退出

标定流程:
    1. 运行脚本，调整全局摄像头角度，确保三个 AprilTag 都在视野内
    2. 观察画面上三个 Tag 是否都被绿色方框标注
    3. 按下 [空格键]，程序自动:
       a. 记录三个 Tag 的像素坐标
       b. 使用已知的机械臂世界坐标（或重新测量）
       c. 计算新的仿射变换矩阵
       d. 保存到 global_camera_calib.json
    4. 屏幕上显示标定结果
"""

import json
import os
import time
from datetime import datetime

import cv2
import numpy as np

from motion_service import MotionService

# ---------- AprilTag 检测器 ----------
APRILTAG_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
APRILTAG_DETECTOR = cv2.aruco.ArucoDetector(APRILTAG_DICT)

# 已知的三个 Tag 世界坐标（固定位置，重标定时可复用）
# 如需更新，使用测量模式 (M键)
DEFAULT_WORLD_COORDS = {
    1: (140.0, 30.0),
    2: (10.0, 220.0),
    4: (60.0, -250.0),
}


def detect_apriltags(frame):
    """检测 AprilTag，返回 {id: (cx, cy, corners)}"""
    corners, ids, _ = APRILTAG_DETECTOR.detectMarkers(frame)
    result = {}
    if ids is not None:
        for i, marker_id in enumerate(ids.flatten()):
            c = corners[i][0]
            cx = int(np.mean(c[:, 0]))
            cy = int(np.mean(c[:, 1]))
            result[int(marker_id)] = (cx, cy, corners[i])
    return result


def draw_info_panel(frame, tags_detected, status_text, color=(0, 255, 0)):
    """在画面上绘制信息面板"""
    h, w = frame.shape[:2]
    overlay = frame.copy()

    # 顶部说明栏 (半透明黑色背景)
    cv2.rectangle(overlay, (0, 0), (w, 110), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.7, frame, 0.3, 0, frame)

    # 标题
    cv2.putText(frame, "Interactive Global Camera Calibration", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

    # 操作说明
    cv2.putText(frame, "[SPACE] Calibrate  [M] Measure World Coords  [S] Save Screenshot  [Q] Quit",
                (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)

    # 状态栏
    cv2.putText(frame, f"Detected Tags: {list(tags_detected.keys())} | Status: {status_text}",
                (10, 85), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

    # 底部状态栏
    cv2.rectangle(frame, (0, h - 40), (w, h), (0, 0, 0), -1)
    cv2.putText(frame, f"Resolution: {w}x{h} | Ready for calibration",
                (10, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1)

    return frame


def calibrate_from_current_frame(frame, world_coords):
    """
    从当前帧的 Tag 检测计算标定矩阵。
    返回: (success, message, affine_matrix or None)
    """
    tags = detect_apriltags(frame)

    if len(tags) < 3:
        missing = set(world_coords.keys()) - set(tags.keys())
        return False, f"需要3个Tag，但只检测到{len(tags)}个。缺少: {missing}", None

    # 提取像素坐标
    pixel_coords = {}
    for tag_id in sorted(tags.keys()):
        cx, cy, _ = tags[tag_id]
        pixel_coords[tag_id] = (cx, cy)

    # 构建对应点
    common_tags = sorted(set(pixel_coords.keys()) & set(world_coords.keys()))
    pixel_points = np.array([pixel_coords[t] for t in common_tags], dtype=np.float32)
    world_points = np.array([world_coords[t] for t in common_tags], dtype=np.float32)

    # 计算仿射变换
    affine_matrix = cv2.getAffineTransform(pixel_points, world_points)

    # 验证
    max_err = 0
    for tag_id in common_tags:
        u, v = pixel_coords[tag_id]
        pt = affine_matrix @ np.array([[u], [v], [1.0]])
        pred_x, pred_y = pt[0, 0], pt[1, 0]
        actual_x, actual_y = world_coords[tag_id]
        err = np.linalg.norm([pred_x - actual_x, pred_y - actual_y])
        max_err = max(max_err, err)

    return True, f"标定成功! {len(common_tags)}点, 最大误差={max_err:.2f}mm", affine_matrix


def save_calibration(affine_matrix, pixel_coords, world_coords):
    """保存标定结果到文件"""
    calib_data = {
        "created_at": datetime.now().isoformat(),
        "camera": {"index": 1, "name": "Logitech BRIO", "resolution": [1920, 1080]},
        "marker_type": "AprilTag_36h11",
        "num_points": len(pixel_coords),
        "pixel_points": {str(k): list(v) for k, v in pixel_coords.items()},
        "world_points": {str(k): list(v) for k, v in world_coords.items()},
        "affine_matrix": affine_matrix.tolist(),
        "usage": {
            "python": "pixel_to_world(u, v, affine_matrix)",
            "note": "输入(u,v)为BRIO像素坐标，输出(X,Y)为机械臂世界坐标(mm)",
        },
    }

    with open("global_camera_calib.json", "w", encoding="utf-8") as f:
        json.dump(calib_data, f, indent=2, ensure_ascii=False)


def measure_world_coords(ms):
    """
    用机械臂重新测量三个 Tag 的世界坐标。
    返回: {tag_id: (x, y)} 或 None
    """
    print("\n" + "=" * 60)
    print("测量模式: 用机械臂末端相机测量 Tag 世界坐标")
    print("=" * 60)

    # 这里简化处理：直接返回默认坐标
    # 如需完整测量，可以调用 measure_apriltag_with_arm.py 的逻辑
    print("使用预设的世界坐标:")
    for tag_id, (x, y) in DEFAULT_WORLD_COORDS.items():
        print(f"  Tag {tag_id}: ({x}, {y})")
    print("(如需重新测量，请手动运行 measure_apriltag_with_arm.py)")
    print("=" * 60)

    return DEFAULT_WORLD_COORDS.copy()


def main():
    print("=" * 70)
    print("交互式全局摄像头标定工具")
    print("=" * 70)
    print("操作说明:")
    print("  [SPACE] 空格键 - 立即标定 (记录当前 Tag 位置)")
    print("  [M]     测量模式 - 显示/更新 Tag 世界坐标")
    print("  [S]     保存截图 - 保存当前画面")
    print("  [R]     刷新 - 重新检测")
    print("  [Q/ESC] 退出")
    print("=" * 70)

    # 打开全局摄像头
    cap = cv2.VideoCapture(1)
    if not cap.isOpened():
        print("[ERROR] 无法打开全局摄像头 index=1")
        return

    # 设置更高分辨率（如果支持）
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

    print("[OK] 摄像头已打开，请调整角度使三个 AprilTag 都在视野内...")

    # 预热
    for _ in range(10):
        cap.read()

    # 尝试加载已有世界坐标
    world_coords = DEFAULT_WORLD_COORDS.copy()

    status_text = "Waiting for 3 tags..."
    status_color = (0, 165, 255)  # 橙色

    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        # 检测 AprilTag
        tags = detect_apriltags(frame)

        # 绘制检测框
        if tags:
            for tag_id, (cx, cy, corners) in tags.items():
                cv2.aruco.drawDetectedMarkers(frame, [corners], np.array([[tag_id]]))
                cv2.putText(frame, f"ID={tag_id}", (cx - 30, cy - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        # 更新状态
        if len(tags) >= 3:
            status_text = f"3 Tags ready! Press SPACE to calibrate"
            status_color = (0, 255, 0)  # 绿色
        else:
            missing = set([1, 2, 4]) - set(tags.keys())
            status_text = f"Missing: {missing}"
            status_color = (0, 165, 255)  # 橙色

        # 绘制信息面板
        display = draw_info_panel(frame.copy(), tags, status_text, status_color)

        cv2.imshow("Interactive Calibration - Press SPACE to calibrate", display)

        key = cv2.waitKey(1) & 0xFF

        if key == ord('q') or key == 27:  # Q 或 ESC
            break

        elif key == ord(' '):  # 空格键 - 标定
            print("\n[CALIBRATE] 触发标定...")

            success, msg, affine_matrix = calibrate_from_current_frame(frame, world_coords)
            print(f"  {msg}")

            if success:
                # 提取像素坐标
                pixel_coords = {}
                for tag_id in sorted(tags.keys()):
                    cx, cy, _ = tags[tag_id]
                    pixel_coords[tag_id] = (cx, cy)

                # 保存
                save_calibration(affine_matrix, pixel_coords, world_coords)
                print("  [OK] 标定结果已保存到 global_camera_calib.json")

                # 在画面上显示成功信息（冻结几帧）
                success_frame = display.copy()
                h, w = success_frame.shape[:2]
                cv2.rectangle(success_frame, (w // 2 - 300, h // 2 - 40),
                              (w // 2 + 300, h // 2 + 40), (0, 100, 0), -1)
                cv2.putText(success_frame, "CALIBRATION SAVED!",
                            (w // 2 - 220, h // 2 + 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 3)
                cv2.imshow("Interactive Calibration - Press SPACE to calibrate", success_frame)
                cv2.waitKey(2000)  # 显示2秒

            else:
                # 显示失败信息
                fail_frame = display.copy()
                h, w = fail_frame.shape[:2]
                cv2.rectangle(fail_frame, (w // 2 - 300, h // 2 - 40),
                              (w // 2 + 300, h // 2 + 40), (0, 0, 100), -1)
                cv2.putText(fail_frame, "CALIBRATION FAILED",
                            (w // 2 - 200, h // 2 + 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3)
                cv2.putText(fail_frame, msg,
                            (w // 2 - 280, h // 2 + 45),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                cv2.imshow("Interactive Calibration - Press SPACE to calibrate", fail_frame)
                cv2.waitKey(2000)

        elif key == ord('m'):  # M - 测量模式
            print("\n[MEASURE] 查看当前 Tag 世界坐标...")
            print("当前使用的世界坐标:")
            for tag_id, (x, y) in world_coords.items():
                print(f"  Tag {tag_id}: ({x}, {y})")
            print("如需重新测量，请手动运行 measure_apriltag_with_arm.py")

            # 冻结画面显示坐标
            measure_frame = display.copy()
            h, w = measure_frame.shape[:2]
            y_offset = h // 2 - 60
            cv2.rectangle(measure_frame, (w // 2 - 250, y_offset),
                          (w // 2 + 250, y_offset + 120), (50, 50, 50), -1)
            cv2.putText(measure_frame, "World Coords:",
                        (w // 2 - 200, y_offset + 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
            for i, (tag_id, (x, y)) in enumerate(world_coords.items()):
                cv2.putText(measure_frame, f"Tag {tag_id}: ({x}, {y})",
                            (w // 2 - 180, y_offset + 60 + i * 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
            cv2.imshow("Interactive Calibration - Press SPACE to calibrate", measure_frame)
            cv2.waitKey(3000)

        elif key == ord('s'):  # S - 保存截图
            filename = f"calib_snapshot_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jpg"
            cv2.imwrite(filename, frame)
            print(f"\n[SAVE] 截图已保存: {filename}")

            # 显示保存提示
            save_frame = display.copy()
            h, w = save_frame.shape[:2]
            cv2.rectangle(save_frame, (w // 2 - 200, h // 2 - 30),
                          (w // 2 + 200, h // 2 + 30), (50, 50, 50), -1)
            cv2.putText(save_frame, f"Saved: {filename}",
                        (w // 2 - 180, h // 2 + 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.imshow("Interactive Calibration - Press SPACE to calibrate", save_frame)
            cv2.waitKey(1000)

        elif key == ord('r'):  # R - 刷新
            print("\n[REFRESH] 重新检测...")

    cap.release()
    cv2.destroyAllWindows()
    print("\n[OK] 标定工具已关闭")


if __name__ == "__main__":
    main()
