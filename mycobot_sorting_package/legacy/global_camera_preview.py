#!/usr/bin/env python3
"""
global_camera_preview.py
=======================
全局摄像头 (BRIO) 实时预览 + AprilTag 检测

用法:
    python3 global_camera_preview.py

按键:
    q - 退出
    s - 保存当前帧
    r - 切换检测区域显示
"""

import cv2
import numpy as np

# AprilTag 检测器
APRILTAG_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
APRILTAG_DETECTOR = cv2.aruco.ArucoDetector(APRILTAG_DICT)

# 三个 Tag 的大致像素位置（参考）
TAG_REFS = {
    1: (790, 550),
    2: (981, 823),
    4: (937, 179),
}


def detect_apriltags(frame):
    """检测 AprilTag，返回 {id: (cx, cy)}"""
    corners, ids, _ = APRILTAG_DETECTOR.detectMarkers(frame)
    result = {}
    if ids is not None:
        for i, marker_id in enumerate(ids.flatten()):
            c = corners[i][0]
            cx = int(np.mean(c[:, 0]))
            cy = int(np.mean(c[:, 1]))
            result[int(marker_id)] = (cx, cy)
    return result, corners, ids


def main():
    print("=" * 60)
    print("全局摄像头 (BRIO) 实时预览")
    print("=" * 60)
    print("按键说明:")
    print("  q - 退出")
    print("  s - 保存当前帧")
    print("  r - 显示/隐藏参考区域")
    print("=" * 60)

    cap = cv2.VideoCapture(1)
    if not cap.isOpened():
        print("[ERROR] 无法打开全局摄像头 index=1")
        return

    print("[OK] 摄像头已打开")
    
    frame_count = 0
    show_regions = True
    tags = {}
    corners = None
    ids = None

    while True:
        ret, frame = cap.read()
        if not ret:
            print("[WARN] 读取失败")
            continue

        frame_count += 1
        vis = frame.copy()
        h, w = vis.shape[:2]

        # 每10帧检测一次 AprilTag（降低CPU占用）
        if frame_count % 10 == 0:
            tags, corners, ids = detect_apriltags(frame)
        
        # 画出检测到的 Tag
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(vis, corners, ids)
            for i, marker_id in enumerate(ids.flatten()):
                c = corners[i][0]
                cx = int(np.mean(c[:, 0]))
                cy = int(np.mean(c[:, 1]))
                cv2.putText(vis, f"ID={marker_id}", (cx - 40, cy - 20),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        # 画出参考区域（三个 Tag 应该出现的位置）
        if show_regions:
            for tag_id, (cx, cy) in TAG_REFS.items():
                color = (0, 255, 255) if tag_id not in [t for t in (tags.keys() if 'tags' in dir() else [])] else (0, 255, 0)
                cv2.circle(vis, (cx, cy), 30, color, 2)
                cv2.putText(vis, f"Tag{tag_id}", (cx - 25, cy - 35),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        # 画面中心十字
        cv2.line(vis, (w // 2, 0), (w // 2, h), (255, 0, 0), 1)
        cv2.line(vis, (0, h // 2), (w, h // 2), (255, 0, 0), 1)

        # 信息显示
        info_text = f"Frame: {frame_count} | Resolution: {w}x{h}"
        if ids is not None:
            info_text += f" | Tags: {list(ids.flatten())}"
        cv2.putText(vis, info_text, (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        cv2.imshow("Global Camera (BRIO) - Press 'q' to exit", vis)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            filename = f"global_snapshot_{frame_count}.jpg"
            cv2.imwrite(filename, frame)
            print(f"[SAVE] 已保存 {filename}")
        elif key == ord('r'):
            show_regions = not show_regions
            print(f"[INFO] 参考区域显示: {show_regions}")

    cap.release()
    cv2.destroyAllWindows()
    print("[OK] 预览已关闭")


if __name__ == "__main__":
    main()
