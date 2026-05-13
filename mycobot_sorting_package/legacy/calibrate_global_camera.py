#!/usr/bin/env python3
"""
calibrate_global_camera.py
==========================
全局摄像头 (Eye-to-Hand) 标定脚本。

标定原理:
  机械臂末端携带明显颜色标记（红色/黄色），依次移动到 N 个已知世界坐标点。
  全局摄像头拍下每个位置的标记像素坐标。
  用对应点求解仿射变换矩阵 (X,Y) = f(u,v)。

用法:
  1. 在机械臂末端夹一张红色纸片（或贴红色圆点）
  2. python3 calibrate_global_camera.py --color red
  3. 标定完成后生成 global_camera_calib.json
"""

import argparse
import json
import logging
import os
import time
from datetime import datetime

import cv2
import numpy as np

from motion_service import MotionService

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s"
)
logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# 标定点：使用当前扫描路径的 9 个路点（已知世界坐标）
# ---------------------------------------------------------------------------
DEFAULT_CALIBRATION_POINTS = [
    {"x": 140, "y": -80, "name": "前左"},
    {"x": 170, "y": -80, "name": "前中左"},
    {"x": 200, "y": -80, "name": "前右"},
    {"x": 200, "y":   0, "name": "中右"},
    {"x": 170, "y":   0, "name": "中中"},
    {"x": 140, "y":   0, "name": "中左"},
    {"x": 140, "y":  80, "name": "后左"},
    {"x": 170, "y":  80, "name": "后中左"},
    {"x": 200, "y":  80, "name": "后右"},
]

# ---------------------------------------------------------------------------
# HSV 颜色阈值（用于检测标定标记）
# ---------------------------------------------------------------------------
COLOR_RANGES = {
    "red": {
        "ranges": [
            (np.array([0, 120, 70]), np.array([10, 255, 255])),
            (np.array([170, 120, 70]), np.array([180, 255, 255])),
        ],
        "merge": True,   # red 需要两段 HSV 合并
    },
    "yellow": {
        "ranges": [
            (np.array([20, 100, 100]), np.array([35, 255, 255])),
        ],
        "merge": False,
    },
    "blue": {
        "ranges": [
            (np.array([100, 150, 0]), np.array([140, 255, 255])),
        ],
        "merge": False,
    },
    "green": {
        "ranges": [
            (np.array([35, 80, 180]), np.array([85, 255, 255])),
        ],
        "merge": False,
    },
}


# ---------------------------------------------------------------------------
# 全局摄像头封装
# ---------------------------------------------------------------------------
class GlobalCamera:
    """罗技 BRIO 全局摄像头 (index=1, 1920x1080)"""

    def __init__(self, index: int = 1, width: int = 1920, height: int = 1080):
        self.index = index
        self.width = width
        self.height = height
        self._cap = None

    def open(self) -> bool:
        self._cap = cv2.VideoCapture(self.index)
        if not self._cap.isOpened():
            logger.error(f"全局摄像头 index={self.index} 打开失败")
            return False
        # BRIO 在 macOS 上可能需要先读一帧来稳定
        for _ in range(3):
            self._cap.read()
        logger.info(f"全局摄像头已打开: index={self.index}")
        return True

    def grab(self, retries: int = 5) -> np.ndarray:
        """读取一帧，多次尝试避免黑帧/丢帧。"""
        if self._cap is None:
            return None
        best_frame = None
        best_mean = 0
        for _ in range(retries):
            ret, frame = self._cap.read()
            if ret and frame is not None:
                m = np.mean(frame)
                if m > best_mean:
                    best_mean = m
                    best_frame = frame
        return best_frame

    def close(self):
        if self._cap:
            self._cap.release()
            self._cap = None

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()
        return False


# ---------------------------------------------------------------------------
# 颜色标记检测（针对 LED 点光源优化）
# ---------------------------------------------------------------------------
def detect_color_center(frame: np.ndarray, color_name: str, min_area: int = 3, max_area: int = 200):
    """
    在全局摄像头画面中检测 LED 颜色标记的中心像素坐标。
    针对 LED 点光源优化：亮度优先 + 颜色验证，避免过曝导致颜色丢失。
    返回: (cx, cy, debug_img) 或 (None, None, debug_img)
    """
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # ---- 阶段1: 亮度阈值提取候选亮斑 ----
    # LED 在全局画面中是非常亮的点光源，先按亮度筛选
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # 自适应亮度阈值：取高亮区域（前 0.5% 或固定阈值）
    brightness_thresh = max(200, np.percentile(gray, 99.5))
    bright_mask = cv2.inRange(gray, int(brightness_thresh), 255)

    # 形态学连接邻近像素
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    bright_mask = cv2.morphologyEx(bright_mask, cv2.MORPH_CLOSE, kernel)

    # ---- 阶段2: 找连通区域 ----
    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(bright_mask, connectivity=8)

    candidates = []
    for i in range(1, num_labels):  # 跳过背景 0
        area = stats[i, cv2.CC_STAT_AREA]
        if area < min_area or area > max_area:
            continue

        cx = int(centroids[i][0])
        cy = int(centroids[i][1])

        # 获取该连通区域的掩码
        component_mask = (labels == i)

        # 颜色验证：计算区域内平均 HSV
        mean_h = np.mean(hsv[:, :, 0][component_mask])
        mean_s = np.mean(hsv[:, :, 1][component_mask])
        mean_v = np.mean(hsv[:, :, 2][component_mask])

        # 绿色 LED 验证：H 在绿范围内，V 很高
        is_green = (35 <= mean_h <= 85) and (mean_v >= 180)
        is_bright = mean_v >= 200

        # 打分：越亮、越绿、面积越合适分数越高
        score = mean_v
        if is_green:
            score += 50
        if is_bright:
            score += 30

        candidates.append({
            "cx": cx, "cy": cy, "area": area,
            "mean_h": mean_h, "mean_s": mean_s, "mean_v": mean_v,
            "is_green": is_green, "score": score,
            "mask": component_mask,
        })

    if not candidates:
        # 没有候选，返回调试图（原始帧 + 亮度掩码叠加）
        debug = frame.copy()
        overlay = cv2.cvtColor(bright_mask, cv2.COLOR_GRAY2BGR)
        overlay[:, :, 1] = 0  # 去掉绿色，用蓝色显示掩码
        overlay[:, :, 2] = bright_mask
        cv2.addWeighted(debug, 1.0, overlay, 0.3, 0, debug)
        cv2.putText(debug, f"{color_name}: no candidate", (30, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        return None, None, debug

    # ---- 阶段3: 选最优候选 ----
    best = max(candidates, key=lambda c: c["score"])
    cx, cy = best["cx"], best["cy"]
    area = best["area"]

    # 画调试图
    debug = frame.copy()
    # 画出所有候选（黄色小点）
    for c in candidates:
        cv2.circle(debug, (c["cx"], c["cy"]), 3, (0, 255, 255), -1)
    # 画出最优候选（红色大圈 + 绿色十字）
    cv2.circle(debug, (cx, cy), 10, (0, 0, 255), 2)
    cv2.line(debug, (cx - 12, cy), (cx + 12, cy), (0, 255, 0), 2)
    cv2.line(debug, (cx, cy - 12), (cx, cy + 12), (0, 255, 0), 2)
    cv2.putText(debug, f"LED:({cx},{cy}) A={area} H={best['mean_h']:.0f} V={best['mean_v']:.0f}",
                (cx + 15, cy - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    return cx, cy, debug

    return cx, cy, debug


# ---------------------------------------------------------------------------
# 标定主流程
# ---------------------------------------------------------------------------
def run_calibration(color_name: str = "red", output_dir: str = "calib_run"):
    os.makedirs(output_dir, exist_ok=True)

    # ---- 连接机械臂 ----
    ms = MotionService()
    logger.info("[1/5] 连接机械臂...")
    if not ms.connect(warmup=True):
        logger.error("机械臂连接失败")
        return False

    # ---- 打开全局摄像头 ----
    logger.info("[2/5] 打开全局摄像头 (Logitech BRIO)...")
    global_cam = GlobalCamera(index=1)
    if not global_cam.open():
        logger.error("全局摄像头打开失败")
        ms.close()
        return False

    # 先拍一张全局图看看
    preview = global_cam.grab()
    if preview is not None:
        preview_path = os.path.join(output_dir, "global_preview.jpg")
        cv2.imwrite(preview_path, preview)
        logger.info(f"全局预览图已保存: {preview_path} ({preview.shape[1]}x{preview.shape[0]})")

    # ---- 安全初始化 ----
    logger.info("[3/5] 机械臂初始化...")
    ms.gripper_open()
    time.sleep(0.5)
    ms.go_init(speed=10)
    time.sleep(1.0)

    # ---- 采集对应点 ----
    logger.info(f"[4/5] 开始采集 {len(DEFAULT_CALIBRATION_POINTS)} 个标定点 (标记颜色={color_name})...")
    logger.info("请确认机械臂末端已夹好/贴好 %s 颜色标记!", color_name)
    time.sleep(2.0)

    world_points = []   # (X, Y)
    pixel_points = []   # (u, v)

    for idx, pt in enumerate(DEFAULT_CALIBRATION_POINTS):
        wx, wy = pt["x"], pt["y"]
        name = pt["name"]
        logger.info(f"  -> 标定点 {idx+1}/{len(DEFAULT_CALIBRATION_POINTS)}: {name} ({wx}, {wy})")

        # 移动到标定点上方
        ms.move_to(x=wx, y=wy, z=200, rx=-180, ry=0, rz=45, speed=20, wait=True)
        time.sleep(1.5)  # 等待稳定

        # 全局摄像头拍照
        frame = global_cam.grab()
        if frame is None:
            logger.warning(f"    拍照失败，跳过 {name}")
            continue

        # 检测颜色标记中心
        cx, cy, debug = detect_color_center(frame, color_name)

        # 保存调试图
        debug_path = os.path.join(output_dir, f"calib_{idx:02d}_{name}.jpg")
        if debug is not None:
            cv2.imwrite(debug_path, debug)

        if cx is None:
            logger.warning(f"    未检测到 {color_name} 标记，跳过 {name}")
            continue

        logger.info(f"    像素中心: ({cx}, {cy})  <=>  世界坐标: ({wx}, {wy})")
        world_points.append([float(wx), float(wy)])
        pixel_points.append([float(cx), float(cy)])

    # ---- 收尾 ----
    logger.info("[5/5] 回到零位...")
    ms.go_init(speed=10)
    time.sleep(1.0)
    ms.close()
    global_cam.close()

    # ---- 求解变换矩阵 ----
    n = len(world_points)
    if n < 4:
        logger.error(f"有效标定点仅 {n} 个，至少需要 4 个!")
        return False

    logger.info(f"有效标定点: {n} 个，开始求解变换矩阵...")

    world_np = np.array(world_points, dtype=np.float32)
    pixel_np = np.array(pixel_points, dtype=np.float32)

    # 方法1: 仿射变换 (6自由度，适合平行平面)
    affine_matrix, inliers = cv2.estimateAffine2D(
        pixel_np, world_np,
        method=cv2.LMEDS
    )
    logger.info(f"仿射变换矩阵 (inliers={inliers.sum() if inliers is not None else n}/{n}):\n{affine_matrix}")

    # 方法2: 透视变换 (8自由度，更通用)
    h_matrix, status = cv2.findHomography(
        pixel_np, world_np,
        method=cv2.RANSAC,
        ransacReprojThreshold=5.0
    )
    logger.info(f"透视变换矩阵 (inliers={status.sum() if status is not None else n}/{n}):\n{h_matrix}")

    # ---- 验证精度 ----
    logger.info("验证精度...")
    errors = []
    for i in range(n):
        u, v = pixel_np[i]
        # 仿射变换预测
        pred = affine_matrix @ np.array([[u], [v], [1.0]])
        pred_x, pred_y = pred[0, 0], pred[1, 0]
        err = np.linalg.norm([pred_x - world_np[i, 0], pred_y - world_np[i, 1]])
        errors.append(err)
        logger.info(f"  {i+1}: pred=({pred_x:.1f}, {pred_y:.1f}), actual=({world_np[i,0]:.0f}, {world_np[i,1]:.0f}), err={err:.2f}mm")

    mean_err = np.mean(errors)
    max_err = np.max(errors)
    logger.info(f"平均误差: {mean_err:.2f}mm, 最大误差: {max_err:.2f}mm")

    # ---- 保存标定文件 ----
    calib_data = {
        "created_at": datetime.now().isoformat(),
        "camera": {
            "index": 1,
            "name": "Logitech BRIO",
            "resolution": [1920, 1080],
        },
        "marker_color": color_name,
        "num_points": n,
        "world_points": world_points,
        "pixel_points": pixel_points,
        "affine_matrix": affine_matrix.tolist(),
        "homography_matrix": h_matrix.tolist(),
        "validation": {
            "mean_error_mm": round(float(mean_err), 2),
            "max_error_mm": round(float(max_err), 2),
        },
        "usage": {
            "python": "pixel_to_world(u, v, affine_matrix)",
            "note": "输入 (u,v) 为像素坐标，输出 (X,Y) 为机械臂世界坐标(mm)",
        },
    }

    calib_path = os.path.join(output_dir, "global_camera_calib.json")
    with open(calib_path, "w", encoding="utf-8") as f:
        json.dump(calib_data, f, indent=2, ensure_ascii=False)

    logger.info(f"标定结果已保存: {calib_path}")
    logger.info("标定完成!")
    return True


# ---------------------------------------------------------------------------
# 工具函数：像素 -> 世界坐标
# ---------------------------------------------------------------------------
def pixel_to_world(u: float, v: float, affine_matrix: np.ndarray) -> tuple[float, float]:
    """使用仿射变换矩阵将像素坐标转换为机械臂世界坐标。"""
    pt = affine_matrix @ np.array([[u], [v], [1.0]])
    return float(pt[0, 0]), float(pt[1, 0])


def load_calibration(path: str = "global_camera_calib.json") -> dict:
    """加载标定结果。"""
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


# ---------------------------------------------------------------------------
# 测试模式：仅检测，不移动机械臂
# ---------------------------------------------------------------------------
def test_detection(output_dir: str = "calib_test", color_name: str = "green"):
    """静态测试：连续检测颜色标记，不移动机械臂。"""
    os.makedirs(output_dir, exist_ok=True)
    logger.info("[TEST] 打开全局摄像头并连续检测 %s 标记...", color_name)
    global_cam = GlobalCamera(index=1)
    if not global_cam.open():
        logger.error("摄像头打开失败")
        return False

    # 丢弃前几帧
    for _ in range(5):
        global_cam.grab()
    time.sleep(0.5)

    for i in range(10):
        frame = global_cam.grab()
        if frame is None:
            logger.warning("第 %d 帧读取失败", i+1)
            time.sleep(0.2)
            continue
        cx, cy, debug = detect_color_center(frame, color_name)
        path = os.path.join(output_dir, f"test_{i:02d}.jpg")
        cv2.imwrite(path, debug if debug is not None else frame)
        if cx is not None:
            logger.info("第 %d 帧: 检测到 (%d, %d)", i+1, cx, cy)
        else:
            logger.warning("第 %d 帧: 未检测到 %s", i+1, color_name)
        time.sleep(0.3)

    global_cam.close()
    logger.info("测试完成，图片保存在 %s", output_dir)
    return True


# ---------------------------------------------------------------------------
# 主函数
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description="全局摄像头标定")
    parser.add_argument(
        "--color", default="red",
        choices=["red", "yellow", "blue", "green"],
        help="末端标记颜色 (默认: red)"
    )
    parser.add_argument(
        "--output", default="calib_run",
        help="输出目录"
    )
    parser.add_argument(
        "--test", action="store_true",
        help="仅测试颜色检测，不运行标定"
    )
    args = parser.parse_args()

    if args.test:
        success = test_detection(output_dir=args.output, color_name=args.color)
    else:
        success = run_calibration(color_name=args.color, output_dir=args.output)
    if not success:
        exit(1)


if __name__ == "__main__":
    main()
