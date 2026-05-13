"""
vision.py
=========
统一视觉模块。整合项目中所有重复的颜色检测、相机封装和坐标转换逻辑。

设计原则：
  - 纯函数优先：检测逻辑不依赖全局状态，只接收图像和参数
  - 相机封装统一：GlobalCamera 和 EndCamera 行为一致（open/grab/close）
  - 与 config.py 配合：颜色阈值、ROI 等参数从配置读取
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np

from config import (
    DEFAULT_COLOR_RANGES,
    END_CAMERA_FPS,
    END_CAMERA_HEIGHT,
    END_CAMERA_INDEX,
    END_CAMERA_WIDTH,
    GLOBAL_CAMERA_FPS,
    GLOBAL_CAMERA_HEIGHT,
    GLOBAL_CAMERA_INDEX,
    GLOBAL_CAMERA_WIDTH,
    GLOBAL_COLOR_RANGES,
    GLOBAL_ROI,
    MAX_OBJECT_AREA,
    MIN_GLOBAL_AREA,
    MIN_OBJECT_AREA,
    load_affine_matrix,
)

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# 数据结构
# ---------------------------------------------------------------------------

@dataclass
class DetectionResult:
    """颜色检测的统一结果格式"""
    found: bool
    color: str
    pixel_center: Optional[Tuple[int, int]] = None
    area: Optional[float] = None
    offset_from_center: Optional[Tuple[int, int]] = None
    world_xy: Optional[Tuple[float, float]] = None
    debug_image: Optional[np.ndarray] = None
    reason: Optional[str] = None  # 未找到时说明原因

    def to_dict(self) -> dict:
        d = {"found": self.found, "color": self.color}
        if self.pixel_center:
            d["pixel_center"] = self.pixel_center
        if self.area is not None:
            d["area"] = round(self.area, 1)
        if self.offset_from_center:
            d["offset_from_center"] = self.offset_from_center
        if self.world_xy:
            d["world_xy"] = self.world_xy
        if self.reason:
            d["reason"] = self.reason
        return d


# ---------------------------------------------------------------------------
# 颜色检测（纯函数）
# ---------------------------------------------------------------------------

def detect_color_in_frame(
    frame: np.ndarray,
    color: str,
    color_ranges: Optional[Dict[str, List[Tuple[np.ndarray, np.ndarray]]]] = None,
    min_area: int = MIN_OBJECT_AREA,
    max_area: int = MAX_OBJECT_AREA,
    kernel_size: int = 5,
    return_debug: bool = False,
) -> DetectionResult:
    """
    在单帧图像中检测指定颜色目标。

    参数:
        frame: BGR 图像
        color: 颜色名称，如 "red"/"blue"/"green"/"yellow"
        color_ranges: HSV 阈值字典，None 则使用 DEFAULT_COLOR_RANGES
        min_area / max_area: 轮廓面积过滤范围
        kernel_size: 形态学核大小
        return_debug: 是否返回带标注的调试图像

    返回:
        DetectionResult
    """
    if frame is None or frame.size == 0:
        return DetectionResult(found=False, color=color, reason="empty frame")

    ranges = color_ranges or DEFAULT_COLOR_RANGES
    if color not in ranges:
        return DetectionResult(found=False, color=color, reason=f"unknown color: {color}")

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # 构建掩码
    masks = []
    for lower, upper in ranges[color]:
        masks.append(cv2.inRange(hsv, lower, upper))
    mask = masks[0]
    for m in masks[1:]:
        mask = cv2.bitwise_or(mask, m)

    # 形态学去噪
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # 找轮廓
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return DetectionResult(found=False, color=color, reason="no contours")

    # 筛选面积并排序
    candidates = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if min_area <= area <= max_area:
            M = cv2.moments(cnt)
            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                candidates.append((area, cx, cy, cnt))

    if not candidates:
        return DetectionResult(found=False, color=color, reason="no valid area")

    # 选最大
    candidates.sort(reverse=True)
    area, cx, cy, cnt = candidates[0]
    h, w = frame.shape[:2]
    dx = cx - w // 2
    dy = cy - h // 2

    # 生成调试图
    debug = None
    if return_debug:
        debug = frame.copy()
        cv2.circle(debug, (cx, cy), 8, (0, 255, 0), 2)
        cv2.drawMarker(debug, (w // 2, h // 2), (0, 0, 255), cv2.MARKER_CROSS, 30, 2)
        cv2.putText(debug, f"{color}: ({cx},{cy}) a={int(area)}", (cx + 10, cy - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

    return DetectionResult(
        found=True,
        color=color,
        pixel_center=(cx, cy),
        area=area,
        offset_from_center=(dx, dy),
        debug_image=debug,
    )


def detect_color_global(
    frame: np.ndarray,
    color: str,
    apply_roi: bool = True,
    return_debug: bool = False,
) -> DetectionResult:
    """
    使用全局摄像头画面检测颜色（应用全局 ROI 和全局颜色阈值）。
    """
    if frame is None:
        return DetectionResult(found=False, color=color, reason="empty frame")

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    ranges = GLOBAL_COLOR_RANGES
    if color not in ranges:
        return DetectionResult(found=False, color=color, reason=f"unknown color: {color}")

    masks = []
    for lower, upper in ranges[color]:
        masks.append(cv2.inRange(hsv, lower, upper))
    mask = masks[0]
    for m in masks[1:]:
        mask = cv2.bitwise_or(mask, m)

    # 应用 ROI
    if apply_roi:
        roi_mask = np.zeros_like(mask)
        roi_mask[GLOBAL_ROI["v_min"]:GLOBAL_ROI["v_max"],
                 GLOBAL_ROI["u_min"]:GLOBAL_ROI["u_max"]] = \
            mask[GLOBAL_ROI["v_min"]:GLOBAL_ROI["v_max"],
                 GLOBAL_ROI["u_min"]:GLOBAL_ROI["u_max"]]
        mask = roi_mask

    # 形态学
    kernel = np.ones((7, 7), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return DetectionResult(found=False, color=color, reason="no contours in ROI")

    best = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(best)
    if area < MIN_GLOBAL_AREA:
        return DetectionResult(found=False, color=color, reason="too small", area=area)

    M = cv2.moments(best)
    if M["m00"] == 0:
        return DetectionResult(found=False, color=color, reason="moment zero")
    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])
    h, w = frame.shape[:2]

    debug = None
    if return_debug:
        debug = frame.copy()
        cv2.circle(debug, (cx, cy), 12, (0, 255, 0), 2)
        cv2.putText(debug, f"{color}:({cx},{cy}) a={int(area)}", (cx + 15, cy - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        # 画出 ROI 框
        cv2.rectangle(debug,
                      (GLOBAL_ROI["u_min"], GLOBAL_ROI["v_min"]),
                      (GLOBAL_ROI["u_max"], GLOBAL_ROI["v_max"]),
                      (255, 0, 0), 2)

    return DetectionResult(
        found=True,
        color=color,
        pixel_center=(cx, cy),
        area=area,
        offset_from_center=(cx - w // 2, cy - h // 2),
        debug_image=debug,
    )


# ---------------------------------------------------------------------------
# 像素 -> 世界坐标转换
# ---------------------------------------------------------------------------

def pixel_to_world(
    u: float, v: float,
    affine_matrix: Optional[np.ndarray] = None,
) -> Optional[Tuple[float, float]]:
    """
    将全局摄像头像素坐标转换为机械臂世界坐标 (mm)。
    若未提供矩阵，自动尝试加载 global_camera_calib.json。
    """
    M = affine_matrix if affine_matrix is not None else load_affine_matrix()
    if M is None or M.shape != (2, 3):
        logger.warning("affine_matrix not available")
        return None
    pt = M @ np.array([[u], [v], [1.0]])
    return float(pt[0, 0]), float(pt[1, 0])


def world_to_pixel(
    x: float, y: float,
    affine_matrix: Optional[np.ndarray] = None,
) -> Optional[Tuple[float, float]]:
    """
    将世界坐标转换为像素坐标（需要逆矩阵）。
    若未提供矩阵，自动尝试加载 global_camera_calib.json。
    """
    M = affine_matrix if affine_matrix is not None else load_affine_matrix()
    if M is None or M.shape != (2, 3):
        return None
    try:
        M_inv = cv2.invertAffineTransform(M)
        pt = M_inv @ np.array([[x], [y], [1.0]])
        return float(pt[0, 0]), float(pt[1, 0])
    except Exception as exc:
        logger.warning("world_to_pixel failed: %s", exc)
        return None


# ---------------------------------------------------------------------------
# 坐标修正（迭代对准用）
# ---------------------------------------------------------------------------

def compute_correction(
    dx_px: int, dy_px: int,
    scale: float = 0.32,
    ratio: float = 0.6,
) -> Tuple[float, float]:
    """
    根据像素偏移计算机械臂坐标修正量 (dX, dY)。
    图像坐标系 -> 机械臂坐标系映射：
      画面上方 -> X增加 (y像素减小) -> dX = -dy_px * scale * ratio
      画面左方 -> Y增加 (x像素减小) -> dY = -dx_px * scale * ratio
    """
    dX = -dy_px * scale * ratio
    dY = -dx_px * scale * ratio
    return dX, dY


# ---------------------------------------------------------------------------
# 统一相机封装
# ---------------------------------------------------------------------------

class _CameraBase:
    """相机基类，统一下层接口"""

    def __init__(self, source, width: int, height: int, fps: int):
        self.source = source
        self.width = width
        self.height = height
        self.fps = fps
        self._cap: Optional[cv2.VideoCapture] = None

    def open(self) -> bool:
        raise NotImplementedError

    def grab(self, retries: int = 3) -> Optional[np.ndarray]:
        """抓取一帧图像，失败重试"""
        if self._cap is None or not self._cap.isOpened():
            return None
        for _ in range(retries):
            ret, frame = self._cap.read()
            if ret and frame is not None:
                return frame
        return None

    def close(self) -> None:
        if self._cap is not None:
            self._cap.release()
            self._cap = None

    def is_opened(self) -> bool:
        return self._cap is not None and self._cap.isOpened()

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()
        return False


class EndEffectorCamera(_CameraBase):
    """末端 USB 相机（通常 index=0, 640x480）"""

    def __init__(
        self,
        source: int = END_CAMERA_INDEX,
        width: int = END_CAMERA_WIDTH,
        height: int = END_CAMERA_HEIGHT,
        fps: int = END_CAMERA_FPS,
    ):
        super().__init__(source, width, height, fps)

    def open(self) -> bool:
        idx = int(self.source)
        # macOS 自动探测：如果 idx=0 且目标 640x480，尝试找匹配的相机
        if idx == 0 and (self.width == 640 and self.height == 480):
            detected = self._find_by_resolution(target_w=self.width, target_h=self.height, max_index=4)
            if detected is not None:
                idx = detected
                logger.info("Auto-selected camera index %d as end-effector camera (%dx%d)",
                            idx, self.width, self.height)

        self._cap = cv2.VideoCapture(idx)
        if not self._cap.isOpened():
            logger.error("Failed to open camera index %d", idx)
            return False
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self._cap.set(cv2.CAP_PROP_FPS, self.fps)

        actual_w = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        logger.info("End camera opened: index=%d, %dx%d", idx, actual_w, actual_h)
        return True

    def _find_by_resolution(self, target_w: int, target_h: int, max_index: int = 4) -> Optional[int]:
        for i in range(max_index + 1):
            cap = cv2.VideoCapture(i)
            if cap.isOpened():
                ret, frame = cap.read()
                w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                cap.release()
                if ret and frame is not None and w == target_w and h == target_h:
                    return i
            cap.release()
        return None


class GlobalCamera(_CameraBase):
    """全局摄像头（如 Logitech BRIO，通常 index=1, 1920x1080）"""

    def __init__(
        self,
        source: int = GLOBAL_CAMERA_INDEX,
        width: int = GLOBAL_CAMERA_WIDTH,
        height: int = GLOBAL_CAMERA_HEIGHT,
        fps: int = GLOBAL_CAMERA_FPS,
    ):
        super().__init__(source, width, height, fps)

    def open(self) -> bool:
        self._cap = cv2.VideoCapture(int(self.source))
        if not self._cap.isOpened():
            logger.error("Failed to open global camera index %s", self.source)
            return False
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        # 预热丢弃前几帧
        for _ in range(3):
            self._cap.read()
        logger.info("Global camera opened: index=%s", self.source)
        return True

    def grab(self, retries: int = 5) -> Optional[np.ndarray]:
        """全局摄像头多帧取最优（避免黑帧/丢帧）"""
        if self._cap is None or not self._cap.isOpened():
            return None
        best_frame = None
        best_mean = 0.0
        for _ in range(retries):
            ret, frame = self._cap.read()
            if ret and frame is not None:
                m = np.mean(frame)
                if m > best_mean:
                    best_mean = m
                    best_frame = frame
        return best_frame


# ---------------------------------------------------------------------------
# 调试图保存
# ---------------------------------------------------------------------------

def save_debug_image(
    frame: np.ndarray,
    path: Path,
    detection: Optional[DetectionResult] = None,
    crosshair: bool = True,
) -> Path:
    """保存带标注的调试图像"""
    vis = frame.copy()
    h, w = vis.shape[:2]

    if crosshair:
        cx, cy = w // 2, h // 2
        cv2.drawMarker(vis, (cx, cy), (0, 0, 255), cv2.MARKER_CROSS, 30, 2)

    if detection and detection.found and detection.pixel_center:
        px, py = detection.pixel_center
        cv2.circle(vis, (px, py), 8, (0, 255, 0), 2)
        info = f"{detection.color}:({px},{py})"
        if detection.area:
            info += f" a={int(detection.area)}"
        cv2.putText(vis, info, (px + 10, py - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(path), vis)
    logger.info("Debug image saved: %s", path)
    return path


# ---------------------------------------------------------------------------
# 便捷封装：完整视觉流程
# ---------------------------------------------------------------------------

def global_detect_and_convert(
    frame: np.ndarray,
    color: str,
    return_debug: bool = False,
) -> DetectionResult:
    """
    全局摄像头检测 + 自动转换为世界坐标。
    一步完成 detect_color_global + pixel_to_world。
    """
    result = detect_color_global(frame, color, apply_roi=True, return_debug=return_debug)
    if result.found and result.pixel_center:
        world = pixel_to_world(*result.pixel_center)
        if world:
            result.world_xy = world
    return result


# ---------------------------------------------------------------------------
__all__ = [
    "DetectionResult",
    "detect_color_in_frame",
    "detect_color_global",
    "pixel_to_world",
    "world_to_pixel",
    "compute_correction",
    "EndEffectorCamera",
    "GlobalCamera",
    "save_debug_image",
    "global_detect_and_convert",
]
