"""
camera_vision.py
================
基于 OpenCV 的颜色检测与视觉工具，用于 myCobot 280 Eye-in-Hand 相机。

功能:
1. 实时抓取末端相机图像 (支持索引 / 文件回退 / dummy)
2. 颜色空间阈值分割 (HSV)
3. 轮廓检测与中心点计算
4. 像素坐标 -> 相机坐标系偏移量 的映射 (基于单目相机标定参数)
"""

from __future__ import annotations

import logging
import math
from typing import Dict, Optional, Tuple

import cv2
import numpy as np

# ---------------------------------------------------------------------------
# Simple data classes (replaces missing mycobot_280_active_search_expert import)
# ---------------------------------------------------------------------------

class Coords:
    def __init__(self, x: float, y: float, z: float):
        self.x = x
        self.y = y
        self.z = z

    def __repr__(self):
        return f"Coords(x={self.x:.2f}, y={self.y:.2f}, z={self.z:.2f})"


class DetectedObject:
    def __init__(
        self,
        relative_offset: Coords,
        confidence: float,
        pixel_center: Tuple[int, int],
        color_name: str,
    ):
        self.relative_offset = relative_offset
        self.confidence = confidence
        self.pixel_center = pixel_center
        self.color_name = color_name

    def __repr__(self):
        return (
            f"DetectedObject(color={self.color_name}, "
            f"pixel={self.pixel_center}, conf={self.confidence:.2f})"
        )


logger = logging.getLogger(__name__)
logger.addHandler(logging.StreamHandler())
logger.setLevel(logging.INFO)


# ---------------------------------------------------------------------------
# Default HSV color definitions (tunable)
# ---------------------------------------------------------------------------

DEFAULT_COLOR_RANGES: Dict[str, Tuple[np.ndarray, np.ndarray]] = {
    "red": (
        np.array([0, 120, 70]),
        np.array([10, 255, 255]),
    ),
    "red2": (  # red wraps around in HSV
        np.array([170, 120, 70]),
        np.array([180, 255, 255]),
    ),
    "blue": (
        np.array([100, 150, 0]),
        np.array([140, 255, 255]),
    ),
    "green": (
        np.array([40, 100, 100]),
        np.array([80, 255, 255]),
    ),
    "yellow": (
        np.array([20, 100, 100]),
        np.array([35, 255, 255]),
    ),
    "orange": (
        np.array([10, 100, 100]),
        np.array([25, 255, 255]),
    ),
}


# ---------------------------------------------------------------------------
# Camera interface
# ---------------------------------------------------------------------------

class EndEffectorCamera:
    """
    封装末端 USB 相机，提供统一的 grab() 接口。

    支持模式:
        - "index":   标准 OpenCV VideoCapture 索引 (如 0, 1)
        - "file":    视频文件路径 (用于离线测试)
        - "dummy":   虚拟相机，返回纯色/噪声图像 (纯软件测试)
    """

    def __init__(
        self,
        source=None,
        width: int = 640,
        height: int = 480,
        fps: int = 30,
        mode: str = "auto",
    ):
        self.source = source if source is not None else 0
        self.width = width
        self.height = height
        self.fps = fps
        self._cap: Optional[cv2.VideoCapture] = None
        self._mode = mode

        if mode == "auto":
            if isinstance(self.source, int) or (
                isinstance(self.source, str) and str(self.source).isdigit()
            ):
                self._mode = "index"
            elif isinstance(self.source, str):
                self._mode = "file"
            else:
                self._mode = "dummy"

        self._dummy_frame_count = 0

    # ------------------------------------------------------------------

    def open(self) -> bool:
        """打开相机/文件。支持自动探测末端相机（选择 640x480 分辨率）。"""
        if self._mode == "index":
            idx = int(self.source)

            # macOS AVFoundation 索引漂移：自动探测目标分辨率的相机
            if idx == 0 and (self.width == 640 and self.height == 480):
                detected = self._find_camera_by_resolution(
                    target_w=self.width, target_h=self.height, max_index=4
                )
                if detected is not None:
                    idx = detected
                    logger.info(f"Auto-selected camera index {idx} as end-effector camera ({self.width}x{self.height})")
                else:
                    logger.warning(f"Could not auto-detect {self.width}x{self.height} camera; falling back to index {idx}")

            self._cap = cv2.VideoCapture(idx)
            if not self._cap.isOpened():
                logger.error(f"Failed to open camera index {idx}")
                return False
            self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            self._cap.set(cv2.CAP_PROP_FPS, self.fps)

            # 验证实际分辨率
            actual_w = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_h = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            logger.info(f"Camera opened: index={idx}, {actual_w}x{actual_h}")
            return True

        if self._mode == "file":
            self._cap = cv2.VideoCapture(str(self.source))
            if not self._cap.isOpened():
                logger.error(f"Failed to open video file {self.source}")
                return False
            logger.info(f"Video file opened: {self.source}")
            return True

        # dummy
        logger.info("Dummy camera initialized.")
        return True

    def _find_camera_by_resolution(self, target_w: int, target_h: int, max_index: int = 4) -> Optional[int]:
        """
        枚举相机索引，返回第一个实际分辨率匹配 target_w x target_h 的索引。
        """
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

    def close(self) -> None:
        if self._cap is not None:
            self._cap.release()
            self._cap = None

    def is_opened(self) -> bool:
        if self._mode == "dummy":
            return True
        return self._cap is not None and self._cap.isOpened()

    def grab(self) -> Optional[np.ndarray]:
        """抓取一帧图像。"""
        if self._mode == "dummy":
            return self._generate_dummy_frame()

        if self._cap is None or not self._cap.isOpened():
            return None

        ret, frame = self._cap.read()
        if not ret:
            return None
        return frame

    # ------------------------------------------------------------------

    def _generate_dummy_frame(self) -> np.ndarray:
        """生成一张用于纯软件测试的虚拟图像。"""
        self._dummy_frame_count += 1
        frame = np.ones((self.height, self.width, 3), dtype=np.uint8) * 80
        # 画一个简单色块在中心偏右下 (模拟目标)
        cx, cy = self.width // 2 + 40, self.height // 2 + 30
        cv2.circle(frame, (cx, cy), 30, (0, 0, 255), -1)  # red circle
        # 加一点噪声
        noise = np.random.randint(0, 20, frame.shape, dtype=np.uint8)
        frame = cv2.add(frame, noise)
        return frame

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()
        return False


# ---------------------------------------------------------------------------
# Color detector
# ---------------------------------------------------------------------------

class ColorDetector:
    """
    基于 HSV 阈值的颜色检测器。

    输入图像 -> 颜色掩码 -> 轮廓 -> 面积筛选 -> 中心点 ->
    像素偏移 -> 相机坐标系偏移 (mm)
    """

    def __init__(
        self,
        camera: EndEffectorCamera,
        color_ranges=None,
        min_area: int = 500,
        max_area: int = 100000,
        focal_length_px: float = 600.0,  # 默认焦距 (像素)
        object_real_size_mm: float = 40.0,  # 假设目标真实直径 (mm)
        camera_offset=(0.0, 0.0, 0.0),
    ):
        """
        参数:
            camera: 相机实例
            color_ranges: HSV 阈值字典
            min_area / max_area: 轮廓面积过滤范围
            focal_length_px: 相机焦距 (像素)，用于估算距离
            object_real_size_mm: 目标物实际尺寸，配合成像大小估算 Z 偏移
            camera_offset: 相机光心相对末端法兰的偏移 [dx, dy, dz] (mm)
        """
        self.camera = camera
        self.color_ranges = color_ranges or DEFAULT_COLOR_RANGES.copy()
        self.min_area = min_area
        self.max_area = max_area
        self.focal_length_px = focal_length_px
        self.object_real_size_mm = object_real_size_mm
        self.camera_offset = camera_offset

    # ------------------------------------------------------------------

    def detect(self, color_name: str) -> Optional[DetectedObject]:
        """
        抓取一帧图像并检测指定颜色目标。

        返回:
            DetectedObject (包含相对偏移、置信度、像素中心)
            None: 未检测到
        """
        frame = self.camera.grab()
        if frame is None:
            logger.warning("Camera returned empty frame.")
            return None

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 构建掩码 (红色需要两段 HSV)
        mask = self._build_mask(hsv, color_name)
        if mask is None:
            logger.warning(f"Unknown color: {color_name}")
            return None

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
        if not (self.min_area <= area <= self.max_area):
            return None

        # 计算中心点 (像素)
        M = cv2.moments(largest)
        if M["m00"] == 0:
            return None
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])

        # 像素 -> 相机坐标系偏移
        rel = self._pixel_to_camera_offset(frame, cx, cy, area)

        # 置信度：根据面积占图像比例简单估算
        img_area = frame.shape[0] * frame.shape[1]
        confidence = min(1.0, area / (img_area * 0.3))

        return DetectedObject(
            relative_offset=rel,
            confidence=confidence,
            pixel_center=(cx, cy),
            color_name=color_name,
        )

    # ------------------------------------------------------------------

    def _build_mask(self, hsv: np.ndarray, color_name: str) -> Optional[np.ndarray]:
        """构建 HSV 掩码。"""
        lower, upper = self.color_ranges.get(color_name, (None, None))
        if lower is None:
            return None
        mask = cv2.inRange(hsv, lower, upper)

        # 红色需要两段合并
        if color_name == "red":
            lower2, upper2 = self.color_ranges.get("red2", (None, None))
            if lower2 is not None:
                mask2 = cv2.inRange(hsv, lower2, upper2)
                mask = cv2.bitwise_or(mask, mask2)
        return mask

    def _pixel_to_camera_offset(
        self,
        frame: np.ndarray,
        cx: int,
        cy: int,
        contour_area: float,
    ) -> Coords:
        """
        将像素坐标转换为相机坐标系下的相对偏移 (mm)。

        简化模型 (pinhole, Z 由目标尺寸估算):
            X_cam = (cx - cx0) * Z / fx
            Y_cam = (cy - cy0) * Z / fy
            Z_cam = object_real_size_mm * fx / sqrt(area/pi*4)

        注意:
        - 这里是简化的单目估算。如有标定参数，应替换为 reprojectImageTo3D
          或 solvePnP 精确解。
        - 输出已叠加 camera_offset，得到相对末端法兰的偏移。
        """
        h, w = frame.shape[:2]
        cx0, cy0 = w / 2.0, h / 2.0
        fx = fy = self.focal_length_px

        # 用轮廓面积反推目标到相机的距离 (Z)
        # 假设轮廓近似圆形: area = pi * r^2  =>  r_pixel = sqrt(area/pi)
        # 真实半径 r_mm = object_real_size_mm / 2
        # Z = f * r_mm / r_pixel
        r_pixel = math.sqrt(contour_area / math.pi)
        if r_pixel < 1e-3:
            z_cam = 200.0
        else:
            r_mm = self.object_real_size_mm / 2.0
            z_cam = fx * r_mm / r_pixel

        # 像素偏移 -> 相机坐标偏移
        x_cam = (cx - cx0) * z_cam / fx
        y_cam = (cy - cy0) * z_cam / fy

        # 叠加相机安装偏移 (光心到末端法兰)
        dx, dy, dz = self.camera_offset
        return Coords(
            x=x_cam + dx,
            y=y_cam + dy,
            z=z_cam + dz,
        )

    def visualize(self, frame: np.ndarray, obj: DetectedObject) -> np.ndarray:
        """在图像上绘制检测结果。"""
        vis = frame.copy()
        cx, cy = obj.pixel_center
        cv2.circle(vis, (cx, cy), 5, (0, 255, 0), -1)
        cv2.putText(
            vis,
            f"{obj.color_name}: {obj.confidence:.2f}",
            (cx + 10, cy - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2,
        )
        return vis
