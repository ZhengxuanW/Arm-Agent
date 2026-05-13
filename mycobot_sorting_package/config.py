"""
config.py
=========
统一配置模块。项目所有硬编码参数的单一事实来源。

设计原则：
  - 任何文件如果需要"魔法数字"，先从这里 import
  - 环境相关配置（串口、相机索引）可通过环境变量覆盖
  - 机器人硬件参数与业务参数分离
"""

from __future__ import annotations

import json
import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

# ---------------------------------------------------------------------------
# 目录常量
# ---------------------------------------------------------------------------
PROJECT_ROOT = Path(__file__).parent.resolve()
LOG_DIR = PROJECT_ROOT / "local_agent_logs"
STATE_LOG_FILE = PROJECT_ROOT / "robot_state.log"
CALIB_FILE = PROJECT_ROOT / "global_camera_calib.json"
MARKER_MAP_FILE = PROJECT_ROOT / "marker_map.json"
GRASP_OFFSET_FILE = PROJECT_ROOT / "grasp_offset.json"

# ---------------------------------------------------------------------------
# 硬件连接配置（可通过环境变量覆盖）
# ---------------------------------------------------------------------------
ROBOT_PORT = os.getenv("MYCobot_PORT", "/dev/cu.usbserial-1110")
ROBOT_BAUD = int(os.getenv("MYCobot_BAUD", "1000000"))

# 相机索引 (macOS 下末端相机通常是 640x480, BRIO 是 1920x1080)
END_CAMERA_INDEX = int(os.getenv("MYCobot_END_CAM", "0"))
END_CAMERA_WIDTH = 640
END_CAMERA_HEIGHT = 480
END_CAMERA_FPS = 30

GLOBAL_CAMERA_INDEX = int(os.getenv("MYCobot_GLOBAL_CAM", "1"))
GLOBAL_CAMERA_WIDTH = 1920
GLOBAL_CAMERA_HEIGHT = 1080
GLOBAL_CAMERA_FPS = 30

# ---------------------------------------------------------------------------
# 安全边界 (mm) — 机械臂工作空间限制
# myCobot 280 理论工作半径 ~280mm，此处留 10mm 余量
# ---------------------------------------------------------------------------
SAFE_X_LIMIT: Tuple[float, float] = (-280.0, 280.0)
SAFE_Y_LIMIT: Tuple[float, float] = (-280.0, 280.0)
SAFE_Z_LIMIT: Tuple[float, float] = (30.0, 400.0)

# 软边界：到达此边界时减速
SOFT_X_LIMIT: Tuple[float, float] = (-260.0, 260.0)
SOFT_Y_LIMIT: Tuple[float, float] = (-260.0, 260.0)
SOFT_Z_LIMIT: Tuple[float, float] = (50.0, 350.0)

# ---------------------------------------------------------------------------
# 运动参数
# ---------------------------------------------------------------------------
DEFAULT_SPEED = 20          # 默认笛卡尔运动速度
HOME_SPEED = 10             # 回零速度（更慢更安全）
GRIPPER_SPEED = 50          # 夹爪开合速度
SAFE_MAX_SPEED = 50         # 绝对速度上限

# 初始化位姿 (关节空间)
INIT_ANGLES = [0.0, 0.0, 0.0, 0.0, 20.0, 0.0]

# HOME 位姿 (笛卡尔)
HOME_COORDS = [0.0, 160.0, 200.0, -180.0, 0.0, 0.0]

# 奇异点阈值 (J5 角度绝对值小于此值时发出警告)
SINGULARITY_J5_THRESH = 5.0

# ---------------------------------------------------------------------------
# 高度层 (mm) — 定义清晰的高度语义
# ---------------------------------------------------------------------------
Z_SCAN = 200          # 扫描/拍照高度
Z_APPROACH = 140      # 接近/悬停高度（防碰撞）
Z_GRAB = 95           # 抓取高度（末端夹爪已补偿后）
Z_RELEASE = 105       # 放置高度

# ---------------------------------------------------------------------------
# 视觉参数
# ---------------------------------------------------------------------------
# 像素 -> 世界坐标比例 (Z=200mm 时的经验值)
SCALE_Z200 = 0.32

# 迭代对准参数
ALIGN_RATIO = 0.60            # 每次修正偏移量的比例
ALIGN_THRESH_PX = 20          # 像素收敛阈值
MAX_ALIGN_ITERS = 8           # 最大迭代次数
ALIGN_SPEED = 15              # 对准过程的运动速度

# 颜色检测面积阈值
MIN_OBJECT_AREA = 300         # 末端相机最小有效面积
MAX_OBJECT_AREA = 100000      # 最大有效面积（原40000太小，近距离目标容易超出）
MIN_GLOBAL_AREA = 500         # 全局相机最小有效面积

# ---------------------------------------------------------------------------
# 夹爪偏移 — 相机光心到夹爪中心的固定偏移 (mm)
# 注意：只在 Z=95 验证有效，其余高度需另行标定
# ---------------------------------------------------------------------------
GRASP_OFFSET_X = 30.0
GRASP_OFFSET_Y = 0.0

# ---------------------------------------------------------------------------
# HSV 颜色阈值 — 统一、可复用的颜色定义
# ---------------------------------------------------------------------------

ColorRange = Tuple[np.ndarray, np.ndarray]

DEFAULT_COLOR_RANGES: Dict[str, List[ColorRange]] = {
    "red": [
        (np.array([0, 80, 80]), np.array([12, 255, 255])),
        (np.array([160, 80, 80]), np.array([180, 255, 255])),
    ],
    "blue": [
        (np.array([100, 150, 0]), np.array([140, 255, 255])),
    ],
    "green": [
        (np.array([40, 100, 100]), np.array([80, 255, 255])),
    ],
    "yellow": [
        (np.array([20, 100, 100]), np.array([35, 255, 255])),
    ],
    "orange": [
        (np.array([10, 100, 100]), np.array([25, 255, 255])),
    ],
}

# 全局摄像头用（阈值略有不同，因光照/分辨率差异）
GLOBAL_COLOR_RANGES: Dict[str, List[ColorRange]] = {
    "red": [
        (np.array([0, 100, 80]), np.array([12, 255, 255])),
        (np.array([160, 100, 80]), np.array([180, 255, 255])),
    ],
    "blue": [
        (np.array([90, 100, 50]), np.array([130, 255, 255])),
    ],
    "green": [
        (np.array([35, 80, 50]), np.array([85, 255, 255])),
    ],
    "yellow": [
        (np.array([18, 120, 80]), np.array([35, 255, 255])),
    ],
}

# ---------------------------------------------------------------------------
# 全局摄像头工作区域 ROI (像素坐标，针对 1920x1080)
# ---------------------------------------------------------------------------
GLOBAL_ROI = {"u_min": 500, "u_max": 1200, "v_min": 300, "v_max": 800}

# ---------------------------------------------------------------------------
# 扫描路径
# ---------------------------------------------------------------------------
# 紧凑扫描（末端精对准前）
TIGHT_SCAN_OFFSETS = [(0, 0), (0, -25), (0, 25)]

# 扩大扫描（紧凑扫描失败后）
WIDE_SCAN_OFFSETS = [
    (0, 0), (40, 0), (-40, 0),
    (0, 40), (0, -40), (40, 40), (-40, -40)
]

# 默认扫描区域 (用于 scan 命令)
DEFAULT_SCAN_X_RANGE = [140, 200]
DEFAULT_SCAN_Y_RANGE = [-80, 80]
DEFAULT_SCAN_STEPS = 3

# ---------------------------------------------------------------------------
# 放置目标区域坐标（从 marker_map.json 加载，失败则使用内置默认值）
# ---------------------------------------------------------------------------

@dataclass
class Marker:
    key: str
    name: str
    coords: Tuple[float, float]          # 相机对准坐标 (x, y)
    release: Tuple[float, float]         # 实际释放坐标 (考虑偏移)
    color: str
    marker_type: str


@dataclass
class MarkerMap:
    zone: str
    placement_offset_note: str
    markers: Dict[str, Marker] = field(default_factory=dict)
    aliases: Dict[str, str] = field(default_factory=dict)

    def resolve(self, key_or_alias: str) -> Optional[Marker]:
        """通过键名或别名查找 Marker"""
        key = self.aliases.get(key_or_alias, key_or_alias)
        return self.markers.get(key)

    def by_color(self, color: str) -> Optional[Marker]:
        """根据颜色查找对应的 Marker"""
        for m in self.markers.values():
            if m.color == color.lower():
                return m
        return None


def _load_marker_map(path: Path = MARKER_MAP_FILE) -> MarkerMap:
    """加载 marker_map.json，失败时回退到硬编码默认值"""
    defaults = {
        "B": {"name": "蓝色", "coords": (-90, 230), "release": (-60, 230), "color": "blue", "type": "color"},
        "G": {"name": "绿色", "coords": (-30, 230), "release": (0, 230), "color": "green", "type": "color"},
        "R": {"name": "红色", "coords": (30, 230), "release": (60, 230), "color": "red", "type": "color"},
        "Y": {"name": "黄色", "coords": (90, 230), "release": (120, 230), "color": "yellow", "type": "color"},
    }
    aliases = {
        "蓝": "B", "蓝色": "B", "blue": "B",
        "绿": "G", "绿色": "G", "green": "G",
        "红": "R", "红色": "R", "red": "R",
        "黄": "Y", "黄色": "Y", "yellow": "Y",
    }
    if path.exists():
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
            markers = {}
            for k, v in data.get("markers", {}).items():
                markers[k] = Marker(
                    key=k,
                    name=v.get("name", k),
                    coords=tuple(v["coords"]),
                    release=tuple(v["release"]),
                    color=v.get("color", "gray"),
                    marker_type=v.get("type", "unknown"),
                )
            return MarkerMap(
                zone=data.get("zone", "zone1"),
                placement_offset_note=data.get("placement_offset", {}).get("note", ""),
                markers=markers,
                aliases=data.get("aliases", aliases),
            )
        except Exception:
            pass  # fallthrough to defaults

    # 硬编码回退
    markers = {
        k: Marker(key=k, name=v["name"], coords=v["coords"], release=v["release"],
                  color=v["color"], marker_type=v["type"])
        for k, v in defaults.items()
    }
    return MarkerMap(zone="zone1", placement_offset_note="release_x = camera_x + 30",
                     markers=markers, aliases=aliases)


MARKER_MAP: MarkerMap = _load_marker_map()

# ---------------------------------------------------------------------------
# 全局摄像头标定矩阵加载
# ---------------------------------------------------------------------------

def load_affine_matrix(path: Path = CALIB_FILE) -> Optional[np.ndarray]:
    """加载全局摄像头标定的仿射变换矩阵"""
    if not path.exists():
        return None
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
        return np.array(data["affine_matrix"], dtype=np.float32)
    except Exception:
        return None


# ---------------------------------------------------------------------------
# 夹爪偏移加载（若用户有保存自定义偏移）
# ---------------------------------------------------------------------------

def load_grasp_offset(path: Path = GRASP_OFFSET_FILE) -> Tuple[float, float]:
    """加载用户自定义夹爪偏移，失败返回默认值"""
    if not path.exists():
        return GRASP_OFFSET_X, GRASP_OFFSET_Y
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
        return float(data.get("dx", GRASP_OFFSET_X)), float(data.get("dy", GRASP_OFFSET_Y))
    except Exception:
        return GRASP_OFFSET_X, GRASP_OFFSET_Y


# 运行时实际使用的偏移（允许用户通过文件覆盖）
RUNTIME_GRASP_OFFSET = load_grasp_offset()


# ---------------------------------------------------------------------------
# 实用函数
# ---------------------------------------------------------------------------

def ensure_log_dir() -> Path:
    """创建并返回带时间戳的运行日志目录"""
    from datetime import datetime
    run_id = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = LOG_DIR / run_id
    run_dir.mkdir(parents=True, exist_ok=True)
    return run_dir


def clamp(value: float, low: float, high: float) -> float:
    """将值限制在 [low, high] 范围内"""
    return max(low, min(high, value))


def is_within_safe_boundary(x: float, y: float, z: float) -> bool:
    """检查坐标是否在安全工作空间内"""
    return (
        SAFE_X_LIMIT[0] <= x <= SAFE_X_LIMIT[1]
        and SAFE_Y_LIMIT[0] <= y <= SAFE_Y_LIMIT[1]
        and SAFE_Z_LIMIT[0] <= z <= SAFE_Z_LIMIT[1]
    )


def get_safe_speed(current: Optional[List[float]], target: List[float], base_speed: int) -> int:
    """根据距离自动调整安全速度"""
    if current is None or len(current) < 3:
        return min(base_speed, 30)
    dist = ((current[0] - target[0]) ** 2 + (current[1] - target[1]) ** 2 + (current[2] - target[2]) ** 2) ** 0.5
    if dist > 200:
        return min(base_speed, 30)
    elif dist > 100:
        return min(base_speed, 40)
    return min(base_speed, 50)


# ---------------------------------------------------------------------------
# 方便 import 的快捷访问
# ---------------------------------------------------------------------------
__all__ = [
    # 路径
    "PROJECT_ROOT", "LOG_DIR", "CALIB_FILE", "MARKER_MAP_FILE",
    # 硬件
    "ROBOT_PORT", "ROBOT_BAUD",
    "END_CAMERA_INDEX", "END_CAMERA_WIDTH", "END_CAMERA_HEIGHT", "END_CAMERA_FPS",
    "GLOBAL_CAMERA_INDEX", "GLOBAL_CAMERA_WIDTH", "GLOBAL_CAMERA_HEIGHT", "GLOBAL_CAMERA_FPS",
    # 安全
    "SAFE_X_LIMIT", "SAFE_Y_LIMIT", "SAFE_Z_LIMIT",
    "SOFT_X_LIMIT", "SOFT_Y_LIMIT", "SOFT_Z_LIMIT",
    # 运动
    "DEFAULT_SPEED", "HOME_SPEED", "GRIPPER_SPEED", "SAFE_MAX_SPEED",
    "INIT_ANGLES", "HOME_COORDS", "SINGULARITY_J5_THRESH",
    # 高度
    "Z_SCAN", "Z_APPROACH", "Z_GRAB", "Z_RELEASE",
    # 视觉
    "SCALE_Z200", "ALIGN_RATIO", "ALIGN_THRESH_PX", "MAX_ALIGN_ITERS", "ALIGN_SPEED",
    "MIN_OBJECT_AREA", "MAX_OBJECT_AREA", "MIN_GLOBAL_AREA",
    # 颜色
    "DEFAULT_COLOR_RANGES", "GLOBAL_COLOR_RANGES",
    # ROI & 扫描
    "GLOBAL_ROI", "TIGHT_SCAN_OFFSETS", "WIDE_SCAN_OFFSETS",
    "DEFAULT_SCAN_X_RANGE", "DEFAULT_SCAN_Y_RANGE", "DEFAULT_SCAN_STEPS",
    # 偏移
    "GRASP_OFFSET_X", "GRASP_OFFSET_Y", "RUNTIME_GRASP_OFFSET",
    # 数据结构
    "Marker", "MarkerMap", "MARKER_MAP",
    # 函数
    "load_affine_matrix", "load_grasp_offset", "ensure_log_dir",
    "clamp", "is_within_safe_boundary", "get_safe_speed",
]
