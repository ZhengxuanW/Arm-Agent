#!/usr/bin/env python3
"""
robot_service.py
================
机械臂原子化操作 HTTP 服务。

这是 AI 控制机械臂的**唯一入口**。AI 不应该直接 import 本项目中的任何模块，
也不应该读取任何 .py 文件。AI 应该只阅读 MANUAL.md，然后通过 HTTP 调用此服务。

启动方式:
    python3 robot_service.py              # 前台运行
    python3 robot_service.py --daemon     # 后台运行
    python3 robot_service.py --port 5000  # 指定端口

设计原则:
    - 每个 API 都是原子化的（不可再分的最小操作）
    - 返回值始终是 JSON，包含 status 和详细信息
    - 服务启动时自动连接机械臂，服务关闭时自动断开
    - 所有图像通过 URL 返回，可通过浏览器/HTTP 客户端查看
"""

from __future__ import annotations

import argparse
import json
import logging
import os
import signal
import sys
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Optional

import cv2
import numpy as np

# 项目内部模块（服务层使用）
from config import (
    DEFAULT_COLOR_RANGES,
    END_CAMERA_FPS,
    END_CAMERA_HEIGHT,
    END_CAMERA_INDEX,
    END_CAMERA_WIDTH,
    ARUCO_CALIB_RZ,
    GLOBAL_CAMERA_FPS,
    GLOBAL_CAMERA_HEIGHT,
    GLOBAL_CAMERA_INDEX,
    GLOBAL_CAMERA_WIDTH,
    LOG_DIR,
    STATE_LOG_FILE,
    Z_SCAN,
    ensure_log_dir,
)
from motion_service import MotionService
from vision import EndEffectorCamera, GlobalCamera, detect_color_in_frame, save_debug_image

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# 配置
# ---------------------------------------------------------------------------
DEFAULT_PORT = int(os.getenv("ROBOT_SERVICE_PORT", "5000"))
DEFAULT_HOST = os.getenv("ROBOT_SERVICE_HOST", "0.0.0.0")
RUN_DIR = ensure_log_dir()
SCAN_GRID_FILE = Path(__file__).parent / "aruco_scan_grid_selection.json"
DEFAULT_GRID_X = [-150, -125, -100, -75, -50, -25, 0, 25, 50, 75]
DEFAULT_GRID_Y = [-300, -250, -200, -150, -100, -50, 0, 50, 100, 150, 200, 250, 300]

# 全局机械臂实例（服务进程内唯一）
_ms: Optional[MotionService] = None
_end_cam: Optional[EndEffectorCamera] = None
_global_cam: Optional[GlobalCamera] = None
_camera_lock = threading.Lock()
_scan_lock = threading.Lock()


def _analyze_frame_health(frame: np.ndarray) -> dict:
    mean_bgr = frame.mean(axis=(0, 1))
    std_bgr = frame.std(axis=(0, 1))
    mean = {"b": round(float(mean_bgr[0]), 1), "g": round(float(mean_bgr[1]), 1), "r": round(float(mean_bgr[2]), 1)}
    stddev = {"b": round(float(std_bgr[0]), 1), "g": round(float(std_bgr[1]), 1), "r": round(float(std_bgr[2]), 1)}

    solid_green = mean_bgr[1] > 180 and mean_bgr[0] < 80 and mean_bgr[2] < 80 and float(std_bgr.mean()) < 8
    low_detail = float(std_bgr.mean()) < 3
    reason = None
    if solid_green:
        reason = "frame is solid green; camera path is likely invalid"
    elif low_detail:
        reason = "frame has almost no detail; check lens/stream"

    return {
        "mean_bgr": mean,
        "stddev_bgr": stddev,
        "likely_invalid": bool(reason),
        "reason": reason,
    }


def _get_ms() -> MotionService:
    """获取或创建 MotionService 实例"""
    global _ms, _end_cam
    if _ms is None:
        if _end_cam is not None:
            _end_cam.close()
            _end_cam = None
        _ms = MotionService()
        logger.info("[SERVICE] 连接机械臂...")
        if not _ms.connect(warmup=True):
            raise RuntimeError("机械臂连接失败，请检查串口和电源")
        logger.info("[SERVICE] 机械臂已连接")
    return _ms


def _disconnect():
    """断开机械臂连接"""
    global _ms, _end_cam, _global_cam
    if _ms:
        logger.info("[SERVICE] 断开机械臂...")
        _ms.close()
        _ms = None
    if _end_cam:
        logger.info("[SERVICE] 断开末端相机...")
        _end_cam.close()
        _end_cam = None
    if _global_cam:
        logger.info("[SERVICE] 断开全局相机...")
        _global_cam.close()
        _global_cam = None


def _get_end_cam() -> Optional[EndEffectorCamera]:
    """获取 dashboard fallback 末端相机；机械臂连接时优先使用 MotionService.cam。"""
    global _end_cam
    if _ms is not None and _ms.cam is not None:
        return _ms.cam
    if _end_cam is None:
        _end_cam = EndEffectorCamera()
        if not _end_cam.open():
            _end_cam.close()
            _end_cam = None
            return None
    return _end_cam


def _get_global_cam() -> Optional[GlobalCamera]:
    """获取服务内共享的全局相机，避免多进程/多实例抢占设备。"""
    global _global_cam
    if _global_cam is None:
        _global_cam = GlobalCamera()
        if not _global_cam.open():
            _global_cam.close()
            _global_cam = None
            return None
    return _global_cam


# ---------------------------------------------------------------------------
# 状态日志 — 记录所有操作，供 AI 读取
# ---------------------------------------------------------------------------

class StateLogger:
    """
    记录服务所有操作的状态日志。
    
    设计目标：
      - 每次 API 调用都记录到 robot_state.log
      - AI 可以通过 GET /api/v1/log 读取完整历史
      - 服务重启后日志不丢失（追加模式）
      - 包含时间戳、操作、参数、结果摘要
    """

    def __init__(self, log_path: Path = STATE_LOG_FILE):
        self.log_path = log_path
        self._entries: list[dict] = []
        self._load()

    def _load(self):
        """加载已有日志"""
        if self.log_path.exists():
            try:
                text = self.log_path.read_text(encoding="utf-8")
                for line in text.strip().split("\n"):
                    if line.strip():
                        self._entries.append(json.loads(line))
            except Exception as e:
                logger.warning("[STATE_LOG] 加载历史日志失败: %s", e)

    def record(self, action: str, params: dict, result: dict):
        """记录一次操作"""
        entry = {
            "timestamp": datetime.now().isoformat(),
            "action": action,
            "params": self._safe_params(params),
            "result_summary": self._safe_result(result),
        }
        self._entries.append(entry)
        
        # 追加写入文件
        try:
            with open(self.log_path, "a", encoding="utf-8") as f:
                f.write(json.dumps(entry, ensure_ascii=False) + "\n")
        except Exception as e:
            logger.warning("[STATE_LOG] 写入日志失败: %s", e)

    def get_recent(self, limit: int = 50) -> list[dict]:
        """获取最近 N 条日志"""
        return self._entries[-limit:]

    def get_all(self) -> list[dict]:
        """获取全部日志"""
        return self._entries.copy()

    def clear(self):
        """清空日志（谨慎使用）"""
        self._entries = []
        if self.log_path.exists():
            self.log_path.unlink()

    @staticmethod
    def _safe_params(params: dict) -> dict:
        """提取关键参数，避免日志过大"""
        safe = {}
        for k, v in params.items():
            if k in ("x", "y", "z", "rx", "ry", "rz", "speed", "color", "action", "steps"):
                safe[k] = v
        return safe

    @staticmethod
    def _safe_result(result: dict) -> dict:
        """提取结果摘要，避免日志过大"""
        summary = {"status": result.get("status")}
        
        if "found" in result:
            summary["found"] = result["found"]
        if "world_xy" in result:
            summary["world_xy"] = result["world_xy"]
        if "offset_from_center" in result:
            summary["offset_from_center"] = result["offset_from_center"]
        if "pixel" in result:
            summary["pixel"] = result["pixel"]
        if "result" in result and isinstance(result["result"], dict):
            summary["move_success"] = result["result"].get("success")
            if "actual" in result["result"]:
                summary["actual"] = result["result"]["actual"]
        if "reason" in result:
            summary["reason"] = result["reason"]
        
        return summary


# 全局状态日志实例
_state_logger = StateLogger()


# ---------------------------------------------------------------------------
# API 实现（原子操作）
# ---------------------------------------------------------------------------

def _api_wrapper(action: str, params: dict, result: dict) -> dict:
    """包装 API 调用，自动记录到状态日志"""
    _state_logger.record(action, params, result)
    return result

def api_init(params: dict) -> dict:
    """初始化：回零 + 张开夹爪"""
    ms = _get_ms()
    ms.gripper_open()
    time.sleep(0.3)
    result = ms.go_init(speed=10)
    time.sleep(1.0)
    return {"status": "ok", "action": "init", "result": result}


def api_shutdown(params: dict) -> dict:
    """安全关闭：回零 + 张开夹爪 + 断开连接"""
    try:
        ms = _get_ms()
        ms.gripper_open()
        time.sleep(0.3)
        result = ms.go_init(speed=10)
        time.sleep(1.0)
        _disconnect()
        return {"status": "ok", "action": "shutdown", "result": result}
    except Exception as e:
        _disconnect()
        return {"status": "error", "action": "shutdown", "reason": str(e)}


def api_move(params: dict) -> dict:
    """移动到指定坐标"""
    ms = _get_ms()
    x = params.get("x")
    y = params.get("y")
    z = params.get("z", Z_SCAN)
    rx = params.get("rx", -180)
    ry = params.get("ry", 0)
    rz = params.get("rz", ARUCO_CALIB_RZ)
    speed = params.get("speed", 20)
    wait = params.get("wait", True)

    if x is None or y is None:
        return {"status": "error", "action": "move", "reason": "缺少 x 或 y 参数"}

    result = ms.move_to(x=x, y=y, z=z, rx=rx, ry=ry, rz=rz, speed=speed, wait=wait)
    if wait:
        time.sleep(0.3)

    return {
        "status": "ok",
        "action": "move",
        "target": [x, y, z, rx, ry, rz],
        "result": result,
    }


def api_capture(params: dict) -> dict:
    """拍照并返回图像 URL"""
    ms = _get_ms()
    with _camera_lock:
        frame = ms.capture()
    if frame is None:
        return {"status": "error", "action": "capture", "reason": "拍照失败"}

    timestamp = datetime.now().strftime("%H%M%S_%f")[:-3]
    filename = f"cap_{timestamp}.jpg"
    filepath = RUN_DIR / filename
    cv2.imwrite(str(filepath), frame)

    # 构建可访问的 URL
    url = f"/images/{RUN_DIR.name}/{filename}"

    return {
        "status": "ok",
        "action": "capture",
        "image_path": str(filepath),
        "image_url": url,
        "shape": list(frame.shape),
    }


def api_detect(params: dict) -> dict:
    """颜色检测"""
    color = params.get("color")
    if not color:
        return {"status": "error", "action": "detect", "reason": "缺少 color 参数"}

    ms = _get_ms()
    with _camera_lock:
        frame = ms.capture()
    if frame is None:
        return {"status": "error", "action": "detect", "reason": "拍照失败"}

    result = detect_color_in_frame(
        frame=frame,
        color=color,
        color_ranges=DEFAULT_COLOR_RANGES,
        return_debug=True,
    )

    # 保存调试图
    if result.debug_image is not None:
        debug_name = f"detect_{color}_{datetime.now().strftime('%H%M%S')}.jpg"
        debug_path = RUN_DIR / debug_name
        cv2.imwrite(str(debug_path), result.debug_image)
        debug_url = f"/images/{RUN_DIR.name}/{debug_name}"
    else:
        debug_path = None
        debug_url = None

    if not result.found:
        return {
            "status": "ok",
            "action": "detect",
            "found": False,
            "color": color,
            "reason": result.reason,
            "debug_image_url": debug_url,
        }

    return {
        "status": "ok",
        "action": "detect",
        "found": True,
        "color": color,
        "pixel": result.pixel_center,
        "area": result.area,
        "offset_from_center": result.offset_from_center,
        "debug_image_url": debug_url,
    }


def api_gripper(params: dict) -> dict:
    """夹爪控制"""
    action = params.get("action")
    if action not in ("open", "close"):
        return {"status": "error", "action": "gripper", "reason": "action 必须是 'open' 或 'close'"}

    ms = _get_ms()
    if action == "open":
        ms.gripper_open()
        time.sleep(0.3)
        return {"status": "ok", "action": "gripper", "state": "open"}
    else:
        ms.gripper_close()
        time.sleep(0.5)
        return {"status": "ok", "action": "gripper", "state": "close"}


def api_position(params: dict) -> dict:
    """获取当前位置"""
    ms = _get_ms()
    coords = ms.robot.get_coords()
    angles = ms.robot.get_angles()
    return {
        "status": "ok",
        "action": "position",
        "coords": coords,
        "angles": angles,
    }


def api_status(params: dict) -> dict:
    """获取服务状态"""
    global _ms
    return {
        "status": "ok",
        "action": "status",
        "connected": _ms is not None and _ms._connected,
        "log_dir": str(RUN_DIR),
    }


def api_global_capture(params: dict) -> dict:
    """全局摄像头(BRIO)拍照"""
    cam = _get_global_cam()
    if cam is None:
        return {"status": "error", "action": "global_capture", "reason": "全局摄像头打开失败"}

    with _camera_lock:
        frame = cam.grab(retries=5)
    if frame is None:
        return {"status": "error", "action": "global_capture", "reason": "拍照失败"}

    timestamp = datetime.now().strftime("%H%M%S_%f")[:-3]
    filename = f"global_cap_{timestamp}.jpg"
    filepath = RUN_DIR / filename
    cv2.imwrite(str(filepath), frame)

    url = f"/images/{RUN_DIR.name}/{filename}"

    return {
        "status": "ok",
        "action": "global_capture",
        "image_path": str(filepath),
        "image_url": url,
        "shape": list(frame.shape),
    }


def api_global_detect(params: dict) -> dict:
    """
    全局摄像头检测颜色并返回世界坐标。
    需要当前 4-ArUco 全局摄像头标定文件 global_camera_calib_4aruco.json。
    """
    from vision import detect_color_global, pixel_to_world

    color = params.get("color")
    if not color:
        return {"status": "error", "action": "global_detect", "reason": "缺少 color 参数"}

    cam = _get_global_cam()
    if cam is None:
        return {"status": "error", "action": "global_detect", "reason": "全局摄像头打开失败"}

    with _camera_lock:
        frame = cam.grab(retries=5)
    if frame is None:
        return {"status": "error", "action": "global_detect", "reason": "拍照失败"}

    # 保存原始图
    timestamp = datetime.now().strftime("%H%M%S_%f")[:-3]
    raw_name = f"global_raw_{timestamp}.jpg"
    raw_path = RUN_DIR / raw_name
    cv2.imwrite(str(raw_path), frame)

    # 颜色检测
    result = detect_color_global(frame, color, apply_roi=True, return_debug=True)

    # 保存调试图
    if result.debug_image is not None:
        debug_name = f"global_detect_{color}_{timestamp}.jpg"
        debug_path = RUN_DIR / debug_name
        cv2.imwrite(str(debug_path), result.debug_image)
        debug_url = f"/images/{RUN_DIR.name}/{debug_name}"
    else:
        debug_url = None

    if not result.found:
        return {
            "status": "ok",
            "action": "global_detect",
            "found": False,
            "color": color,
            "reason": result.reason,
            "raw_image_url": f"/images/{RUN_DIR.name}/{raw_name}",
            "debug_image_url": debug_url,
        }

    # 转换为世界坐标
    world_xy = None
    if result.pixel_center:
        world_xy = pixel_to_world(*result.pixel_center)

    return {
        "status": "ok",
        "action": "global_detect",
        "found": True,
        "color": color,
        "pixel": result.pixel_center,
        "world_xy": world_xy,
        "area": result.area,
        "raw_image_url": f"/images/{RUN_DIR.name}/{raw_name}",
        "debug_image_url": debug_url,
    }


# 指令路由表
API_ROUTES = {
    "init": api_init,
    "shutdown": api_shutdown,
    "move": api_move,
    "capture": api_capture,
    "global_capture": api_global_capture,
    "global_detect": api_global_detect,
    "detect": api_detect,
    "gripper": api_gripper,
    "position": api_position,
    "status": api_status,
}


ROBOT_DASHBOARD_HTML = """
<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Robot Service Dashboard</title>
  <style>
    body { margin: 0; font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif; background: #101216; color: #e8edf2; }
    header { padding: 18px 24px; border-bottom: 1px solid #2b3038; background: #171a21; }
    h1 { margin: 0; font-size: 22px; }
    .sub { color: #9aa4b2; margin-top: 6px; }
    main { padding: 20px; display: grid; grid-template-columns: repeat(auto-fit, minmax(360px, 1fr)); gap: 20px; }
    .card { background: #171a21; border: 1px solid #2b3038; border-radius: 14px; overflow: hidden; box-shadow: 0 12px 30px rgba(0,0,0,.25); }
    .card h2 { margin: 0; padding: 14px 16px; font-size: 18px; border-bottom: 1px solid #2b3038; }
    .feed { background: #08090b; aspect-ratio: 16 / 9; display: flex; align-items: center; justify-content: center; }
    .feed img { width: 100%; height: 100%; object-fit: contain; }
    .meta { padding: 12px 16px; font-family: ui-monospace, SFMono-Regular, Menlo, monospace; font-size: 13px; line-height: 1.6; color: #c9d1d9; white-space: pre-wrap; }
    .ok { color: #4ade80; font-weight: 700; }
    .bad { color: #fb7185; font-weight: 700; }
    .wide { grid-column: 1 / -1; }
    .controls { display: flex; gap: 10px; flex-wrap: wrap; padding: 14px 16px; border-bottom: 1px solid #2b3038; }
    button { border: 1px solid #394150; border-radius: 9px; padding: 8px 12px; color: #e8edf2; background: #222833; cursor: pointer; }
    button:hover { background: #2d3544; }
    button.primary { border-color: #2563eb; background: #1d4ed8; }
    button.danger { border-color: #9f1239; background: #881337; }
    label.param { display: flex; align-items: center; gap: 6px; color: #c9d1d9; font-size: 13px; }
    input.param { width: 74px; border: 1px solid #394150; border-radius: 8px; padding: 7px 8px; color: #e8edf2; background: #0f172a; }
    .grid-wrap { padding: 16px; overflow: auto; }
    .grid { display: grid; gap: 6px; width: max-content; }
    .cell { min-width: 74px; min-height: 42px; border: 1px solid #394150; border-radius: 8px; background: #111827; color: #c9d1d9; font-family: ui-monospace, SFMono-Regular, Menlo, monospace; }
    .cell.selected { background: #14532d; border-color: #22c55e; color: #dcfce7; }
    .axis { display: flex; align-items: center; justify-content: center; color: #9aa4b2; font-family: ui-monospace, SFMono-Regular, Menlo, monospace; font-size: 12px; }
    .hint { padding: 0 16px 14px; color: #9aa4b2; font-size: 13px; }
    .scan-results { padding: 12px 16px; display: grid; gap: 10px; }
    .scan-item { border: 1px solid #2b3038; border-radius: 10px; padding: 10px; background: #111827; }
    .scan-item img { max-width: 240px; max-height: 180px; display: block; margin-top: 8px; background: #08090b; }
    .position { padding: 12px 16px; display: grid; grid-template-columns: repeat(6, minmax(80px, 1fr)); gap: 10px; }
    .pos-cell { border: 1px solid #2b3038; border-radius: 10px; padding: 10px; background: #111827; }
    .pos-label { color: #9aa4b2; font-size: 12px; }
    .pos-value { margin-top: 4px; font-family: ui-monospace, SFMono-Regular, Menlo, monospace; font-size: 18px; color: #e8edf2; }
  </style>
</head>
<body>
  <header>
    <h1>Robot Service Dashboard</h1>
    <div class="sub">由 robot_service 共享相机实例，避免 dashboard 与测试服务抢占摄像头。</div>
  </header>
  <main>
    <section class="card">
      <h2>末端相机 End-Effector <span id="state-end"></span></h2>
      <div class="feed"><img src="/dashboard/video/end" alt="end camera feed"></div>
      <div class="meta" id="meta-end">loading...</div>
    </section>
    <section class="card">
      <h2>俯视相机 Overhead <span id="state-global"></span></h2>
      <div class="feed"><img src="/dashboard/video/global" alt="global camera feed"></div>
      <div class="meta" id="meta-global">loading...</div>
    </section>
    <section class="card wide">
      <h2>当前位置 Current Position</h2>
      <div class="position">
        <div class="pos-cell"><div class="pos-label">X</div><div class="pos-value" id="pos-x">--</div></div>
        <div class="pos-cell"><div class="pos-label">Y</div><div class="pos-value" id="pos-y">--</div></div>
        <div class="pos-cell"><div class="pos-label">Z</div><div class="pos-value" id="pos-z">--</div></div>
        <div class="pos-cell"><div class="pos-label">RX</div><div class="pos-value" id="pos-rx">--</div></div>
        <div class="pos-cell"><div class="pos-label">RY</div><div class="pos-value" id="pos-ry">--</div></div>
        <div class="pos-cell"><div class="pos-label">RZ</div><div class="pos-value" id="pos-rz-val">--</div></div>
      </div>
      <div class="meta" id="pos-meta">loading...</div>
    </section>
    <section class="card wide">
      <h2>ArUco 初始扫描网格</h2>
      <div class="controls">
        <button class="primary" onclick="saveGrid()">保存选区</button>
        <button onclick="loadGrid()">重新加载</button>
        <button onclick="clearGrid()">清空</button>
        <button onclick="selectReachablePreset()">推荐可达点</button>
        <label class="param">speed <input class="param" id="scan-speed" type="number" min="1" max="80" value="15"></label>
        <label class="param">rz <input class="param" id="scan-rz" type="number" min="-180" max="180" value="-45"></label>
        <button class="danger" onclick="runCoarseScan()">按选区粗扫</button>
      </div>
      <div class="hint">流程：每个选中格子执行 init -> move(z=150, rz=-45) -> capture -> init。当前 ArUco 标定只使用这个已验证姿态。</div>
      <div class="grid-wrap"><div id="scan-grid" class="grid"></div></div>
      <div class="meta" id="grid-meta">loading...</div>
      <div class="scan-results" id="scan-results"></div>
    </section>
  </main>
  <script>
    const gridX = [-150, -125, -100, -75, -50, -25, 0, 25, 50, 75];
    const gridY = [-300, -250, -200, -150, -100, -50, 0, 50, 100, 150, 200, 250, 300];
    const selected = new Set();

    function keyOf(x, y) { return `${x},${y}`; }
    function parseKey(key) { const [x, y] = key.split(',').map(Number); return {x, y}; }

    function renderGrid() {
      const grid = document.getElementById('scan-grid');
      grid.style.gridTemplateColumns = `70px repeat(${gridY.length}, 74px)`;
      grid.innerHTML = '<div class="axis">X \\ Y</div>' + gridY.map(y => `<div class="axis">Y=${y}</div>`).join('');
      for (const x of gridX) {
        grid.insertAdjacentHTML('beforeend', `<div class="axis">X=${x}</div>`);
        for (const y of gridY) {
          const key = keyOf(x, y);
          const cls = selected.has(key) ? 'cell selected' : 'cell';
          grid.insertAdjacentHTML('beforeend', `<button class="${cls}" data-key="${key}" onclick="toggleCell('${key}')">${x}<br>${y}</button>`);
        }
      }
      updateGridMeta();
    }

    function toggleCell(key) {
      if (selected.has(key)) selected.delete(key); else selected.add(key);
      renderGrid();
    }

    function updateGridMeta(extra = '') {
      const points = [...selected].map(parseKey).sort((a, b) => a.x - b.x || a.y - b.y);
      document.getElementById('grid-meta').textContent = JSON.stringify({count: points.length, speed: getScanSpeed(), rz: getScanRz(), points, note: extra}, null, 2);
    }

    function clearGrid() {
      selected.clear();
      renderGrid();
    }

    function selectReachablePreset() {
      selected.clear();
      [[75,-50], [50,-100], [50,100], [0,100], [-50,100], [-100,150], [-150,150], [-150,-150]].forEach(([x, y]) => selected.add(keyOf(x, y)));
      renderGrid();
    }

    async function loadGrid() {
      const resp = await fetch('/dashboard/api/scan-grid');
      const data = await resp.json();
      selected.clear();
      for (const p of data.points || []) selected.add(keyOf(p.x, p.y));
      document.getElementById('scan-speed').value = data.speed || 15;
      document.getElementById('scan-rz').value = data.rz ?? 45;
      renderGrid();
      updateGridMeta('loaded from server');
    }

    async function saveGrid() {
      const points = [...selected].map(parseKey);
      const resp = await fetch('/dashboard/api/scan-grid', {
        method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify({points, speed: getScanSpeed(), rz: getScanRz()})
      });
      updateGridMeta(await resp.text());
    }

    function getScanSpeed() {
      return Number(document.getElementById('scan-speed').value || 15);
    }

    function getScanRz() {
      return Number(document.getElementById('scan-rz').value || 45);
    }

    async function runCoarseScan() {
      if (selected.size === 0) return alert('先选择至少一个扫描格子');
      if (!confirm(`将扫描 ${selected.size} 个点。机械臂会移动，确认工作区安全？`)) return;
      await saveGrid();
      document.getElementById('scan-results').innerHTML = '<div class="scan-item">粗扫运行中，请等待...</div>';
      const resp = await fetch('/dashboard/api/coarse-scan', {method: 'POST'});
      const data = await resp.json();
      const items = (data.results || []).map(item => {
        const img = item.image_url ? `<img src="${item.image_url}" alt="capture">` : '';
        return `<div class="scan-item"><pre>${JSON.stringify(item, null, 2)}</pre>${img}</div>`;
      }).join('');
      document.getElementById('scan-results').innerHTML = `<div class="meta">${JSON.stringify({status: data.status, count: data.count, run_dir: data.run_dir}, null, 2)}</div>${items}`;
    }

    async function refresh() {
      const resp = await fetch('/dashboard/api/status');
      const data = await resp.json();
      for (const cam of data.cameras) {
        const state = document.getElementById(`state-${cam.key}`);
        state.textContent = cam.opened ? 'OPEN' : 'CLOSED';
        state.className = cam.opened ? 'ok' : 'bad';
        document.getElementById(`meta-${cam.key}`).textContent = JSON.stringify(cam, null, 2);
      }
    }

    async function refreshPosition() {
      try {
        const resp = await fetch('/api/v1/position', {method: 'POST'});
        const data = await resp.json();
        const coords = data.coords || [];
        ['x', 'y', 'z', 'rx', 'ry', 'rz-val'].forEach((name, idx) => {
          const el = document.getElementById(`pos-${name}`);
          const value = coords[idx];
          el.textContent = Number.isFinite(value) ? Number(value).toFixed(1) : '--';
        });
        document.getElementById('pos-meta').textContent = JSON.stringify(data, null, 2);
      } catch (err) {
        document.getElementById('pos-meta').textContent = String(err);
      }
    }
    renderGrid();
    loadGrid();
    refresh();
    refreshPosition();
    setInterval(refresh, 1500);
    setInterval(refreshPosition, 1500);
  </script>
</body>
</html>
"""


def _read_dashboard_frame(key: str) -> tuple[Optional[np.ndarray], Optional[str]]:
    try:
        with _camera_lock:
            if key == "end":
                if _ms is not None and _ms.cam is not None:
                    frame = _ms.capture()
                else:
                    cam = _get_end_cam()
                    frame = cam.grab(retries=5) if cam else None
            elif key == "global":
                cam = _get_global_cam()
                frame = cam.grab(retries=5) if cam else None
            else:
                return None, "unknown camera"
    except Exception as exc:
        return None, str(exc)

    if frame is None:
        return None, "failed to read frame"
    return frame, None


def _dashboard_camera_status(key: str) -> dict:
    if key == "end":
        expected = {"width": END_CAMERA_WIDTH, "height": END_CAMERA_HEIGHT, "fps": END_CAMERA_FPS}
        source = END_CAMERA_INDEX
        label = "末端相机 End-Effector"
    else:
        expected = {"width": GLOBAL_CAMERA_WIDTH, "height": GLOBAL_CAMERA_HEIGHT, "fps": GLOBAL_CAMERA_FPS}
        source = GLOBAL_CAMERA_INDEX
        label = "俯视相机 Overhead"

    frame, error = _read_dashboard_frame(key)
    if frame is None:
        return {
            "key": key,
            "label": label,
            "source": source,
            "expected": expected,
            "opened": False,
            "actual": {"width": None, "height": None},
            "frame_health": None,
            "last_error": error,
        }

    height, width = frame.shape[:2]
    health = _analyze_frame_health(frame)
    return {
        "key": key,
        "label": label,
        "source": source,
        "expected": expected,
        "opened": not health["likely_invalid"],
        "actual": {"width": width, "height": height},
        "frame_health": health,
        "last_error": health["reason"],
    }


def _placeholder(label: str, error: Optional[str]) -> np.ndarray:
    frame = np.zeros((360, 640, 3), dtype=np.uint8)
    cv2.putText(frame, label, (30, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (220, 220, 220), 2)
    cv2.putText(frame, error or "camera unavailable", (30, 200), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (80, 120, 255), 2)
    return frame


def _dashboard_mjpeg(key: str):
    label = "末端相机 End-Effector" if key == "end" else "俯视相机 Overhead"
    while True:
        frame, error = _read_dashboard_frame(key)
        if frame is None:
            frame = _placeholder(label, error)
            time.sleep(0.5)
        ok, encoded = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        if ok:
            yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + encoded.tobytes() + b"\r\n"
        time.sleep(0.03)


def _normalize_scan_points(points: list[dict]) -> list[dict]:
    normalized = []
    seen = set()
    allowed_x = set(DEFAULT_GRID_X)
    allowed_y = set(DEFAULT_GRID_Y)
    for item in points:
        try:
            x = int(item["x"])
            y = int(item["y"])
        except (KeyError, TypeError, ValueError):
            continue
        if x not in allowed_x or y not in allowed_y:
            continue
        key = (x, y)
        if key in seen:
            continue
        seen.add(key)
        normalized.append({"x": x, "y": y})
    return normalized


def _scan_settings_from_body(body: dict) -> tuple[int, float]:
    try:
        speed = int(body.get("speed", 15))
    except (TypeError, ValueError):
        speed = 15
    try:
        rz = float(body.get("rz", ARUCO_CALIB_RZ))
    except (TypeError, ValueError):
        rz = ARUCO_CALIB_RZ
    return max(1, min(80, speed)), max(-180.0, min(180.0, rz))


def _load_scan_grid() -> list[dict]:
    if not SCAN_GRID_FILE.exists():
        return []
    try:
        data = json.loads(SCAN_GRID_FILE.read_text(encoding="utf-8"))
        return _normalize_scan_points(data.get("points", []))
    except Exception:
        return []


def _load_scan_grid_payload() -> dict:
    if not SCAN_GRID_FILE.exists():
        return {"points": [], "speed": 15, "rz": ARUCO_CALIB_RZ}
    try:
        data = json.loads(SCAN_GRID_FILE.read_text(encoding="utf-8"))
        speed, rz = _scan_settings_from_body(data)
        return {"points": _normalize_scan_points(data.get("points", [])), "speed": speed, "rz": rz, "updated_at": data.get("updated_at")}
    except Exception:
        return {"points": [], "speed": 15, "rz": ARUCO_CALIB_RZ}


def _save_scan_grid(points: list[dict], speed: int = 15, rz: float = ARUCO_CALIB_RZ) -> dict:
    normalized = _normalize_scan_points(points)
    payload = {
        "updated_at": datetime.now().isoformat(),
        "z": Z_SCAN,
        "rx": -180,
        "ry": 0,
        "rz": rz,
        "speed": speed,
        "points": normalized,
    }
    SCAN_GRID_FILE.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")
    return payload


def _run_coarse_scan(points: list[dict], speed: int = 15, rz: float = 45.0) -> dict:
    if not points:
        return {"status": "error", "reason": "no scan points selected", "results": []}
    if not _scan_lock.acquire(blocking=False):
        return {"status": "error", "reason": "scan already running", "results": []}

    results = []
    try:
        ms = _get_ms()
        for idx, point in enumerate(points, start=1):
            x = float(point["x"])
            y = float(point["y"])
            item = {"index": idx, "x": x, "y": y, "z": Z_SCAN, "speed": speed, "rz": rz, "image_url": None, "image_path": None}
            try:
                init_result = ms.go_init(speed=speed, wait=True)
                time.sleep(0.8)
                move_result = ms.move_to(x=x, y=y, z=Z_SCAN, rx=-180, ry=0, rz=rz, speed=speed, wait=True)
                item["init"] = init_result
                item["move"] = move_result
                if move_result.get("success"):
                    time.sleep(0.8)
                    with _camera_lock:
                        frame = ms.capture()
                    if frame is not None:
                        filename = f"aruco_coarse_{idx:02d}_x{int(x)}_y{int(y)}_{datetime.now().strftime('%H%M%S_%f')[:-3]}.jpg"
                        filepath = RUN_DIR / filename
                        cv2.imwrite(str(filepath), frame)
                        item["image_path"] = str(filepath)
                        item["image_url"] = f"/images/{RUN_DIR.name}/{filename}"
                    else:
                        item["capture_error"] = "failed to capture end camera frame"
                results.append(item)
            except Exception as exc:
                item["error"] = str(exc)
                results.append(item)
            finally:
                try:
                    ms.go_init(speed=speed, wait=True)
                    time.sleep(0.5)
                except Exception as exc:
                    item["final_init_error"] = str(exc)
        return {"status": "ok", "count": len(results), "run_dir": str(RUN_DIR), "results": results}
    finally:
        _scan_lock.release()


# ---------------------------------------------------------------------------
# Flask HTTP 服务
# ---------------------------------------------------------------------------

def create_app():
    """创建 Flask 应用"""
    try:
        from flask import Flask, Response, request, jsonify, render_template_string, send_from_directory
    except ImportError:
        print("[ERROR] Flask 未安装，请先运行: pip install flask")
        sys.exit(1)

    app = Flask(__name__)

    @app.route("/dashboard")
    def dashboard():
        """内置相机 dashboard，复用 robot_service 的相机实例。"""
        return render_template_string(ROBOT_DASHBOARD_HTML)

    @app.route("/dashboard/api/status")
    def dashboard_status():
        """返回共享相机状态。"""
        return jsonify({"cameras": [_dashboard_camera_status("end"), _dashboard_camera_status("global")]})

    @app.route("/dashboard/video/<key>")
    def dashboard_video(key: str):
        """返回共享相机 MJPEG 视频流。"""
        if key not in ("end", "global"):
            return jsonify({"status": "error", "reason": "unknown camera"}), 404
        return Response(_dashboard_mjpeg(key), mimetype="multipart/x-mixed-replace; boundary=frame")

    @app.route("/dashboard/api/scan-grid", methods=["GET", "POST"])
    def dashboard_scan_grid():
        """读取或保存 ArUco 初始扫描网格。"""
        if request.method == "POST":
            body = request.get_json(silent=True) or {}
            speed, rz = _scan_settings_from_body(body)
            payload = _save_scan_grid(body.get("points", []), speed=speed, rz=rz)
        else:
            saved = _load_scan_grid_payload()
            payload = {
                "updated_at": saved.get("updated_at"),
                "z": Z_SCAN,
                "rx": -180,
                "ry": 0,
                "rz": saved.get("rz", 45.0),
                "speed": saved.get("speed", 15),
                "points": saved.get("points", []),
            }
        payload["grid"] = {"x": DEFAULT_GRID_X, "y": DEFAULT_GRID_Y}
        return jsonify(payload)

    @app.route("/dashboard/api/coarse-scan", methods=["POST"])
    def dashboard_coarse_scan():
        """按已保存网格执行一次粗扫，供人眼快速判断。"""
        body = request.get_json(silent=True) or {}
        if body.get("points"):
            speed, rz = _scan_settings_from_body(body)
            points = _normalize_scan_points(body.get("points", []))
        else:
            saved = _load_scan_grid_payload()
            points = saved.get("points", [])
            speed = saved.get("speed", 15)
            rz = saved.get("rz", 45.0)
        result = _run_coarse_scan(points, speed=speed, rz=rz)
        status_code = 200 if result.get("status") == "ok" else 400
        return jsonify(result), status_code

    # 图像文件服务（使用 /images/ 而非 /static/，避免与 Flask 默认静态路由冲突）
    @app.route("/images/<path:path>")
    def serve_static(path):
        """提供日志目录中的图像文件"""
        parts = path.split("/", 1)
        if len(parts) == 2:
            run_id, filename = parts
            directory = LOG_DIR / run_id
            if directory.exists():
                return send_from_directory(str(directory), filename)
        return jsonify({"status": "error", "reason": "file not found"}), 404

    @app.route("/api/v1/<action>", methods=["POST", "GET"])
    def handle_api(action):
        """统一 API 入口"""
        if action not in API_ROUTES:
            return jsonify({
                "status": "error",
                "reason": f"未知操作: {action}",
                "available": list(API_ROUTES.keys()),
            }), 404

        # 获取参数
        if request.method == "POST":
            if request.is_json:
                params = request.get_json(silent=True) or {}
            else:
                params = request.form.to_dict()
        else:
            params = request.args.to_dict()

        # 尝试转换数字参数
        for key in ["x", "y", "z", "rx", "ry", "rz", "speed", "steps"]:
            if key in params:
                try:
                    if key == "speed" or key == "steps":
                        params[key] = int(params[key])
                    else:
                        params[key] = float(params[key])
                except (ValueError, TypeError):
                    pass

        logger.info("[API] %s params=%s", action, params)

        try:
            result = API_ROUTES[action](params)
            _state_logger.record(action, params, result)
            status_code = 200 if result.get("status") == "ok" else 500
            return jsonify(result), status_code
        except Exception as e:
            error_result = {"status": "error", "action": action, "reason": str(e)}
            _state_logger.record(action, params, error_result)
            logger.exception("[API] %s 执行失败", action)
            return jsonify(error_result), 500

    @app.route("/api/v1/log", methods=["GET", "POST"])
    def api_log():
        """读取操作日志"""
        if request.method == "POST" and request.is_json:
            body = request.get_json(silent=True) or {}
            limit = body.get("limit", 50)
            clear = body.get("clear", False)
        else:
            limit = request.args.get("limit", 50, type=int)
            clear = request.args.get("clear", "false").lower() == "true"
        
        if clear:
            _state_logger.clear()
            return jsonify({"status": "ok", "action": "log", "cleared": True})
        
        entries = _state_logger.get_recent(limit=limit)
        return jsonify({
            "status": "ok",
            "action": "log",
            "count": len(entries),
            "entries": entries,
        })

    @app.route("/api/v1/", methods=["GET"])
    def api_index():
        """API 索引页"""
        return jsonify({
            "service": "robot_service",
            "version": "1.0",
            "status": "running",
            "endpoints": {
                name: {
                    "method": "POST",
                    "description": func.__doc__.strip().split("\n")[0] if func.__doc__ else "",
                }
                for name, func in API_ROUTES.items()
            },
        })

    @app.errorhandler(404)
    def not_found(e):
        return jsonify({"status": "error", "reason": "endpoint not found"}), 404

    return app


# ---------------------------------------------------------------------------
# 主函数
# ---------------------------------------------------------------------------

def run_service(host: str = DEFAULT_HOST, port: int = DEFAULT_PORT, daemon: bool = False):
    """启动服务"""
    app = create_app()

    # 预连接机械臂（如果失败也继续运行，等待 init 调用）
    try:
        _get_ms()
        logger.info("[SERVICE] 机械臂预连接成功")
    except Exception as e:
        logger.warning("[SERVICE] 机械臂预连接失败: %s，等待 init 调用时重试", e)

    if daemon:
        # 后台运行
        import daemon
        import lockfile

        pidfile = lockfile.FileLock("/tmp/robot_service.pid")
        logfile = open("/tmp/robot_service.log", "a+")
        context = daemon.DaemonContext(
            pidfile=pidfile,
            stdout=logfile,
            stderr=logfile,
            working_directory=str(Path(__file__).parent),
        )
        with context:
            app.run(host=host, port=port, debug=False, use_reloader=False)
    else:
        print(f"=" * 60)
        print(f"Robot Service 启动")
        print(f"=" * 60)
        print(f"监听地址: http://{host}:{port}")
        print(f"API 入口: http://{host}:{port}/api/v1/")
        print(f"Dashboard: http://{host}:{port}/dashboard")
        print(f"日志目录: {RUN_DIR}")
        print()
        print("可用 API:")
        for name in API_ROUTES:
            print(f"  POST /api/v1/{name}")
        print()
        print("按 Ctrl+C 停止服务")
        print("=" * 60)

        try:
            app.run(host=host, port=port, debug=False, use_reloader=False)
        finally:
            _disconnect()


def main():
    parser = argparse.ArgumentParser(description="Robot Service - 机械臂 HTTP 服务")
    parser.add_argument("--host", default=DEFAULT_HOST, help=f"绑定地址 (默认: {DEFAULT_HOST})")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help=f"端口 (默认: {DEFAULT_PORT})")
    parser.add_argument("--daemon", action="store_true", help="后台运行")
    args = parser.parse_args()

    # 配置日志
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(message)s",
    )

    run_service(host=args.host, port=args.port, daemon=args.daemon)


if __name__ == "__main__":
    main()
