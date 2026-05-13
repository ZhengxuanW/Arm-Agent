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
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Optional

import cv2
import numpy as np

# 项目内部模块（服务层使用）
from config import (
    DEFAULT_COLOR_RANGES,
    DEFAULT_SCAN_STEPS,
    DEFAULT_SCAN_X_RANGE,
    DEFAULT_SCAN_Y_RANGE,
    END_CAMERA_FPS,
    END_CAMERA_HEIGHT,
    END_CAMERA_INDEX,
    END_CAMERA_WIDTH,
    LOG_DIR,
    STATE_LOG_FILE,
    ensure_log_dir,
)
from motion_service import MotionService
from vision import detect_color_in_frame, save_debug_image

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# 配置
# ---------------------------------------------------------------------------
DEFAULT_PORT = int(os.getenv("ROBOT_SERVICE_PORT", "5000"))
DEFAULT_HOST = os.getenv("ROBOT_SERVICE_HOST", "0.0.0.0")
RUN_DIR = ensure_log_dir()

# 全局机械臂实例（服务进程内唯一）
_ms: Optional[MotionService] = None


def _get_ms() -> MotionService:
    """获取或创建 MotionService 实例"""
    global _ms
    if _ms is None:
        _ms = MotionService()
        logger.info("[SERVICE] 连接机械臂...")
        if not _ms.connect(warmup=True):
            raise RuntimeError("机械臂连接失败，请检查串口和电源")
        logger.info("[SERVICE] 机械臂已连接")
    return _ms


def _disconnect():
    """断开机械臂连接"""
    global _ms
    if _ms:
        logger.info("[SERVICE] 断开机械臂...")
        _ms.close()
        _ms = None


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
    z = params.get("z", 200)
    rx = params.get("rx", -180)
    ry = params.get("ry", 0)
    rz = params.get("rz", 45)
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


def api_scan(params: dict) -> dict:
    """蛇形扫描"""
    ms = _get_ms()
    x_range = params.get("x_range", DEFAULT_SCAN_X_RANGE)
    y_range = params.get("y_range", DEFAULT_SCAN_Y_RANGE)
    steps = int(params.get("steps", DEFAULT_SCAN_STEPS))
    z = float(params.get("z", 200))

    xs = np.linspace(x_range[0], x_range[1], steps)
    ys = np.linspace(y_range[0], y_range[1], steps)
    images = []

    for i, y in enumerate(ys):
        row_xs = xs if i % 2 == 0 else reversed(xs)
        for x in row_xs:
            ms.move_to(x=x, y=y, z=z, rx=-180, ry=0, rz=45, speed=20, wait=True)
            time.sleep(0.5)
            frame = ms.capture()
            if frame is not None:
                name = f"scan_{int(x)}_{int(y)}.jpg"
                path = RUN_DIR / name
                cv2.imwrite(str(path), frame)
                url = f"/images/{RUN_DIR.name}/{name}"
                images.append({"filename": name, "url": url})

    return {"status": "ok", "action": "scan", "images": images, "count": len(images)}


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
    from vision import GlobalCamera

    cam = GlobalCamera()
    if not cam.open():
        return {"status": "error", "action": "global_capture", "reason": "全局摄像头打开失败"}

    try:
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
    finally:
        cam.close()


def api_global_detect(params: dict) -> dict:
    """
    全局摄像头检测颜色并返回世界坐标。
    需要全局摄像头标定文件 global_camera_calib.json。
    """
    from vision import GlobalCamera, detect_color_global, pixel_to_world

    color = params.get("color")
    if not color:
        return {"status": "error", "action": "global_detect", "reason": "缺少 color 参数"}

    cam = GlobalCamera()
    if not cam.open():
        return {"status": "error", "action": "global_detect", "reason": "全局摄像头打开失败"}

    try:
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
    finally:
        cam.close()


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
    "scan": api_scan,
    "status": api_status,
}


# ---------------------------------------------------------------------------
# Flask HTTP 服务
# ---------------------------------------------------------------------------

def create_app():
    """创建 Flask 应用"""
    try:
        from flask import Flask, request, jsonify, send_from_directory
    except ImportError:
        print("[ERROR] Flask 未安装，请先运行: pip install flask")
        sys.exit(1)

    app = Flask(__name__)

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
