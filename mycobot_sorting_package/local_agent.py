#!/usr/bin/env python3
"""
local_agent.py
==============
本地执行代理（Local Execution Agent）— 硬件访问唯一入口。

职责（精简后）：
  - 连接机械臂和相机（唯一有硬件访问权限的进程）
  - 接收 JSON 指令，执行原子操作
  - 返回结构化结果（JSON）：坐标、照片路径、检测状态

三种模式:
  1. 交互模式: python3 local_agent.py
  2. 守护进程模式: python3 local_agent.py --daemon
     AI 通过 ai_agent_bridge.py 与之通信
  3. 单次执行: python3 local_agent.py --once

AI 通信流程:
  1. AI -> ai_agent_bridge -> command.json
  2. Local Agent 检测并执行 -> result.json
  3. ai_agent_bridge 读取结果 -> 返回给 AI

注意:
  - 本文件只负责硬件层，视觉检测逻辑已迁移到 vision.py
  - 所有参数统一从 config.py 读取
  - 禁止直接实例化本模块的类来绕过守护进程
"""

from __future__ import annotations

import json
import os
import sys
import time
import argparse
from datetime import datetime
from typing import Any, Dict, Optional

import cv2
import numpy as np

# 统一配置和视觉模块
from config import (
    DEFAULT_COLOR_RANGES,
    DEFAULT_SCAN_STEPS,
    DEFAULT_SCAN_X_RANGE,
    DEFAULT_SCAN_Y_RANGE,
    ensure_log_dir,
)
from vision import detect_color_in_frame, save_debug_image
from motion_service import MotionService

# ---------- 日志目录 ----------
RUN_DIR = ensure_log_dir()

# AI 通信文件（工作目录下）
CMD_FILE = "command.json"
RESULT_FILE = "result.json"
AGENT_PID_FILE = ".local_agent.pid"

# 机械臂实例（单例，守护进程内唯一）
_ms: Optional[MotionService] = None


def get_ms() -> MotionService:
    global _ms
    if _ms is None:
        _ms = MotionService()
        if not _ms.connect(warmup=True):
            raise RuntimeError("机械臂连接失败")
    return _ms


def disconnect():
    global _ms
    if _ms:
        _ms.close()
        _ms = None


# ---------- 技能实现（硬件层原子操作） ----------

def cmd_go_home(_params: dict) -> dict:
    ms = get_ms()
    ms.gripper_open()
    time.sleep(0.3)
    result = ms.go_init(speed=10)
    time.sleep(1.0)
    return {"status": "ok", "action": "go_home", "result": result}


def cmd_move_to(params: dict) -> dict:
    ms = get_ms()
    x = params["x"]
    y = params["y"]
    z = params.get("z", 200)
    rx = params.get("rx", -180)
    ry = params.get("ry", 0)
    rz = params.get("rz", 45)
    speed = params.get("speed", 20)
    wait = params.get("wait", True)

    result = ms.move_to(x=x, y=y, z=z, rx=rx, ry=ry, rz=rz, speed=speed, wait=wait)
    if wait:
        time.sleep(0.5)

    return {
        "status": "ok",
        "action": "move_to",
        "target": [x, y, z, rx, ry, rz],
        "result": result,
    }


def cmd_capture(_params: dict) -> dict:
    ms = get_ms()
    frame = ms.capture()
    if frame is None:
        return {"status": "error", "action": "capture", "reason": "capture failed"}

    timestamp = datetime.now().strftime("%H%M%S_%f")[:-3]
    filename = f"cap_{timestamp}.jpg"
    filepath = os.path.join(RUN_DIR, filename)
    cv2.imwrite(filepath, frame)

    return {
        "status": "ok",
        "action": "capture",
        "image_path": filepath,
        "image_name": filename,
        "shape": frame.shape,
    }


def cmd_detect(params: dict) -> dict:
    """
    颜色检测。视觉逻辑委托给 vision.detect_color_in_frame，
    本函数只负责：拍照 -> 调用检测 -> 保存调试图 -> 组装返回 JSON。
    """
    ms = get_ms()
    color = params["color"]
    frame = ms.capture()
    if frame is None:
        return {"status": "error", "action": "detect", "reason": "capture failed"}

    # 统一视觉检测
    result = detect_color_in_frame(
        frame=frame,
        color=color,
        color_ranges=DEFAULT_COLOR_RANGES,
        return_debug=True,
    )

    # 保存调试图
    if result.debug_image is not None:
        debug_name = f"detect_{color}_{datetime.now().strftime('%H%M%S')}.jpg"
        debug_path = os.path.join(RUN_DIR, debug_name)
        cv2.imwrite(debug_path, result.debug_image)
    else:
        debug_path = None

    if not result.found:
        return {
            "status": "ok",
            "action": "detect",
            "found": False,
            "color": color,
            "reason": result.reason,
            "debug_image": debug_path,
        }

    return {
        "status": "ok",
        "action": "detect",
        "found": True,
        "color": color,
        "pixel": result.pixel_center,
        "area": result.area,
        "offset_from_center": result.offset_from_center,
        "debug_image": debug_path,
    }


def cmd_gripper_open(_params: dict) -> dict:
    get_ms().gripper_open()
    time.sleep(0.3)
    return {"status": "ok", "action": "gripper_open"}


def cmd_gripper_close(_params: dict) -> dict:
    get_ms().gripper_close()
    time.sleep(0.5)
    return {"status": "ok", "action": "gripper_close"}


def cmd_get_position(_params: dict) -> dict:
    ms = get_ms()
    coords = ms.robot.get_coords()
    return {"status": "ok", "action": "get_position", "coords": coords}


def cmd_scan(params: dict) -> dict:
    """
    蛇形扫描。参数默认值从 config.py 读取。
    """
    ms = get_ms()
    x_range = params.get("x_range", DEFAULT_SCAN_X_RANGE)
    y_range = params.get("y_range", DEFAULT_SCAN_Y_RANGE)
    steps = params.get("steps", DEFAULT_SCAN_STEPS)
    z = params.get("z", 200)

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
                path = os.path.join(RUN_DIR, name)
                cv2.imwrite(path, frame)
                images.append(name)

    return {"status": "ok", "action": "scan", "images": images, "count": len(images)}


# ---------- 指令路由 ----------
COMMANDS = {
    "go_home": cmd_go_home,
    "move_to": cmd_move_to,
    "capture": cmd_capture,
    "detect": cmd_detect,
    "gripper_open": cmd_gripper_open,
    "gripper_close": cmd_gripper_close,
    "get_position": cmd_get_position,
    "scan": cmd_scan,
}


def execute_json(command_json: str) -> dict:
    """执行 JSON 指令，返回 dict 结果"""
    try:
        data = json.loads(command_json)
    except json.JSONDecodeError as e:
        return {"status": "error", "reason": f"invalid json: {e}"}

    cmd = data.get("cmd")
    if cmd not in COMMANDS:
        return {"status": "error", "reason": f"unknown cmd: {cmd}", "available": list(COMMANDS.keys())}

    params = {k: v for k, v in data.items() if k != "cmd"}

    try:
        result = COMMANDS[cmd](params)
    except Exception as e:
        result = {"status": "error", "action": cmd, "reason": str(e)}

    return result


# ---------- 守护进程模式 ----------

def daemon_mode():
    import signal

    # 写入 PID 文件
    with open(AGENT_PID_FILE, "w") as f:
        f.write(str(os.getpid()))

    def cleanup(signum=None, frame=None):
        print("\n[DAEMON] 收到退出信号，清理中...")
        disconnect()
        for f in [CMD_FILE, RESULT_FILE, AGENT_PID_FILE]:
            if os.path.exists(f):
                os.remove(f)
        print("[DAEMON] 已退出")
        sys.exit(0)

    signal.signal(signal.SIGINT, cleanup)
    signal.signal(signal.SIGTERM, cleanup)

    print("=" * 60)
    print("Local Agent - 守护进程模式")
    print("=" * 60)
    print(f"日志目录: {RUN_DIR}")
    print(f"监听文件: ./{CMD_FILE}")
    print(f"结果文件: ./{RESULT_FILE}")
    print()
    print("AI 可以通过 ai_agent_bridge.py 发送指令")
    print()
    print("按 Ctrl+C 退出")
    print("=" * 60)

    # 预连接机械臂
    print("[DAEMON] 连接机械臂...")
    try:
        get_ms()
        print("[DAEMON] 机械臂已连接，等待指令...")
    except Exception as e:
        print(f"[DAEMON] [ERROR] 连接失败: {e}")
        cleanup()

    while True:
        if os.path.exists(CMD_FILE):
            try:
                with open(CMD_FILE, "r") as f:
                    command_json = f.read().strip()

                if command_json:
                    print(f"\n[DAEMON] 收到指令: {command_json}")
                    result = execute_json(command_json)
                    print(f"[DAEMON] 执行结果: {json.dumps(result, ensure_ascii=False)}")

                    with open(RESULT_FILE, "w") as f:
                        json.dump(result, f, ensure_ascii=False, indent=2)

            except Exception as e:
                error_result = {"status": "error", "reason": f"daemon execution error: {e}"}
                with open(RESULT_FILE, "w") as f:
                    json.dump(error_result, f, ensure_ascii=False, indent=2)

            finally:
                if os.path.exists(CMD_FILE):
                    os.remove(CMD_FILE)

        time.sleep(0.05)  # 50ms 轮询


# ---------- 交互模式 ----------

def interactive_mode():
    print("=" * 60)
    print("Local Agent - 交互模式")
    print("=" * 60)
    print(f"日志目录: {RUN_DIR}")
    print()
    print("输入 JSON 指令，或 'help' 查看示例，'quit' 退出")
    print()

    while True:
        try:
            user_input = input("LOCAL> ").strip()
        except (EOFError, KeyboardInterrupt):
            break

        if not user_input:
            continue
        if user_input in ("quit", "exit", "q"):
            break

        if user_input == "help":
            print("""
可用指令 (JSON):
  {"cmd": "go_home"}
  {"cmd": "move_to", "x": 170, "y": 0, "z": 200}
  {"cmd": "capture"}
  {"cmd": "detect", "color": "red"}
  {"cmd": "gripper_open"}
  {"cmd": "gripper_close"}
  {"cmd": "get_position"}
  {"cmd": "scan", "x_range": [140,200], "y_range": [-80,80], "steps": 3}

简写: home, capture, open, close, pos
            """)
            continue

        shortcuts = {
            "home": '{"cmd": "go_home"}',
            "capture": '{"cmd": "capture"}',
            "open": '{"cmd": "gripper_open"}',
            "close": '{"cmd": "gripper_close"}',
            "pos": '{"cmd": "get_position"}',
        }
        if user_input in shortcuts:
            user_input = shortcuts[user_input]

        result = execute_json(user_input)
        print(json.dumps(result, ensure_ascii=False, indent=2))
        print()

    disconnect()
    print("\nLocal Agent 已退出")


# ---------- 单次执行 ----------

def execute_once(command_json: str) -> str:
    """单次执行，连接→执行→断开，返回 JSON 字符串"""
    try:
        result = execute_json(command_json)
        return json.dumps(result, ensure_ascii=False, indent=2)
    finally:
        disconnect()


# ---------- 主函数 ----------
def main():
    parser = argparse.ArgumentParser(description="Local Agent - 本地机械臂执行代理")
    parser.add_argument("--daemon", action="store_true", help="守护进程模式（AI 直接通信）")
    parser.add_argument("--once", action="store_true", help="单次执行（从 stdin 读取 JSON）")
    args = parser.parse_args()

    if args.daemon:
        daemon_mode()
    elif args.once:
        command_json = sys.stdin.read().strip()
        if command_json:
            print(execute_once(command_json))
        else:
            print(json.dumps({"status": "error", "reason": "no input"}))
    else:
        interactive_mode()


if __name__ == "__main__":
    main()
