#!/usr/bin/env python3
"""
robot_agent.py
==============
LLM 驱动的机械臂智能体框架。

核心思想:
  把机械臂能力封装为"工具/技能"，LLM 通过 function calling 灵活组合，
  而不是执行写死的脚本。

架构:
  ┌─────────────┐
  │  User Query │  "请把桌上的红色积木放到蓝色区域"
  └──────┬──────┘
         │
  ┌──────▼──────────────────┐
  │  LLM (Planning)         │  理解意图，拆解步骤，选择工具
  │  System Prompt + Tools  │
  └──────┬──────────────────┘
         │ function calling
  ┌──────▼──────────────────┐
  │  Skill Registry         │  注册所有可调用技能
  │  - move_to(x,y,z)       │
  │  - capture()            │
  │  - detect_color("red")  │
  │  - gripper_open/close() │
  └──────┬──────────────────┘
         │
  ┌──────▼──────────────────┐
  │  Observation Loop       │  执行后反馈结果给 LLM
  │  (照片 + 坐标 + 状态)    │
  └─────────────────────────┘

用法:
    python3 robot_agent.py
    # 然后输入自然语言指令，如:
    # > 扫描桌面，找到红色物品并放到 R 区
    # > 去 (170, 0) 拍张照片看看有什么
    # > 先回零位，然后张开夹爪
"""

import json
import base64
import logging
import inspect
from dataclasses import dataclass, asdict
from typing import Callable, Dict, List, Optional, Any
from datetime import datetime

import cv2
import numpy as np

from motion_service import MotionService

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# 1. 工具/技能定义 (Tool Definition)
# ---------------------------------------------------------------------------
@dataclass
class Tool:
    """LLM 可调用的工具"""
    name: str
    description: str          # 给 LLM 看的自然语言描述
    parameters: dict          # JSON Schema 参数定义
    func: Callable            # 实际执行的 Python 函数


class SkillRegistry:
    """
    技能注册表: 所有 LLM 可调用的原子技能都在这里注册。
    类比: 就像给 LLM 提供了一套 API 文档。
    """

    def __init__(self, motion_service: MotionService):
        self.ms = motion_service
        self.tools: Dict[str, Tool] = {}
        self._register_defaults()

    def register(self, tool: Tool):
        self.tools[tool.name] = tool

    def _register_defaults(self):
        """注册默认技能集"""

        # ---- 运动类 ----
        self.register(Tool(
            name="go_home",
            description="让机械臂回到安全零位 (init position)。每次任务开始前应调用一次。",
            parameters={"type": "object", "properties": {}},
            func=lambda: self.ms.go_init(speed=10)
        ))

        self.register(Tool(
            name="move_to",
            description="移动到指定的笛卡尔坐标 (x,y,z,rx,ry,rz)。单位 mm 和度。",
            parameters={
                "type": "object",
                "properties": {
                    "x": {"type": "number", "description": "X 坐标 (mm)"},
                    "y": {"type": "number", "description": "Y 坐标 (mm)"},
                    "z": {"type": "number", "description": "Z 坐标 (mm), 默认200"},
                    "rx": {"type": "number", "description": "RX 角度, 默认-180"},
                    "ry": {"type": "number", "description": "RY 角度, 默认0"},
                    "rz": {"type": "number", "description": "RZ 角度, 默认45"},
                    "speed": {"type": "number", "description": "速度, 默认15"},
                },
                "required": ["x", "y"]
            },
            func=self._move_to_wrapper
        ))

        # ---- 视觉类 ----
        self.register(Tool(
            name="capture",
            description="用末端相机拍一张照片，返回当前视野内容。",
            parameters={"type": "object", "properties": {}},
            func=self._capture_wrapper
        ))

        self.register(Tool(
            name="scan_area",
            description="对指定区域执行蛇形扫描，拍多张照片。用于发现物品。",
            parameters={
                "type": "object",
                "properties": {
                    "x_range": {"type": "array", "description": "X范围 [min, max]", "default": [140, 200]},
                    "y_range": {"type": "array", "description": "Y范围 [min, max]", "default": [-80, 80]},
                    "steps": {"type": "number", "description": "每轴步数", "default": 3},
                }
            },
            func=self._scan_area_wrapper
        ))

        # ---- 夹爪类 ----
        self.register(Tool(
            name="gripper_open",
            description="张开夹爪",
            parameters={"type": "object", "properties": {}},
            func=self.ms.gripper_open
        ))

        self.register(Tool(
            name="gripper_close",
            description="闭合夹爪",
            parameters={"type": "object", "properties": {}},
            func=self.ms.gripper_close
        ))

        # ---- 检测类 ----
        self.register(Tool(
            name="detect_color",
            description="在当前末端相机画面中检测指定颜色物品，返回像素中心坐标。",
            parameters={
                "type": "object",
                "properties": {
                    "color": {"type": "string", "enum": ["red", "blue", "green", "yellow"],
                              "description": "要检测的颜色"},
                },
                "required": ["color"]
            },
            func=self._detect_color_wrapper
        ))

        # ---- 状态类 ----
        self.register(Tool(
            name="get_position",
            description="获取机械臂当前笛卡尔坐标",
            parameters={"type": "object", "properties": {}},
            func=lambda: {"coords": self.ms.robot.get_coords()}
        ))

    # ---- 包装函数 ----
    def _move_to_wrapper(self, x, y, z=200, rx=-180, ry=0, rz=45, speed=15):
        return self.ms.move_to(x=x, y=y, z=z, rx=rx, ry=ry, rz=rz, speed=speed, wait=True)

    def _capture_wrapper(self):
        frame = self.ms.capture()
        if frame is not None:
            # 保存并返回路径
            path = f"agent_capture_{datetime.now().strftime('%H%M%S')}.jpg"
            cv2.imwrite(path, frame)
            return {"image_path": path, "shape": frame.shape}
        return {"error": "capture failed"}

    def _scan_area_wrapper(self, x_range=(140, 200), y_range=(-80, 80), steps=3):
        """蛇形扫描并返回所有照片路径"""
        images = []
        xs = np.linspace(x_range[0], x_range[1], steps)
        ys = np.linspace(y_range[0], y_range[1], steps)

        for i, y in enumerate(ys):
            row_xs = xs if i % 2 == 0 else reversed(xs)
            for x in row_xs:
                self.ms.move_to(x=x, y=y, z=200, rx=-180, ry=0, rz=45, speed=20, wait=True)
                import time; time.sleep(0.5)
                frame = self.ms.capture()
                if frame is not None:
                    path = f"scan_{x:.0f}_{y:.0f}.jpg"
                    cv2.imwrite(path, frame)
                    images.append(path)
        return {"images": images, "count": len(images)}

    def _detect_color_wrapper(self, color):
        frame = self.ms.capture()
        if frame is None:
            return {"found": False, "reason": "capture failed"}

        # 简化的颜色检测
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        if color == "red":
            mask = cv2.inRange(hsv, np.array([0, 80, 80]), np.array([12, 255, 255])) | \
                   cv2.inRange(hsv, np.array([160, 80, 80]), np.array([180, 255, 255]))
        elif color == "blue":
            mask = cv2.inRange(hsv, np.array([100, 150, 0]), np.array([140, 255, 255]))
        elif color == "green":
            mask = cv2.inRange(hsv, np.array([40, 100, 100]), np.array([80, 255, 255]))
        elif color == "yellow":
            mask = cv2.inRange(hsv, np.array([20, 100, 100]), np.array([35, 255, 255]))
        else:
            return {"found": False, "reason": "unknown color"}

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return {"found": False, "reason": "no contours"}

        best = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(best)
        if area < 300:
            return {"found": False, "reason": "too small", "area": area}

        M = cv2.moments(best)
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        h, w = frame.shape[:2]
        return {"found": True, "pixel": (cx, cy), "area": area,
                "offset_from_center": (cx - w // 2, cy - h // 2)}


# ---------------------------------------------------------------------------
# 2. Agent 核心: LLM 规划 + 工具调用
# ---------------------------------------------------------------------------
class RobotAgent:
    """
    LLM 驱动的机械臂智能体。
    支持两种模式:
      A. Manual: 用户直接输入工具调用指令 (类似 REPL)
      B. Auto:    接入 LLM API，由模型自主规划 (需要 OpenAI/Claude API)
    """

    SYSTEM_PROMPT = """你是一个机器人操作助手，控制一台 myCobot 280 机械臂。

可用技能:
{tools_description}

操作规则:
1. 每次任务开始前必须先调用 go_home() 回到零位
2. 抓取任何物品前必须先 gripper_open() 张开夹爪
3. 抓取后需要 +30mm X 偏移补偿 (grasp_x = camera_x + 30)
4. 放置高度通常是 Z=105
5. 如果检测不到物品，先调用 scan_area() 扫描

当前支持的自然语言指令示例:
- "回零位"
- "去 (170, 0) 拍照"
- "扫描桌面"
- "检测红色物品"
- "张开夹爪"
- "移动到 X=150 Y=30 Z=200"
"""

    def __init__(self, motion_service: MotionService):
        self.ms = motion_service
        self.registry = SkillRegistry(motion_service)
        self.history: List[dict] = []  # 执行历史

    def get_tools_description(self) -> str:
        """生成给 LLM 的工具说明文本"""
        lines = []
        for name, tool in self.registry.tools.items():
            lines.append(f"- {name}: {tool.description}")
            lines.append(f"  参数: {json.dumps(tool.parameters, ensure_ascii=False)}")
        return "\n".join(lines)

    def execute_tool(self, name: str, params: dict) -> dict:
        """执行指定工具，返回结果"""
        if name not in self.registry.tools:
            return {"error": f"未知工具: {name}"}

        tool = self.registry.tools[name]
        try:
            # 过滤参数，只传 func 需要的
            sig = inspect.signature(tool.func)
            valid_params = {k: v for k, v in params.items() if k in sig.parameters}
            result = tool.func(**valid_params)
            return {"tool": name, "params": params, "result": result, "status": "ok"}
        except Exception as e:
            return {"tool": name, "params": params, "error": str(e), "status": "error"}

    def parse_user_command(self, text: str) -> Optional[tuple]:
        """
        简单指令解析器: 把自然语言映射到工具调用。
        未来可替换为真正的 LLM function calling。
        """
        text = text.lower().strip()

        # 模式匹配 (简单版)
        if text in ("回零位", "归零", "home", "init", "go home"):
            return "go_home", {}

        if text in ("张开夹爪", "open gripper", "gripper open"):
            return "gripper_open", {}

        if text in ("闭合夹爪", "关闭夹爪", "close gripper", "gripper close"):
            return "gripper_close", {}

        if text in ("拍照", "capture", "拍一张"):
            return "capture", {}

        if text in ("扫描", "scan", "扫描桌面"):
            return "scan_area", {}

        if "获取位置" in text or "当前坐标" in text:
            return "get_position", {}

        # 移动指令: "移动到 X=150 Y=30" 或 "去 (150, 30)"
        import re
        move_match = re.search(r'[去移].*?[\(\[]?\s*(-?\d+\.?\d*)\s*[，,\s]\s*(-?\d+\.?\d*)\s*[\)\]]?', text)
        if move_match:
            x, y = float(move_match.group(1)), float(move_match.group(2))
            z_match = re.search(r'Z[=\s]+(-?\d+\.?\d*)', text)
            z = float(z_match.group(1)) if z_match else 200
            return "move_to", {"x": x, "y": y, "z": z}

        # 检测颜色: "检测红色" 或 "找红色"
        color_match = re.search(r'(检测|找|查找)\s*(红色|蓝色|绿色|黄色)', text)
        if color_match:
            color_map = {"红色": "red", "蓝色": "blue", "绿色": "green", "黄色": "yellow"}
            color = color_map.get(color_match.group(2), "red")
            return "detect_color", {"color": color}

        return None

    def repl(self):
        """交互式命令行 (Manual 模式)"""
        print("\n" + "=" * 70)
        print("Robot Agent - 交互式控制")
        print("=" * 70)
        print("输入自然语言指令，或 'help' 查看可用命令，'quit' 退出")
        print("=" * 70)

        while True:
            try:
                user_input = input("\n🤖 > ").strip()
            except (EOFError, KeyboardInterrupt):
                break

            if not user_input:
                continue

            if user_input in ("quit", "exit", "q"):
                break

            if user_input == "help":
                print("\n可用指令:")
                for name, tool in self.registry.tools.items():
                    print(f"  {name}: {tool.description}")
                print("\n自然语言示例:")
                print('  "回零位"')
                print('  "去 (170, 0)"')
                print('  "检测红色"')
                print('  "扫描桌面"')
                print('  "拍照"')
                continue

            # 解析指令
            parsed = self.parse_user_command(user_input)
            if parsed is None:
                print("❓ 未能理解指令。输入 'help' 查看示例。")
                continue

            tool_name, params = parsed
            print(f"🔧 执行: {tool_name}({params})")

            # 执行
            result = self.execute_tool(tool_name, params)
            self.history.append({"command": user_input, "result": result})

            # 显示结果
            if result.get("status") == "ok":
                print(f"✅ 成功: {result['result']}")
            else:
                print(f"❌ 失败: {result.get('error', 'unknown error')}")

        print("\n👋 再见!")


# ---------------------------------------------------------------------------
# 3. 扩展: 全局视觉技能 (复用现有标定)
# ---------------------------------------------------------------------------
def add_global_vision_skill(registry: SkillRegistry):
    """添加全局摄像头相关技能"""

    import json
    try:
        with open("global_camera_calib.json", "r") as f:
            calib = json.load(f)
        affine_matrix = np.array(calib["affine_matrix"], dtype=np.float32)
    except Exception:
        affine_matrix = None

    def global_detect_color(color: str):
        """用全局摄像头检测颜色并返回世界坐标"""
        cap = cv2.VideoCapture(1)
        for _ in range(5):
            cap.read()
        ret, frame = cap.read()
        cap.release()
        if not ret:
            return {"error": "global camera capture failed"}

        # 限制工作区域检测
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        if color == "red":
            mask = cv2.inRange(hsv, np.array([0, 100, 80]), np.array([12, 255, 255])) | \
                   cv2.inRange(hsv, np.array([160, 100, 80]), np.array([180, 255, 255]))
        # ... 其他颜色类似
        else:
            return {"error": f"color {color} not supported in global mode"}

        # ROI 限制
        roi_mask = np.zeros_like(mask)
        roi_mask[300:800, 500:1200] = mask[300:800, 500:1200]

        contours, _ = cv2.findContours(roi_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return {"found": False, "reason": "no target in work area"}

        best = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(best)
        if area < 500:
            return {"found": False, "reason": "too small", "area": area}

        M = cv2.moments(best)
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])

        # 转换为世界坐标
        if affine_matrix is not None:
            pt = affine_matrix @ np.array([[cx], [cy], [1.0]])
            wx, wy = float(pt[0, 0]), float(pt[1, 0])
            return {"found": True, "pixel": (cx, cy), "world": (round(wx, 1), round(wy, 1)), "area": area}
        else:
            return {"found": True, "pixel": (cx, cy), "area": area, "warning": "no calibration"}

    registry.register(Tool(
        name="global_detect_color",
        description="用全局摄像头(BRIO)检测颜色物品，直接返回世界坐标。不需要先移动机械臂。",
        parameters={
            "type": "object",
            "properties": {
                "color": {"type": "string", "enum": ["red", "blue", "green", "yellow"]}
            },
            "required": ["color"]
        },
        func=global_detect_color
    ))


# ---------------------------------------------------------------------------
# 主函数
# ---------------------------------------------------------------------------
def main():
    print("=" * 70)
    print("Robot Agent - LLM 驱动的机械臂智能体")
    print("=" * 70)

    ms = MotionService()
    print("\n[INFO] 连接机械臂...")
    if not ms.connect(warmup=True):
        print("[ERROR] 连接失败")
        return

    agent = RobotAgent(ms)

    # 注册全局视觉技能
    add_global_vision_skill(agent.registry)

    print(f"[OK] 已注册 {len(agent.registry.tools)} 个技能")

    # 进入交互模式
    agent.repl()

    ms.close()


if __name__ == "__main__":
    main()
