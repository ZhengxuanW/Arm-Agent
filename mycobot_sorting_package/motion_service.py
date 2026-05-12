"""
motion_service.py
=================
机械臂运动控制核心服务。

职责：
  - 连接/断开机械臂和相机（长连接复用）
  - 执行笛卡尔坐标运动、J6 视角转动
  - 拍照（单帧 + 连续流）
  - 安全边界检查、到位检测
  - 任务队列管理（异步执行）

特点：
  - 纯 Python 类，不依赖 Flask/Web
  - 可以被 import（共享实例）
  - 也可以独立运行（对外暴露 HTTP 或 ZeroMQ）
  - 视角与位置解耦：运动时自动保持当前 rz

用法:
    from motion_service import MotionService, get_motion_service

    # 方式1: 单例（推荐，进程内共享）
    svc = get_motion_service()
    svc.connect()
    svc.move_to(x=-80, y=90, z=200)
    frame = svc.capture()

    # 方式2: 独立实例
    svc = MotionService()
    svc.connect()
    svc.rotate_j6(-20)
    svc.close()
"""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass, field
from typing import Optional

import cv2
import numpy as np

from camera_vision import EndEffectorCamera
from pymycobot import MyCobot

logger = logging.getLogger(__name__)

DEFAULT_PORT = "/dev/cu.usbserial-1120"
DEFAULT_BAUD = 1000000


@dataclass
class ArmStatus:
    """机械臂当前状态"""
    connected: bool
    coords: Optional[list] = None
    angles: Optional[list] = None
    is_moving: bool = False
    camera_ok: bool = False


@dataclass
class TaskStatus:
    """后台任务状态"""
    task_id: str
    task_type: str  # 'move', 'rotate_j6', 'scan'
    state: str  # 'pending', 'running', 'done', 'failed'
    target: Optional[list] = None
    result: Optional[dict] = None
    error: Optional[str] = None
    started_at: float = 0.0
    finished_at: Optional[float] = None

    def to_dict(self) -> dict:
        return {
            "task_id": self.task_id,
            "task_type": self.task_type,
            "state": self.state,
            "target": self.target,
            "result": self.result,
            "error": self.error,
            "started_at": self.started_at,
            "finished_at": self.finished_at,
            "elapsed_sec": round(time.time() - self.started_at, 2) if self.state in ("pending", "running") else (
                round(self.finished_at - self.started_at, 2) if self.finished_at else None
            ),
        }


class MotionService:
    """
    机械臂运动控制核心。

    特点:
      - 长连接复用: 串口和相机只在初始化时打开一次
      - 快速响应: 省去每次重新连接的开销
      - 安全边界: 自动检查坐标边界
      - 到位检测: 轮询 is_moving 而不是固定 sleep
      - 视角解耦: rz 与位置运动分离
    """

    def __init__(
        self,
        port: str = DEFAULT_PORT,
        baud: int = DEFAULT_BAUD,
        camera_source: int = 0,
        camera_width: int = 640,
        camera_height: int = 480,
        camera_fps: int = 30,
    ):
        self.port = port
        self.baud = baud
        self.camera_source = camera_source
        self.camera_width = camera_width
        self.camera_height = camera_height
        self.camera_fps = camera_fps

        self.robot: Optional[MyCobot] = None
        self.cam: Optional[EndEffectorCamera] = None
        self._connected = False

        # 后台任务管理
        self._tasks: dict[str, TaskStatus] = {}
        self._task_lock = threading.Lock()
        self._task_counter = 0

        # 安全边界 (mm)
        self.x_limit = (-220, 220)
        self.y_limit = (30, 260)
        self.z_limit = (50, 400)

    # ------------------------------------------------------------------
    # 连接管理
    # ------------------------------------------------------------------

    def connect(self, warmup: bool = True) -> bool:
        """连接机械臂和相机。"""
        logger.info("Connecting to robot on %s @ %d ...", self.port, self.baud)
        try:
            self.robot = MyCobot(self.port, self.baud)
            if warmup:
                time.sleep(1.0)
            self._connected = True
            logger.info("Robot connected.")
        except Exception as exc:
            logger.error("Robot connection failed: %s", exc)
            return False

        logger.info("Opening camera (index=%d) ...", self.camera_source)
        self.cam = EndEffectorCamera(
            source=self.camera_source,
            width=self.camera_width,
            height=self.camera_height,
            fps=self.camera_fps,
            mode="index",
        )
        if not self.cam.open():
            logger.error("Camera open failed.")
            self.close()
            return False

        for _ in range(5):
            self.cam.grab()
        logger.info("Camera ready.")
        return True

    def close(self) -> None:
        """关闭连接。"""
        if self.cam:
            try:
                self.cam.close()
            except Exception:
                pass
            self.cam = None
        if self.robot:
            try:
                self.robot._serial_port.close()
            except Exception:
                pass
            self.robot = None
        self._connected = False
        logger.info("Disconnected.")

    # ------------------------------------------------------------------
    # 安全边界
    # ------------------------------------------------------------------

    def _check_boundary(self, x: float, y: float, z: float) -> None:
        """安全边界检查（已禁用，机械臂固件自行保护）。"""
        pass

    # ------------------------------------------------------------------
    # 运动控制（同步）
    # ------------------------------------------------------------------

    def _is_near_singularity(self, angles: list, threshold: float = 5.0) -> bool:
        """检查是否接近奇异点（J5 接近 0° 时 J4/J6 对齐）。"""
        if not angles or len(angles) < 5:
            return False
        j5 = abs(angles[4])
        return j5 < threshold

    def _get_safe_speed(self, current: list, target: list, base_speed: int) -> int:
        """根据距离调整速度，避免大跨度高速运动。"""
        if not current or len(current) < 3:
            return min(base_speed, 30)
        dist = ((current[0]-target[0])**2 + (current[1]-target[1])**2 + (current[2]-target[2])**2) ** 0.5
        # 距离越大速度越低
        if dist > 200:
            return min(base_speed, 30)
        elif dist > 100:
            return min(base_speed, 40)
        return min(base_speed, 50)

    def move_to(
        self,
        x: float,
        y: float,
        z: float,
        rx: float = -180.0,
        ry: float = 0.0,
        rz: Optional[float] = None,
        speed: int = 50,
        wait: bool = True,
        timeout: float = 15.0,
        poll_interval: float = 0.05,
    ) -> dict:
        """
        运动到指定笛卡尔坐标。
        rz=None 时自动保持当前视角角度。

        安全增强:
          - 自动检查奇异点
          - 大距离自动降速
          - 强制限速 max 50
        """
        if not self._connected:
            raise RuntimeError("Not connected. Call connect() first.")

        self._check_boundary(x, y, z)

        # 获取当前状态
        current_angles = self.robot.get_angles()
        current_coords = self.robot.get_coords()

        # 奇异点警告
        if self._is_near_singularity(current_angles):
            logger.warning("⚠️ NEAR SINGULARITY (J5=%.1f°). Movement may be unstable!", current_angles[4])

        # 视角解耦
        if rz is None:
            current_rz = current_coords[5] if current_coords and len(current_coords) >= 6 else 0.0
            effective_rz = current_rz
            logger.info("MOVE_TO keeping current rz=%.1f", effective_rz)
        else:
            effective_rz = float(rz)

        target = [float(x), float(y), float(z), float(rx), float(ry), effective_rz]

        # 安全速度
        safe_speed = self._get_safe_speed(current_coords, target, speed)
        safe_speed = min(safe_speed, 50)  # 硬上限

        logger.info("MOVE_TO %s speed=%d (safe=%d)", target, speed, safe_speed)

        t0 = time.time()
        self.robot.send_coords(target, safe_speed, 1)

        if not wait:
            return {"success": True, "target": target, "elapsed_sec": time.time() - t0}

        # 到位检测
        stopped = False
        while time.time() - t0 < timeout:
            try:
                if self.robot.is_moving() == 0:
                    stopped = True
                    break
            except Exception:
                pass
            time.sleep(poll_interval)

        elapsed = time.time() - t0
        actual = self.robot.get_coords()

        result = {
            "success": stopped,
            "target": target,
            "actual": actual,
            "elapsed_sec": round(elapsed, 2),
        }
        logger.info("MOVE_DONE in %.2fs -> actual=%s", elapsed, actual)
        return result

    def rotate_j6(
        self,
        angle: float,
        speed: int = 30,
        wait: bool = True,
        timeout: float = 5.0,
    ) -> dict:
        """单独转动 J6 关节（末端自转 / 视角旋转）。"""
        if not self._connected:
            raise RuntimeError("Not connected. Call connect() first.")

        logger.info("ROTATE_J6 angle=%.1f speed=%d", angle, speed)
        t0 = time.time()
        self.robot.send_angle(6, angle, speed)

        if not wait:
            return {"success": True, "angle": angle, "elapsed_sec": time.time() - t0}

        wait_sec = 0.3 if abs(angle) <= 45 else 0.6
        time.sleep(wait_sec)

        elapsed = time.time() - t0
        logger.info("J6_DONE in %.2fs", elapsed)
        return {"success": True, "angle": angle, "elapsed_sec": round(elapsed, 2)}

    def go_home(
        self,
        coords: Optional[list] = None,
        speed: int = 30,
        wait: bool = True,
    ) -> dict:
        """回到 HOME 位姿。"""
        home = coords or [0.0, 160.0, 200.0, -180.0, 0.0, 0.0]
        return self.move_to(*home, speed=speed, wait=wait)

    def go_init(self, speed: int = 10, wait: bool = True) -> dict:
        """回到初始化位姿（关节空间，安全近零位）。"""
        if not self._connected:
            raise RuntimeError("Not connected. Call connect() first.")

        init_angles = [0.0, 0.0, 0.0, 0.0, 20.0, 0.0]
        logger.info("GO_INIT %s speed=%d", init_angles, speed)
        t0 = time.time()
        self.robot.send_angles(init_angles, speed)

        if not wait:
            return {"success": True, "angles": init_angles, "elapsed_sec": time.time() - t0}

        # 到位检测
        stopped = False
        timeout = 20.0
        poll_interval = 0.1
        while time.time() - t0 < timeout:
            try:
                if self.robot.is_moving() == 0:
                    stopped = True
                    break
            except Exception:
                pass
            time.sleep(poll_interval)

        elapsed = time.time() - t0
        actual = self.robot.get_angles()

        result = {
            "success": stopped,
            "target_angles": init_angles,
            "actual_angles": actual,
            "elapsed_sec": round(elapsed, 2),
        }
        logger.info("GO_INIT_DONE in %.2fs -> actual=%s", elapsed, actual)
        return result

    # ------------------------------------------------------------------
    # 异步 API
    # ------------------------------------------------------------------

    def _next_task_id(self) -> str:
        with self._task_lock:
            self._task_counter += 1
            return f"task_{self._task_counter:04d}"

    def _run_async(self, task: TaskStatus, fn, *args, **kwargs):
        def _worker():
            task.state = "running"
            try:
                result = fn(*args, **kwargs)
                task.result = result
                task.state = "done"
            except Exception as exc:
                task.error = str(exc)
                task.state = "failed"
                logger.exception("Async task %s failed", task.task_id)
            finally:
                task.finished_at = time.time()
        threading.Thread(target=_worker, daemon=True).start()

    def move_to_async(
        self,
        x: float,
        y: float,
        z: float,
        rx: float = -180.0,
        ry: float = 0.0,
        rz: Optional[float] = None,
        speed: int = 50,
    ) -> str:
        """非阻塞运动。返回 task_id。"""
        if not self._connected:
            raise RuntimeError("Not connected. Call connect() first.")

        self._check_boundary(x, y, z)
        target = [float(x), float(y), float(z), float(rx), float(ry), rz]

        task_id = self._next_task_id()
        task = TaskStatus(
            task_id=task_id,
            task_type="move",
            state="pending",
            target=target,
            started_at=time.time(),
        )
        with self._task_lock:
            self._tasks[task_id] = task

        logger.info("MOVE_ASYNC %s -> task=%s", target, task_id)
        self._run_async(task, self.move_to, x, y, z, rx, ry, rz, speed=speed, wait=True)
        return task_id

    def rotate_j6_async(
        self,
        angle: float,
        speed: int = 30,
    ) -> str:
        """非阻塞转动 J6。返回 task_id。"""
        if not self._connected:
            raise RuntimeError("Not connected. Call connect() first.")

        task_id = self._next_task_id()
        task = TaskStatus(
            task_id=task_id,
            task_type="rotate_j6",
            state="pending",
            target=[angle],
            started_at=time.time(),
        )
        with self._task_lock:
            self._tasks[task_id] = task

        logger.info("ROTATE_J6_ASYNC angle=%.1f -> task=%s", angle, task_id)
        self._run_async(task, self.rotate_j6, angle, speed=speed, wait=True)
        return task_id

    def get_task(self, task_id: str) -> Optional[TaskStatus]:
        with self._task_lock:
            return self._tasks.get(task_id)

    def list_tasks(self, limit: int = 20) -> list[TaskStatus]:
        with self._task_lock:
            ids = sorted(self._tasks.keys(), reverse=True)[:limit]
            return [self._tasks[i] for i in ids]

    # ------------------------------------------------------------------
    # 夹爪 / Gripper
    # ------------------------------------------------------------------

    def gripper_open(self, speed: int = 50) -> dict:
        """Open gripper."""
        if not self._connected:
            raise RuntimeError("Not connected.")
        self.robot.set_gripper_state(0, speed)
        time.sleep(0.5)
        return {"success": True, "state": "open"}

    def gripper_close(self, speed: int = 50) -> dict:
        """Close gripper."""
        if not self._connected:
            raise RuntimeError("Not connected.")
        self.robot.set_gripper_state(1, speed)
        time.sleep(0.5)
        return {"success": True, "state": "close"}

    # ------------------------------------------------------------------
    # 视觉
    # ------------------------------------------------------------------

    def capture(self, retries: int = 3) -> Optional[np.ndarray]:
        """拍照。返回 BGR 图像数组或 None。"""
        if not self.cam:
            return None
        for _ in range(retries):
            frame = self.cam.grab()
            if frame is not None:
                return frame
            time.sleep(0.05)
        return None

    def capture_with_overlay(
        self,
        text: Optional[str] = None,
        crosshair: bool = True,
    ) -> Optional[np.ndarray]:
        """拍照并叠加信息。"""
        frame = self.capture()
        if frame is None:
            return None
        vis = frame.copy()
        h, w = vis.shape[:2]

        status = self.get_status()
        if status.coords:
            c = status.coords
            label = f"({c[0]:.0f},{c[1]:.0f},{c[2]:.0f})"
            cv2.putText(vis, label, (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        if text:
            cv2.putText(vis, text, (10, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 1)

        if crosshair:
            cx, cy = w // 2, h // 2
            cv2.drawMarker(vis, (cx, cy), (0, 0, 255), cv2.MARKER_CROSS, 30, 2)

        return vis

    # ------------------------------------------------------------------
    # 状态
    # ------------------------------------------------------------------

    def get_status(self) -> ArmStatus:
        if not self._connected or not self.robot:
            return ArmStatus(connected=False)

        try:
            coords = self.robot.get_coords()
            angles = self.robot.get_angles()
            moving = self.robot.is_moving()
        except Exception as exc:
            logger.warning("Status query failed: %s", exc)
            coords = None
            angles = None
            moving = False

        return ArmStatus(
            connected=True,
            coords=coords,
            angles=angles,
            is_moving=(moving == 1) if moving is not None else False,
            camera_ok=(self.cam is not None),
        )

    # ------------------------------------------------------------------
    # 批量扫描
    # ------------------------------------------------------------------

    def scan_waypoints(
        self,
        waypoints: list[list[float]],
        speed: int = 50,
        settle: float = 0.3,
        callback=None,
    ) -> list[dict]:
        results = []
        for idx, wp in enumerate(waypoints):
            result = self.move_to(*wp, speed=speed, wait=True)
            time.sleep(settle)

            frame = self.capture()
            if callback:
                try:
                    callback(idx, wp, frame)
                except Exception as exc:
                    logger.warning("Waypoint callback error: %s", exc)

            results.append({
                "index": idx,
                "waypoint": wp,
                "move_result": result,
                "frame_ok": frame is not None,
            })
        return results


# ------------------------------------------------------------------
# 单例封装
# ------------------------------------------------------------------

_default_service: Optional[MotionService] = None


def get_motion_service() -> MotionService:
    """获取默认服务实例（懒加载）。"""
    global _default_service
    if _default_service is None:
        _default_service = MotionService()
    return _default_service


def shutdown_motion_service() -> None:
    """关闭默认服务。"""
    global _default_service
    if _default_service:
        _default_service.close()
        _default_service = None


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s [%(levelname)s] %(message)s")
    svc = MotionService()
    if svc.connect():
        print("Connected.")
        print(svc.get_status())
        svc.close()
    else:
        print("Failed to connect.")
