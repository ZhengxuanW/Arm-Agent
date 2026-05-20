"""
motion_service.py
=================
机械臂运动控制核心服务 — 硬件访问层。

职责（精简后）：
  - 连接/断开机械臂和相机
  - 执行笛卡尔坐标运动、关节运动
  - 拍照（单帧 + 连续流）
  - 安全边界检查（已恢复，非 pass）
  - 夹爪控制

重要变更：
  - 单例已移除。所有上层代码应通过 robot_service.py HTTP API 访问硬件，
    不应直接 import 本模块。
  - 安全边界检查已恢复（不再 pass），超限将抛出 RuntimeError。
  - 所有硬编码参数已迁移到 config.py。
"""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass, field
from typing import Optional

import cv2
import numpy as np

from config import (
    END_CAMERA_FPS,
    END_CAMERA_HEIGHT,
    END_CAMERA_INDEX,
    END_CAMERA_WIDTH,
    HOME_COORDS,
    HOME_SPEED,
    INIT_ANGLES,
    ROBOT_BAUD,
    ROBOT_PORT,
    SAFE_MAX_SPEED,
    SAFE_X_LIMIT,
    SAFE_Y_LIMIT,
    SAFE_Z_LIMIT,
    SINGULARITY_J5_THRESH,
    get_safe_speed,
    is_within_safe_boundary,
)
from vision import EndEffectorCamera

logger = logging.getLogger(__name__)


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

    安全增强：
      - 边界检查：超出 SAFE_*_LIMIT 将拒绝运动并抛出异常
      - 奇异点检测：J5 接近 0° 时发出警告
      - 自动限速：大距离运动时自动降低速度
    """

    def __init__(
        self,
        port: str = ROBOT_PORT,
        baud: int = ROBOT_BAUD,
        camera_source: int = END_CAMERA_INDEX,
        camera_width: int = END_CAMERA_WIDTH,
        camera_height: int = END_CAMERA_HEIGHT,
        camera_fps: int = END_CAMERA_FPS,
    ):
        self.port = port
        self.baud = baud
        self.camera_source = camera_source
        self.camera_width = camera_width
        self.camera_height = camera_height
        self.camera_fps = camera_fps

        self.robot = None
        self.cam: Optional[EndEffectorCamera] = None
        self._connected = False

        # 后台任务管理
        self._tasks: dict[str, TaskStatus] = {}
        self._task_lock = threading.Lock()
        self._task_counter = 0

    # ------------------------------------------------------------------
    # 连接管理
    # ------------------------------------------------------------------

    def connect(self, warmup: bool = True) -> bool:
        """连接机械臂和相机。"""
        from pymycobot import MyCobot
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
        )
        if not self.cam.open():
            logger.warning("Camera open failed — running in arm-only mode.")
            self.cam = None  # 仅移除相机，不阻断机械臂

        if self.cam:
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
        """
        安全边界检查。
        超出安全范围时抛出 RuntimeError，防止碰撞。
        """
        if not is_within_safe_boundary(x, y, z):
            raise RuntimeError(
                f"坐标 ({x}, {y}, {z}) 超出安全工作空间! "
                f"X范围{SAFE_X_LIMIT}, Y范围{SAFE_Y_LIMIT}, Z范围{SAFE_Z_LIMIT}"
            )

    def _is_near_singularity(self, angles: list, threshold: float = SINGULARITY_J5_THRESH) -> bool:
        """检查是否接近奇异点（J5 接近 0° 时 J4/J6 对齐）"""
        if not angles or len(angles) < 5:
            return False
        j5 = abs(angles[4])
        return j5 < threshold

    # ------------------------------------------------------------------
    # 运动控制（同步）
    # ------------------------------------------------------------------

    def move_to(
        self,
        x: float,
        y: float,
        z: float,
        rx: float = -180.0,
        ry: float = 0.0,
        rz: Optional[float] = None,
        speed: int = 20,
        wait: bool = True,
        timeout: float = 15.0,
        poll_interval: float = 0.05,
    ) -> dict:
        """
        运动到指定笛卡尔坐标。
        rz=None 时自动保持当前视角角度。
        """
        if not self._connected:
            raise RuntimeError("Not connected. Call connect() first.")

        self._check_boundary(x, y, z)

        # 获取当前状态
        current_angles = self.robot.get_angles()
        current_coords = self.robot.get_coords()

        # 奇异点警告
        if self._is_near_singularity(current_angles):
            logger.warning("NEAR SINGULARITY (J5=%.1f°). Movement may be unstable!", current_angles[4])

        # 视角解耦
        if rz is None:
            current_rz = current_coords[5] if current_coords and len(current_coords) >= 6 else 0.0
            effective_rz = current_rz
            logger.info("MOVE_TO keeping current rz=%.1f", effective_rz)
        else:
            effective_rz = float(rz)

        target = [float(x), float(y), float(z), float(rx), float(ry), effective_rz]

        # 安全速度
        safe_speed = get_safe_speed(current_coords, target, speed)
        safe_speed = min(safe_speed, SAFE_MAX_SPEED)

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
        reached = False
        position_error = None
        if actual and len(actual) >= 3:
            position_error = [round(float(actual[i]) - float(target[i]), 2) for i in range(3)]
            reached = all(abs(err) <= 15.0 for err in position_error)

        result = {
            "success": bool(stopped and reached),
            "target": target,
            "actual": actual,
            "position_error": position_error,
            "reason": None if reached else "actual position did not reach target within 15mm",
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
        """单独转动 J6 关节（末端自转 / 视角旋转）"""
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
        speed: int = HOME_SPEED,
        wait: bool = True,
    ) -> dict:
        """回到 HOME 位姿（笛卡尔空间）"""
        home = coords or HOME_COORDS
        return self.move_to(*home, speed=speed, wait=wait)

    def go_init(self, speed: int = HOME_SPEED, wait: bool = True) -> dict:
        """回到初始化位姿（关节空间，安全近零位）"""
        if not self._connected:
            raise RuntimeError("Not connected. Call connect() first.")

        init_angles = INIT_ANGLES
        logger.info("GO_INIT %s speed=%d", init_angles, speed)
        t0 = time.time()
        self.robot.send_angles(init_angles, speed)

        if not wait:
            return {"success": True, "angles": init_angles, "elapsed_sec": time.time() - t0}

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
        speed: int = 20,
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
        speed: int = 20,
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
# 注意：不再提供全局单例。请通过 robot_service.py HTTP API 访问硬件。
# ------------------------------------------------------------------
