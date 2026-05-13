#!/usr/bin/env python3
"""
wave_hand.py
==============
让机械臂招招手（手腕摆动模拟挥手）。

安全流程:
  1. connect() -> go_init() -> gripper_open()
  2. 移动到挥手起始位姿
  3. J6 关节左右摆动 N 次
  4. 回到 go_init() -> close()
"""

import time
import logging
from motion_service import MotionService

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
logger = logging.getLogger(__name__)


def wave(
    ms: MotionService,
    cycles: int = 4,
    angle: float = 35.0,
    speed: int = 40,
):
    """
    让 J6 关节左右摆动，模拟招手。

    Args:
        cycles: 摆动次数（左右各一次算一周期）
        angle:  摆动角度幅度 (deg)
        speed:  J6 转动速度
    """
    logger.info("👋 开始招手: %d 周期, 幅度 ±%.0f°", cycles, angle)

    for i in range(cycles):
        # 向左摆
        logger.info("  → 周期 %d/%d: 左摆 +%.0f°", i + 1, cycles, angle)
        ms.rotate_j6(angle, speed=speed, wait=True)
        time.sleep(0.2)

        # 向右摆
        logger.info("  → 周期 %d/%d: 右摆 -%.0f°", i + 1, cycles, angle)
        ms.rotate_j6(-angle, speed=speed, wait=True)
        time.sleep(0.2)

    # 回正中
    ms.rotate_j6(0.0, speed=speed, wait=True)
    logger.info("👋 招手结束")


def main():
    ms = MotionService()

    logger.info("连接机械臂...")
    if not ms.connect(warmup=True):
        logger.error("连接失败，请检查串口和电源")
        return

    try:
        logger.info("打开夹爪...")
        ms.gripper_open()
        time.sleep(0.5)

        logger.info("回到初始位姿...")
        ms.go_init(speed=10)
        time.sleep(1.0)

        # 移动到“伸手”位姿——手伸向前上方，适合挥手
        # 这个位姿经过安全边界检查，远离奇异点
        logger.info("移动到挥手位姿...")
        ms.move_to(x=120, y=0, z=260, rx=-180, ry=0, rz=0, speed=20, wait=True)
        time.sleep(1.0)

        # 执行招手
        wave(ms, cycles=4, angle=35.0, speed=40)

        logger.info("回到初始位姿...")
        ms.go_init(speed=10)
        time.sleep(1.0)

    except Exception as exc:
        logger.exception("招手过程中出错: %s", exc)
    finally:
        ms.close()
        logger.info("已断开连接")


if __name__ == "__main__":
    main()
