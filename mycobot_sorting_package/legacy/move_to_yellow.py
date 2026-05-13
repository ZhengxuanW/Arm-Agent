#!/usr/bin/env python3
"""移动到黄色方块上方"""
import time
from motion_service import MotionService

ms = MotionService()
print("[INFO] 连接机械臂...")
if not ms.connect(warmup=True):
    print("[ERROR] 连接失败")
    exit(1)

try:
    print("[INFO] 打开夹爪...")
    ms.gripper_open()
    time.sleep(0.5)

    print("[INFO] 回零位...")
    ms.go_init(speed=10)
    time.sleep(1.0)

    # 黄色方块上方（扫描高度）
    target_x, target_y, target_z = 205, 0, 200
    print(f"[INFO] 移动到黄色方块上方 ({target_x}, {target_y}, {target_z})...")
    result = ms.move_to(
        x=target_x, y=target_y, z=target_z,
        rx=-180, ry=0, rz=45, speed=20, wait=True
    )
    print(f"[INFO] 到位结果: {result}")
    time.sleep(2.0)

    print("[INFO] 当前状态:", ms.get_status())
    print("[INFO] 机械臂已位于黄色方块正上方，准备就绪。")
    input("按 Enter 键回到零位并断开...")

    ms.go_init(speed=10)
    time.sleep(1.0)
finally:
    ms.close()
    print("[INFO] 已断开连接")
