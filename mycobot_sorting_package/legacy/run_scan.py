import json
import os
import time
from datetime import datetime
from motion_service import MotionService

def main():
    # 创建记录目录
    run_id = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = f"run_{run_id}"
    os.makedirs(out_dir, exist_ok=True)
    print(f"[INFO] 记录目录: {out_dir}")

    # 阶段0: 安全初始化 (AGENTS.md §4.1)
    ms = MotionService()
    print("[INFO] 连接机械臂和相机...")
    ok = ms.connect(warmup=True)
    if not ok:
        print("[ERROR] 连接失败")
        return

    print("[INFO] 开夹爪...")
    ms.gripper_open()
    time.sleep(0.5)

    print("[INFO] 回零位 go_init (speed=10)...")
    ms.go_init(speed=10)
    time.sleep(0.5)

    # 阶段1: 蛇形扫描 (SKILL.md §5.2)
    with open("path_sorting_area_rz45.json") as f:
        data = json.load(f)

    print("[INFO] 开始蛇形扫描，共 {} 个路点...".format(len(data["waypoints"])))
    for wp in data["waypoints"]:
        ms.move_to(
            x=wp["x"], y=wp["y"], z=wp["z"],
            rx=wp["rx"], ry=wp["ry"], rz=wp["rz"],
            speed=wp["speed"]
        )
        time.sleep(2.0)  # 稳定
        frame = ms.capture()
        if frame is not None:
            import cv2
            fname = os.path.join(out_dir, f"scan_{wp['name']}.jpg")
            cv2.imwrite(fname, frame)
            print(f"  [OK] {wp['name']} -> {fname}")
        else:
            print(f"  [WARN] {wp['name']} 拍照失败")

    print("[INFO] 扫描完成。回到零位...")
    ms.go_init(speed=10)
    time.sleep(1.0)
    ms.close()
    print(f"[INFO] 完成。照片保存在 {out_dir}/")

if __name__ == "__main__":
    main()
