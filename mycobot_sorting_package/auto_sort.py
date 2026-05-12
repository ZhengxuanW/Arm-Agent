#!/usr/bin/env python3
"""
auto_sort.py
自动分拣脚本：根据扫描结果识别桌面物品，执行迭代对准、抓取和放置。
"""
import time
import cv2
import numpy as np
import os
from datetime import datetime
from motion_service import MotionService

# ---------- 配置 ----------
SCALE_Z200 = 0.32          # Z=200mm 时像素比例 mm/px
ALIGN_RATIO = 0.6          # 迭代修正比例 60%
ALIGN_THRESH_PX = 20       # 像素收敛阈值
MAX_ALIGN_ITERS = 6        # 最大迭代次数

# 扫描推断的起始位置（红色盒子大致在桌面中央偏右）
INITIAL_SCAN_X = 170
INITIAL_SCAN_Y = 0

# 目标区域（红色 -> R区）
# marker_map.json: R release = [60, 230]
RELEASE_X = 60
RELEASE_Y = 230
RELEASE_Z = 105

RUN_ID = datetime.now().strftime("%Y%m%d_%H%M%S")
OUT_DIR = f"sort_{RUN_ID}"
os.makedirs(OUT_DIR, exist_ok=True)

# ---------- 颜色检测 ----------
def find_red_object(frame):
    """在画面中检测红色物品，返回 (cx, cy, area) 或 None"""
    if frame is None:
        return None
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # 红色在 HSV 中跨越 0°，需要两个区间
    lower1 = np.array([0, 80, 80])
    upper1 = np.array([12, 255, 255])
    lower2 = np.array([160, 80, 80])
    upper2 = np.array([180, 255, 255])
    mask = cv2.inRange(hsv, lower1, upper1) | cv2.inRange(hsv, lower2, upper2)

    # 排除画面边缘大面积背景干扰（桌面标记有时带红/橙色）
    # 形态学开闭去除噪点
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None

    # 选择面积最大的轮廓，并过滤掉太小或太大的
    candidates = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if 300 < area < 40000:  # 合理物品面积范围（640x480画面）
            M = cv2.moments(cnt)
            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                candidates.append((area, cx, cy, cnt))

    if not candidates:
        return None

    candidates.sort(reverse=True)
    area, cx, cy, cnt = candidates[0]
    return cx, cy, area


def save_debug(frame, name, obj=None):
    """保存带标注的调试图"""
    vis = frame.copy()
    h, w = vis.shape[:2]
    cv2.drawMarker(vis, (w // 2, h // 2), (0, 0, 255), cv2.MARKER_CROSS, 30, 2)
    if obj:
        cx, cy, area = obj
        cv2.circle(vis, (cx, cy), 8, (0, 255, 0), 2)
        cv2.putText(vis, f"({cx},{cy}) a={area}", (cx + 10, cy - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    path = os.path.join(OUT_DIR, name)
    cv2.imwrite(path, vis)
    print(f"  [DEBUG] 已保存 {path}")


# ---------- 迭代对准 ----------
def iterative_align(ms, start_x, start_y):
    """
    从 (start_x, start_y, 200) 开始迭代对准红色物品。
    返回对准后的 (camera_x, camera_y, final_cx, final_cy)。
    """
    cam_x, cam_y = start_x, start_y
    for i in range(MAX_ALIGN_ITERS):
        print(f"\n[INFO] 对准迭代 {i + 1}/{MAX_ALIGN_ITERS}: 移动到 ({cam_x:.1f}, {cam_y:.1f}, 200)")
        ms.move_to(x=cam_x, y=cam_y, z=200, rx=-180, ry=0, rz=45, speed=15)
        time.sleep(1.2)  # 稳定+拍照

        frame = ms.capture()
        if frame is None:
            print("[WARN] 拍照失败，重试...")
            time.sleep(0.5)
            frame = ms.capture()
            if frame is None:
                print("[ERROR] 连续拍照失败")
                break

        obj = find_red_object(frame)
        save_debug(frame, f"align_{i + 1}.jpg", obj)

        if obj is None:
            print("[WARN] 未检测到红色物品")
            # 如果第一次就找不到，尝试在周边小范围搜索
            if i == 0:
                print("[INFO] 尝试周边搜索...")
                # 简单搜索附近4个点
                for dx, dy in [(0, -40), (0, 40), (-40, 0), (40, 0)]:
                    try_x, try_y = cam_x + dx, cam_y + dy
                    print(f"  [INFO] 搜索 ({try_x:.1f}, {try_y:.1f})")
                    ms.move_to(x=try_x, y=try_y, z=200, rx=-180, ry=0, rz=45, speed=15)
                    time.sleep(1.0)
                    f = ms.capture()
                    save_debug(f, f"search_{dx}_{dy}.jpg", find_red_object(f))
                    if f is not None and find_red_object(f) is not None:
                        cam_x, cam_y = try_x, try_y
                        obj = find_red_object(f)
                        print(f"  [OK] 在 ({try_x:.1f}, {try_y:.1f}) 找到物品")
                        break
                else:
                    print("[ERROR] 周边搜索失败")
                    return None
            else:
                return None

        cx, cy, area = obj
        h, w = frame.shape[:2]
        dx_px = cx - w // 2
        dy_px = cy - h // 2
        print(f"[INFO] 像素偏移 dx_px={dx_px}, dy_px={dy_px}, area={area}")

        if abs(dx_px) < ALIGN_THRESH_PX and abs(dy_px) < ALIGN_THRESH_PX:
            print("[INFO] 对准已收敛")
            return cam_x, cam_y, cx, cy

        # 坐标修正（AGENTS.md 规则）
        # 画面上方 -> X增加, 画面下方 -> X减少
        # 画面左方 -> Y增加, 画面右方 -> Y减少
        # 图像坐标: y向下增加, x向右增加
        dX = -dy_px * SCALE_Z200 * ALIGN_RATIO
        dY = -dx_px * SCALE_Z200 * ALIGN_RATIO
        cam_x += dX
        cam_y += dY
        print(f"[INFO] 修正 dX={dX:.1f}, dY={dY:.1f} -> 下次目标 ({cam_x:.1f}, {cam_y:.1f})")

    # 最后一次拍照确认
    print(f"\n[INFO] 最终确认位置 ({cam_x:.1f}, {cam_y:.1f}, 200)")
    ms.move_to(x=cam_x, y=cam_y, z=200, rx=-180, ry=0, rz=45, speed=15)
    time.sleep(1.0)
    frame = ms.capture()
    obj = find_red_object(frame)
    save_debug(frame, "align_final.jpg", obj)
    if obj:
        return cam_x, cam_y, obj[0], obj[1]
    return None


# ---------- 主流程 ----------
def main():
    ms = MotionService()
    print("[INFO] 连接机械臂与相机...")
    if not ms.connect(warmup=True):
        print("[ERROR] 连接失败，退出")
        return

    try:
        # 1. 安全初始化
        print("[INFO] 打开夹爪...")
        ms.gripper_open()
        time.sleep(0.5)

        print("[INFO] 回到 go_init (speed=10)...")
        ms.go_init(speed=10)
        time.sleep(1.0)

        # 2. 迭代对准红色物品
        print(f"[INFO] 开始迭代对准，起始位置 ({INITIAL_SCAN_X}, {INITIAL_SCAN_Y}, 200)")
        result = iterative_align(ms, INITIAL_SCAN_X, INITIAL_SCAN_Y)
        if result is None:
            print("[ERROR] 对准失败，终止任务")
            return

        camera_x, camera_y, final_cx, final_cy = result
        print(f"\n[OK] 对准完成: 相机坐标 ({camera_x:.1f}, {camera_y:.1f}), 画面中心偏移 ({final_cx}, {final_cy})")

        # 3. 应用 +30mm X 偏移得到抓取坐标
        grasp_x = camera_x + 30
        grasp_y = camera_y
        print(f"[INFO] 抓取坐标 -> ({grasp_x:.1f}, {grasp_y:.1f})")

        # 4. 悬停过渡
        print("[INFO] 悬停 z=140...")
        ms.move_to(x=grasp_x, y=grasp_y, z=140, rx=-180, ry=0, rz=45, speed=15)
        time.sleep(1.0)

        # 5. 降下抓取
        print("[INFO] 降下抓取 z=95 (speed=10)...")
        ms.move_to(x=grasp_x, y=grasp_y, z=95, rx=-180, ry=0, rz=45, speed=10)
        time.sleep(1.0)

        # 6. 关闭夹爪
        print("[INFO] 关闭夹爪...")
        ms.gripper_close()
        time.sleep(1.0)

        # 7. 抬升
        print("[INFO] 抬升 z=140...")
        ms.move_to(x=grasp_x, y=grasp_y, z=140, rx=-180, ry=0, rz=45, speed=15)
        time.sleep(1.0)

        # 8. 移动到目标区上方 (R区)
        print(f"[INFO] 移动到释放区上方 ({RELEASE_X}, {RELEASE_Y}, 140)...")
        ms.move_to(x=RELEASE_X, y=RELEASE_Y, z=140, rx=-180, ry=0, rz=45, speed=20)
        time.sleep(1.0)

        # 9. 降下放置
        print("[INFO] 降下放置 z=105 (speed=15)...")
        ms.move_to(x=RELEASE_X, y=RELEASE_Y, z=RELEASE_Z, rx=-180, ry=0, rz=45, speed=15)
        time.sleep(1.0)

        # 10. 打开夹爪
        print("[INFO] 打开夹爪释放...")
        ms.gripper_open()
        time.sleep(0.5)

        # 11. 抬升撤离
        print("[INFO] 抬升撤离 z=140...")
        ms.move_to(x=RELEASE_X, y=RELEASE_Y, z=140, rx=-180, ry=0, rz=45, speed=20)
        time.sleep(1.0)

        # 12. 回到零位
        print("[INFO] 回到 go_init...")
        ms.go_init(speed=10)
        time.sleep(1.0)

        print("\n[OK] 分拣任务完成！物品已放置到 R 区（红色标记区）")

    except Exception as exc:
        print(f"[ERROR] 异常: {exc}")
        import traceback
        traceback.print_exc()

    finally:
        print("[INFO] 断开连接...")
        ms.close()


if __name__ == "__main__":
    main()
