#!/usr/bin/env python3
"""
MoveJMulti Demo — 多路径点关节空间运动

向 GeniArm 发送 MoveJMulti RPC 请求，控制机械臂依次经过
多个关节角度路径点，路径点之间做平滑过渡（r 为 TCP 侧的退让距离，单位 m）。

用法:
    python3 movej_multi_demo.py
"""

import requests
import json

# RPC endpoint
URL = "http://127.0.0.1:50080/rpc/genisys_msgs/srv/MoveJMulti"

# 多个关节角度路径点 [joint1, joint2, joint3, joint4, joint5, joint6] (弧度)
payload = {
    "position": [
        {"position": [0.0, -0.2618, 1.7453, 0.4363, 1.5708, 0.0]},      # AUBO 路点 1
        {"position": [0.6269, -0.1969, 1.0465, -0.3274, 1.5708, 0.6269]},  # AUBO 路点 2
        {"position": [0.7163, -0.1335, 1.7244, 0.2869, 1.5708, 0.2032]},   # AUBO 路点 3
    ],
    "r": 0.0,   # TCP 过渡退让距离 (m)，AUBO 示例为 0（逐点停）
    "v": 1.0472,  # 速度 (rad/s) = 60 度/秒
    "a": 1.3963   # 加速度 (rad/s^2) = 80 度/秒^2
}

print("=" * 60)
print("MoveJMulti Demo — 多路径点关节运动")
print("=" * 60)
print("Waypoints:")
for i, wp in enumerate(payload["position"]):
    print(f"  [{i}] {wp['position']}")
print(f"Blend distance: {payload['r']} m (TCP)")
print(f"Velocity: {payload['v']} rad/s, Acceleration: {payload['a']} rad/s^2")
print(f"Sending to {URL} ...")

try:
    resp = requests.post(URL, json=payload, timeout=60)
    resp.raise_for_status()
    result = resp.json()
    print(f"\nResponse: {json.dumps(result, indent=2)}")

    code = result.get("error", {}).get("code", -1)
    if int(code) == 0:
        print("\n✅ MoveJMulti succeeded!")
    else:
        print(f"\n❌ MoveJMulti failed (code={code}): {result.get('error', {}).get('msg', 'unknown')}")

except requests.exceptions.ConnectionError:
    print("\n❌ 无法连接到 GeniArm (127.0.0.1:50080)，请确认程序已启动。")
except requests.exceptions.Timeout:
    print("\n❌ 请求超时，轨迹执行时间过长。")
except requests.exceptions.RequestException as e:
    print(f"\n❌ 请求失败: {e}")
