# analysis.py
import pandas as pd
import numpy as np
import math


# =============================
# Load log file
# =============================
df = pd.read_csv("results/mission_log.csv")
df = df.sort_values("timestamp").reset_index(drop=True)

# Time and position arrays
t = df["timestamp"].to_numpy()
n = df["north_m"].to_numpy()
e = df["east_m"].to_numpy()
d = df["down_m"].to_numpy()   # NED down (âm là bay cao)

# ============================================================
# 1) TỔNG THỜI GIAN BAY
# ============================================================
total_time = t[-1] - t[0]
print(f"Total mission time: {total_time:.2f} sec")

# ============================================================
# 2) TỔNG QUÃNG ĐƯỜNG BAY (2D)
# ============================================================
dn = np.diff(n)
de = np.diff(e)
segment_dist = np.sqrt(dn*dn + de*de)
total_distance = np.sum(segment_dist)
print(f"Total flight distance: {total_distance:.2f} m")

# ============================================================
# 3) SỐ LẦN TRÁNH VẬT CẢN
# ============================================================
# Trong log bạn có ghi event như "avoid_start", "avoid_point"
avoid_count = df[df["event"].str.contains("avoid", na=False)].shape[0]
print(f"Avoidance count: {avoid_count}")

# ============================================================
# 4) KHOẢNG CÁCH TỐI THIỂU TỚI OBSTACLE
# ============================================================
# Nếu bạn có nhiều obstacle, liệt kê ở đây:
obstacles = [
    (5.0, 10.0),   # obstacle 1 (north, east)
    (7.0, 15.0)    # obstacle 2
]

min_d = []
for ox, oy in obstacles:
    dist = np.sqrt((n - ox)**2 + (e - oy)**2)
    min_d.append(dist.min())

print("Minimum distance to obstacles:")
for i, md in enumerate(min_d):
    print(f"  Obstacle {i+1}: {md:.2f} m")

# ============================================================
# 5) ĐỘ MƯỢT ĐƯỜNG BAY (SUM OF SQUARED ACCELERATION)
# ============================================================
# Tính vận tốc rời rạc
dt = np.diff(t)
vx = dn / dt
vy = de / dt

# Tính gia tốc rời rạc
ax = np.diff(vx) / dt[1:]
ay = np.diff(vy) / dt[1:]

smoothness = np.sum(ax*ax + ay*ay)
print(f"Smoothness (sum squared accel): {smoothness:.4f}")

