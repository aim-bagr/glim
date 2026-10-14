#!/usr/bin/env python3
"""Compare two GLIM output directories from repeated runs on the same bag.

usage: compare_glim_runs.py <out_a> <out_b>
"""
import hashlib
import sys
from pathlib import Path

import numpy as np

if len(sys.argv) < 3:
    print(f"Usage: {sys.argv[0]} <out_a> <out_b>")
    sys.exit(1)

a, b = Path(sys.argv[1]), Path(sys.argv[2])

# Compare traj_lidar.txt
if (a / "traj_lidar.txt").exists() and (b / "traj_lidar.txt").exists():
    ta, tb = np.loadtxt(a / "traj_lidar.txt"), np.loadtxt(b / "traj_lidar.txt")
    print(f"traj poses: {len(ta)} vs {len(tb)}")
    n = min(len(ta), len(tb))
    same_stamps = np.array_equal(ta[:n, 0], tb[:n, 0])
    d = np.linalg.norm(ta[:n, 1:4] - tb[:n, 1:4], axis=1)
    traj_byte_identical = (a / "traj_lidar.txt").read_bytes() == (b / "traj_lidar.txt").read_bytes()
    print(f"traj_lidar.txt byte-identical: {traj_byte_identical}")
    print(f"stamps identical: {same_stamps}; position diff: mean {d.mean() * 100:.4f} cm, max {d.max() * 100:.4f} cm")
else:
    print("traj_lidar.txt missing in one or both directories")

# Compare odom_lidar.txt
if (a / "odom_lidar.txt").exists() and (b / "odom_lidar.txt").exists():
    oa, ob = np.loadtxt(a / "odom_lidar.txt"), np.loadtxt(b / "odom_lidar.txt")
    print(f"odom poses: {len(oa)} vs {len(ob)}")
    no = min(len(oa), len(ob))
    do = np.linalg.norm(oa[:no, 1:4] - ob[:no, 1:4], axis=1)
    odom_byte_identical = (a / "odom_lidar.txt").read_bytes() == (b / "odom_lidar.txt").read_bytes()
    print(f"odom_lidar.txt byte-identical: {odom_byte_identical}")
    print(f"odom position diff: mean {do.mean() * 100:.6f} cm, max {do.max() * 100:.6f} cm")
    if len(do) > 1:
        print(f"first odom step diff: {do[1] * 1000:.6f} mm")

# Compare submaps
sa = sorted(p.name for p in a.iterdir() if p.is_dir() and p.name.isdigit())
sb = sorted(p.name for p in b.iterdir() if p.is_dir() and p.name.isdigit())
print(f"submaps: {len(sa)} vs {len(sb)}")
same = [
    n for n in sa
    if n in sb and (a / n / "data.txt").exists() and (b / n / "data.txt").exists()
    and hashlib.md5((a / n / "data.txt").read_bytes()).digest() == hashlib.md5((b / n / "data.txt").read_bytes()).digest()
]
print(f"submap data.txt identical: {len(same)}/{min(len(sa), len(sb)) if sa and sb else 0}")
first = next((n for n in sa if n in sb and n not in same), None)
if first:
    print(f"first differing submap: {first}")
