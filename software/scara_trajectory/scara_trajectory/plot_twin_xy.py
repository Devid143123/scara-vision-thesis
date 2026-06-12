"""
Plot End-Effector XY Path: Real vs Sim (digital twin, Chapter 4.5)
====================================================================
Author: SARIN Chandevid

Reads twin_log.csv (the file produced by twin_recorder.py) and applies the
SCARA forward kinematics to BOTH the real and sim angle streams. The
resulting (x, y) traces are plotted on the same axes so that the agreement
between the real robot and the simulation can be seen at a glance.

Forward kinematics:
    x = a1 * cos(theta1) + a2 * cos(theta1 + theta2)
    y = a1 * sin(theta1) + a2 * sin(theta1 + theta2)
with a1 = a2 = 0.15 m (matching the partner's IK).

Output:
    twin_xy_path.png   - the main figure for Chapter 4.5
    twin_xy_error.png  - Euclidean position error between the two paths

USAGE:
    pip install matplotlib numpy --break-system-packages
    python3 plot_twin_xy.py
"""
import csv
import math
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

LOG = "twin_log.csv"

# Link lengths (must match Reach's IK and your sim controller)
A1 = 0.15
A2 = 0.15


def fk(theta1, theta2):
    """Forward kinematics in Reach's frame: returns (x, y) in meters."""
    x = A1 * np.cos(theta1) + A2 * np.cos(theta1 + theta2)
    y = A1 * np.sin(theta1) + A2 * np.sin(theta1 + theta2)
    return x, y


def load():
    rows = list(csv.DictReader(open(LOG)))
    t   = np.array([float(r["t"])           for r in rows])
    r1  = np.array([float(r["real_theta1"]) for r in rows])
    r2  = np.array([float(r["real_theta2"]) for r in rows])
    s1  = np.array([float(r["sim_theta1"])  for r in rows])
    s2  = np.array([float(r["sim_theta2"])  for r in rows])
    return t, r1, r2, s1, s2


def main():
    t, r1, r2, s1, s2 = load()
    if len(t) == 0:
        print("twin_log.csv is empty - run twin_recorder.py first.")
        return

    rx, ry = fk(r1, r2)
    sx, sy = fk(s1, s2)

    # Convert to mm for readability
    rx_mm = rx * 1000
    ry_mm = ry * 1000
    sx_mm = sx * 1000
    sy_mm = sy * 1000

    # Euclidean error between sim and real (in mm)
    err = np.sqrt((sx - rx)**2 + (sy - ry)**2) * 1000.0

    # === Figure 1: XY path overlay ===
    fig, ax = plt.subplots(figsize=(7, 7))
    ax.plot(rx_mm, ry_mm, "b-",  lw=1.8, label="Real robot")
    ax.plot(sx_mm, sy_mm, "r--", lw=1.4, label="Simulation")
    ax.scatter(rx_mm[0], ry_mm[0], color="black", s=40, zorder=5, label="start")
    ax.scatter(rx_mm[-1], ry_mm[-1], color="orange", s=40, zorder=5, label="end")
    ax.set_xlabel("x (mm)")
    ax.set_ylabel("y (mm)")
    ax.set_title("End-Effector Path: Real Robot vs Simulation")
    ax.set_aspect("equal")
    ax.legend(loc="best")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig("twin_xy_path.png", dpi=140)
    plt.close(fig)
    print("  -> twin_xy_path.png")

    # === Figure 2: position error over time ===
    fig, ax = plt.subplots(figsize=(8, 3.8))
    ax.plot(t, err, "r-", lw=1.2)
    ax.set_xlabel("time (s)")
    ax.set_ylabel("position error (mm)")
    ax.set_title(
        f"Euclidean End-Effector Difference (Sim - Real)  "
        f"max {err.max():.2f} mm, mean {err.mean():.2f} mm"
    )
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig("twin_xy_error.png", dpi=140)
    plt.close(fig)
    print("  -> twin_xy_error.png")

    print()
    print("=" * 60)
    print("END-EFFECTOR DIGITAL TWIN STATISTICS")
    print("=" * 60)
    print(f"  Samples              : {len(t)}")
    print(f"  Duration             : {t[-1]:.2f} s")
    print(f"  Real path length     : {np.sum(np.hypot(np.diff(rx_mm), np.diff(ry_mm))):.1f} mm")
    print(f"  Sim path length      : {np.sum(np.hypot(np.diff(sx_mm), np.diff(sy_mm))):.1f} mm")
    print(f"  Max position error   : {err.max():.3f} mm")
    print(f"  Mean position error  : {err.mean():.3f} mm")
    print(f"  RMS position error   : {np.sqrt(np.mean(err**2)):.3f} mm")
    print()
    print("Write these numbers in Chapter 4.5 of your thesis.")


if __name__ == "__main__":
    main()
