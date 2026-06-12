"""
Plot Results from scara_log.csv  (matches square_demo_v3)
=========================================================
Reference square: centre (0, 0.25), side 0.10 m.

Auto-detects when the motion happens (trims rest period), produces 5 figures
and prints statistics for the thesis.

USAGE:
    pip install matplotlib numpy --break-system-packages
    python3 plot_results.py
"""
import csv, math
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

LOG = "scara_log.csv"

# === MUST MATCH square_demo_v3 ===
CENTER_X = 0.0
CENTER_Y = 0.25
SIDE     = 0.10

NEAR_SQUARE_MM = 40.0


def load():
    rows = list(csv.DictReader(open(LOG)))
    t  = np.array([float(r["t"])       for r in rows])
    t1 = np.array([float(r["theta1"])  for r in rows])
    t2 = np.array([float(r["theta2"])  for r in rows])
    j2 = np.array([float(r["j2"])      for r in rows])
    v0 = np.array([float(r["v0"])      for r in rows])
    v1 = np.array([float(r["v1"])      for r in rows])
    x  = np.array([float(r["x_ee"])    for r in rows])
    y  = np.array([float(r["y_ee"])    for r in rows])
    return t, t1, t2, j2, v0, v1, x, y


def reference_square():
    h = SIDE/2
    return np.array([
        [CENTER_X-h, CENTER_Y-h],
        [CENTER_X+h, CENTER_Y-h],
        [CENTER_X+h, CENTER_Y+h],
        [CENTER_X-h, CENTER_Y+h],
        [CENTER_X-h, CENTER_Y-h],
    ])


def perp_dist(px, py, ax, ay, bx, by):
    dx, dy = bx-ax, by-ay
    if dx == 0 and dy == 0: return math.hypot(px-ax, py-ay)
    s = ((px-ax)*dx + (py-ay)*dy) / (dx*dx + dy*dy)
    s = max(0.0, min(1.0, s))
    cx, cy = ax + s*dx, ay + s*dy
    return math.hypot(px-cx, py-cy)


def tracking_error_mm(x, y, sq):
    err = np.zeros(len(x))
    for i in range(len(x)):
        d = [perp_dist(x[i], y[i], sq[j,0], sq[j,1], sq[j+1,0], sq[j+1,1]) for j in range(4)]
        err[i] = min(d)
    return err * 1000.0


def find_motion_window(x, y, sq):
    err = tracking_error_mm(x, y, sq)
    near = err < NEAR_SQUARE_MM
    if not np.any(near): return 0, len(x)-1
    idx = np.where(near)[0]
    return idx[0], idx[-1]


def main():
    t, t1, t2, j2, v0, v1, x, y = load()
    sq = reference_square()

    print(f"Reference square center: ({CENTER_X}, {CENTER_Y}) m, side {SIDE} m")
    print(f"Total recorded samples : {len(t)}, duration {t[-1]:.2f}s")

    i0, i1 = find_motion_window(x, y, sq)
    print(f"Motion window detected : samples {i0}..{i1}, "
          f"time {t[i0]:.2f}..{t[i1]:.2f}s ({t[i1]-t[i0]:.2f}s of motion)")

    t  = t[i0:i1+1] - t[i0]
    t1 = t1[i0:i1+1]; t2 = t2[i0:i1+1]; j2 = j2[i0:i1+1]
    v0 = v0[i0:i1+1]; v1 = v1[i0:i1+1]
    x  = x[i0:i1+1];  y  = y[i0:i1+1]
    err = tracking_error_mm(x, y, sq)

    # Fig 1: XY path
    fig, ax = plt.subplots(figsize=(6,6))
    ax.plot(sq[:,0]*1000, sq[:,1]*1000, "k--", lw=1.5, label="Reference square")
    ax.plot(x*1000, y*1000, "b-", lw=1.2, label="Measured path")
    ax.set_xlabel("x (mm)"); ax.set_ylabel("y (mm)")
    ax.set_title("End-Effector Path Tracking")
    ax.legend(); ax.grid(True, alpha=0.3); ax.set_aspect("equal")
    fig.tight_layout(); fig.savefig("xy_path.png", dpi=140); plt.close(fig)

    # Fig 2: tracking error
    fig, ax = plt.subplots(figsize=(8,3.6))
    ax.plot(t, err, "r-", lw=1.0)
    ax.set_xlabel("time (s)"); ax.set_ylabel("tracking error (mm)")
    ax.set_title(f"Tracking Error (max {err.max():.3f} mm, mean {err.mean():.3f} mm)")
    ax.grid(True, alpha=0.3)
    fig.tight_layout(); fig.savefig("tracking_error.png", dpi=140); plt.close(fig)

    # Fig 3: joint positions (in Reach's IK frame - clearer for the thesis)
    fig, axes = plt.subplots(2, 1, figsize=(8,6), sharex=True)
    axes[0].plot(t, np.degrees(t1), "b", label=r"$\theta_1$")
    axes[0].plot(t, np.degrees(t2), "g", label=r"$\theta_2$")
    axes[0].set_ylabel("angle (deg)")
    axes[0].set_title("Joint Angles During Square Trajectory")
    axes[0].legend(); axes[0].grid(alpha=0.3)
    axes[1].plot(t, j2*1000, "m"); axes[1].set_ylabel("d3 (mm)"); axes[1].grid(alpha=0.3)
    axes[1].set_xlabel("time (s)")
    fig.tight_layout(); fig.savefig("joint_positions.png", dpi=140); plt.close(fig)

    # Fig 4: joint velocities
    if np.allclose(v0, 0) and np.allclose(v1, 0):
        v0 = np.gradient(t1, t); v1 = np.gradient(t2, t)
        vel_note = " (computed from positions)"
    else:
        vel_note = ""
    fig, ax = plt.subplots(figsize=(8,3.8))
    ax.plot(t, v0, "b", label=r"$\dot\theta_1$")
    ax.plot(t, v1, "g", label=r"$\dot\theta_2$")
    ax.set_xlabel("time (s)"); ax.set_ylabel("velocity (rad/s)")
    ax.set_title(f"Joint Velocities During Square Trajectory{vel_note}")
    ax.legend(); ax.grid(True, alpha=0.3)
    fig.tight_layout(); fig.savefig("joint_velocities.png", dpi=140); plt.close(fig)

    # Fig 5: summary
    fig, axes = plt.subplots(2, 2, figsize=(11,8))
    axes[0,0].plot(sq[:,0]*1000, sq[:,1]*1000, "k--", lw=1.5, label="Reference")
    axes[0,0].plot(x*1000, y*1000, "b-", lw=1.2, label="Measured")
    axes[0,0].set_xlabel("x (mm)"); axes[0,0].set_ylabel("y (mm)")
    axes[0,0].set_title("End-Effector Path"); axes[0,0].legend()
    axes[0,0].grid(alpha=0.3); axes[0,0].set_aspect("equal")
    axes[0,1].plot(t, err, "r-"); axes[0,1].set_xlabel("time (s)")
    axes[0,1].set_ylabel("error (mm)"); axes[0,1].set_title("Tracking Error")
    axes[0,1].grid(alpha=0.3)
    axes[1,0].plot(t, np.degrees(t1), "b", label=r"$\theta_1$")
    axes[1,0].plot(t, np.degrees(t2), "g", label=r"$\theta_2$")
    axes[1,0].set_xlabel("time (s)"); axes[1,0].set_ylabel("angle (deg)")
    axes[1,0].set_title("Joint Angles"); axes[1,0].legend(); axes[1,0].grid(alpha=0.3)
    axes[1,1].plot(t, v0, "b", label=r"$\dot\theta_1$")
    axes[1,1].plot(t, v1, "g", label=r"$\dot\theta_2$")
    axes[1,1].set_xlabel("time (s)"); axes[1,1].set_ylabel("velocity (rad/s)")
    axes[1,1].set_title("Joint Velocities"); axes[1,1].legend(); axes[1,1].grid(alpha=0.3)
    fig.suptitle("SCARA Simulation - Square Trajectory Results", fontsize=14, fontweight="bold")
    fig.tight_layout(); fig.savefig("summary.png", dpi=140); plt.close(fig)

    print()
    print("=" * 60)
    print("RESULTS SUMMARY (use these numbers in your thesis)")
    print("=" * 60)
    print(f"  Samples in motion:      {len(t)}")
    print(f"  Motion duration:        {t[-1]:.2f} s")
    print(f"  Max tracking error:     {err.max():.3f} mm")
    print(f"  Mean tracking error:    {err.mean():.3f} mm")
    print(f"  RMS tracking error:     {np.sqrt(np.mean(err**2)):.3f} mm")
    print(f"  Max |theta1|:           {np.degrees(np.abs(t1).max()):.2f} deg")
    print(f"  Max |theta2|:           {np.degrees(np.abs(t2).max()):.2f} deg")
    print(f"  Max joint velocity:     {max(abs(v0).max(), abs(v1).max()):.4f} rad/s")
    print()
    print("Figures saved: xy_path.png, tracking_error.png, joint_positions.png,")
    print("               joint_velocities.png, summary.png")


if __name__ == "__main__":
    main()
