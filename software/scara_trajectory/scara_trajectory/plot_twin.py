"""
Plot Digital Twin Comparison (real vs sim)
============================================
Author: SARIN Chandevid

Reads twin_log.csv and produces graphs for Chapter 4.5:
    - twin_theta1.png : real vs sim theta1 over time
    - twin_theta2.png : real vs sim theta2 over time
    - twin_overlay.png : both joints on same plot (2-panel)
    - twin_error.png  : sim - real error over time (shows the lag)

USAGE:
    pip install matplotlib numpy --break-system-packages
    python3 plot_twin.py
"""
import csv
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

LOG = "twin_log.csv"


def load():
    rows = list(csv.DictReader(open(LOG)))
    t   = np.array([float(r["t"])            for r in rows])
    r1  = np.array([float(r["real_theta1"])  for r in rows])
    r2  = np.array([float(r["real_theta2"])  for r in rows])
    s1  = np.array([float(r["sim_theta1"])   for r in rows])
    s2  = np.array([float(r["sim_theta2"])   for r in rows])
    d3  = np.array([float(r["sim_d3"])       for r in rows])
    return t, r1, r2, s1, s2, d3


def main():
    t, r1, r2, s1, s2, d3 = load()
    if len(t) == 0:
        print("twin_log.csv is empty - did the robots move while recording?")
        return

    r1d = np.degrees(r1)
    r2d = np.degrees(r2)
    s1d = np.degrees(s1)
    s2d = np.degrees(s2)
    e1  = s1d - r1d
    e2  = s2d - r2d

    # === Fig 1: theta1 real vs sim ===
    fig, ax = plt.subplots(figsize=(8, 4))
    ax.plot(t, r1d, "b-",  lw=1.6, label="Real robot")
    ax.plot(t, s1d, "r--", lw=1.4, label="Simulation")
    ax.set_xlabel("time (s)"); ax.set_ylabel(r"$\theta_1$ (deg)")
    ax.set_title("Joint 1 - Real Robot vs Simulation")
    ax.legend(); ax.grid(alpha=0.3)
    fig.tight_layout(); fig.savefig("twin_theta1.png", dpi=140); plt.close(fig)

    # === Fig 2: theta2 real vs sim ===
    fig, ax = plt.subplots(figsize=(8, 4))
    ax.plot(t, r2d, "g-",  lw=1.6, label="Real robot")
    ax.plot(t, s2d, "m--", lw=1.4, label="Simulation")
    ax.set_xlabel("time (s)"); ax.set_ylabel(r"$\theta_2$ (deg)")
    ax.set_title("Joint 2 - Real Robot vs Simulation")
    ax.legend(); ax.grid(alpha=0.3)
    fig.tight_layout(); fig.savefig("twin_theta2.png", dpi=140); plt.close(fig)

    # === Fig 3: 2-panel overlay ===
    fig, axes = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
    axes[0].plot(t, r1d, "b-",  lw=1.6, label="Real")
    axes[0].plot(t, s1d, "r--", lw=1.4, label="Sim")
    axes[0].set_ylabel(r"$\theta_1$ (deg)")
    axes[0].set_title("Digital Twin: Real Robot vs Simulation")
    axes[0].legend(); axes[0].grid(alpha=0.3)

    axes[1].plot(t, r2d, "g-",  lw=1.6, label="Real")
    axes[1].plot(t, s2d, "m--", lw=1.4, label="Sim")
    axes[1].set_xlabel("time (s)"); axes[1].set_ylabel(r"$\theta_2$ (deg)")
    axes[1].legend(); axes[1].grid(alpha=0.3)
    fig.tight_layout(); fig.savefig("twin_overlay.png", dpi=140); plt.close(fig)

    # === Fig 4: error sim - real ===
    fig, ax = plt.subplots(figsize=(8, 3.8))
    ax.plot(t, e1, "b-", lw=1.2, label=r"$\theta_1$ error (sim - real)")
    ax.plot(t, e2, "g-", lw=1.2, label=r"$\theta_2$ error (sim - real)")
    ax.set_xlabel("time (s)"); ax.set_ylabel("angle difference (deg)")
    ax.set_title(
        f"Sim - Real Angle Difference  "
        f"(theta1 max {abs(e1).max():.2f} deg, "
        f"theta2 max {abs(e2).max():.2f} deg)"
    )
    ax.legend(); ax.grid(alpha=0.3)
    ax.axhline(0, color="k", lw=0.5)
    fig.tight_layout(); fig.savefig("twin_error.png", dpi=140); plt.close(fig)

    print()
    print("=" * 60)
    print("DIGITAL TWIN STATISTICS (use these numbers in your thesis)")
    print("=" * 60)
    print(f"  Recording duration   : {t[-1]:.2f} s ({len(t)} samples)")
    print(f"  theta1 mean diff     : {e1.mean():+.3f} deg")
    print(f"  theta1 max |diff|    : {abs(e1).max():.3f} deg")
    print(f"  theta1 RMS diff      : {np.sqrt(np.mean(e1**2)):.3f} deg")
    print(f"  theta2 mean diff     : {e2.mean():+.3f} deg")
    print(f"  theta2 max |diff|    : {abs(e2).max():.3f} deg")
    print(f"  theta2 RMS diff      : {np.sqrt(np.mean(e2**2)):.3f} deg")
    print()
    print("Figures saved: twin_theta1.png, twin_theta2.png,")
    print("               twin_overlay.png, twin_error.png")


if __name__ == "__main__":
    main()
