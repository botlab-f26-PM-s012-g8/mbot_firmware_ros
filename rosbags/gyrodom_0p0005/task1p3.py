"""Task 1.3: compare wheel odometry (/odom) with gyrodometry (/gyrodom) on a square path."""
import os

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

HERE = os.path.dirname(os.path.abspath(__file__))
CSV_DIR = os.path.join(HERE, "csv_output")
SQUARE_SIDE = 1.0  # [m] side length of the traced square
STATIONARY_S = 5.0  # [s] robot is still at the start; used to estimate gyro bias
OUT_NAME = "task1p3_grydom_0p0005"


def load(name):
    d = pd.read_csv(os.path.join(CSV_DIR, name))
    t = d["header.stamp.sec"] + d["header.stamp.nanosec"] * 1e-9
    theta = 2.0 * np.arctan2(d["pose.pose.orientation.z"], d["pose.pose.orientation.w"])
    return pd.DataFrame({
        "t": t.to_numpy(),
        "x": d["pose.pose.position.x"].to_numpy(),
        "y": d["pose.pose.position.y"].to_numpy(),
        "theta": np.unwrap(theta.to_numpy()),
        "wz": d["twist.twist.angular.z"].to_numpy(),
    })


odom = load("odom.csv")
gyrodom = load("gyrodom.csv")
t0 = min(odom.t[0], gyrodom.t[0])
odom.t -= t0
gyrodom.t -= t0

# Slip-free heading reference: integrate the raw gyro rate (gyrodom twist.angular.z)
# after removing the bias measured while the robot sat still.
bias = gyrodom.wz[gyrodom.t < STATIONARY_S].mean()
gyro_theta = np.concatenate(([0.0], np.cumsum(0.5 * (gyrodom.wz[1:].to_numpy() + gyrodom.wz[:-1].to_numpy() - 2 * bias)
                                               * np.diff(gyrodom.t))))
gyro_theta_on_odom = np.interp(odom.t, gyrodom.t, gyro_theta)

# End-of-lap errors: the robot started and ended at the origin facing 0 (360 deg after a full lap)
pos_err = {k: np.hypot(df.x.iloc[-1], df.y.iloc[-1]) for k, df in (("odom", odom), ("gyrodom", gyrodom))}
head_err = {"odom": np.degrees(odom.theta.iloc[-1]) - 360.0,
            "gyrodom": np.degrees(gyrodom.theta.iloc[-1]) - 360.0,
            "gyro": np.degrees(gyro_theta[-1]) - 360.0}

# Reference square: start at origin facing +x, turning CCW
sq = SQUARE_SIDE * np.array([[0, 0], [1, 0], [1, 1], [0, 1], [0, 0]])

# ---------------------------------------------------------------- figure ----
plt.rcParams.update({
    "font.family": "serif",
    "mathtext.fontset": "dejavuserif",
    "font.size": 9,
    "axes.labelsize": 9,
    "axes.titlesize": 9.5,
    "legend.fontsize": 8,
    "xtick.labelsize": 8,
    "ytick.labelsize": 8,
    "axes.linewidth": 0.7,
    "grid.linewidth": 0.5,
    "grid.alpha": 0.35,
    "lines.linewidth": 1.4,
    "savefig.bbox": "tight",
    "savefig.pad_inches": 0.03,
})

C_ODOM, C_GYRO, C_GREF, C_REF = "#1f77b4", "#d95f02", "0.15", "0.55"

fig = plt.figure(figsize=(7.0, 4.6), constrained_layout=True)  # full text width of a report page
gs = fig.add_gridspec(4, 2, width_ratios=[1.0, 1.05])
ax_xy = fig.add_subplot(gs[:, 0])
ax_x = fig.add_subplot(gs[0, 1])
ax_y = fig.add_subplot(gs[1, 1], sharex=ax_x)
ax_th = fig.add_subplot(gs[2, 1], sharex=ax_x)
ax_err = fig.add_subplot(gs[3, 1], sharex=ax_x)

# (a) XY trajectory
ax_xy.plot(sq[:, 0], sq[:, 1], "--", color=C_REF, lw=1.0, label=f"Reference ({SQUARE_SIDE:g} m square)")
ax_xy.plot(odom.x, odom.y, color=C_ODOM, label="Odometry (wheels)")
ax_xy.plot(gyrodom.x, gyrodom.y, color=C_GYRO, label="Gyrodometry (wheels + IMU)")
ax_xy.plot(0, 0, marker="*", color="k", ms=9, ls="none", label="Start")
for df, c in ((odom, C_ODOM), (gyrodom, C_GYRO)):
    ax_xy.plot(df.x.iloc[-1], df.y.iloc[-1], marker="o", color=c, ms=4.5, mec="white", mew=0.6, ls="none")
ax_xy.plot([], [], marker="o", color="0.4", ms=4.5, ls="none", label="End")
ax_xy.annotate("", xy=(0.45 * SQUARE_SIDE, 0.06), xytext=(0.3 * SQUARE_SIDE, 0.06),
               arrowprops=dict(arrowstyle="->", color=C_REF, lw=0.9))
ax_xy.set_aspect("equal")
ax_xy.set_xlabel("$x$ [m]")
ax_xy.set_ylabel("$y$ [m]")
ax_xy.set_title("(a) Trajectory")
ax_xy.grid(True)
ax_xy.legend(loc="center", bbox_to_anchor=(0.5, 0.6), frameon=True, framealpha=0.95, edgecolor="0.8")
ax_xy.text(0.5 * SQUARE_SIDE, 0.22 * SQUARE_SIDE,
           "End position error\n"
           f"odometry: {pos_err['odom'] * 100:.1f} cm\n"
           f"gyrodometry: {pos_err['gyrodom'] * 100:.1f} cm",
           ha="center", va="center", fontsize=7.5, color="0.25")

# (b), (c) x and y vs time
for ax, col, label, title in ((ax_x, "x", "$x$ [m]", "(b)"), (ax_y, "y", "$y$ [m]", "(c)")):
    ax.plot(odom.t, odom[col], color=C_ODOM)
    ax.plot(gyrodom.t, gyrodom[col], color=C_GYRO)
    ax.set_ylabel(label)
    ax.grid(True)
    ax.text(0.01, 0.92, title, transform=ax.transAxes, va="top", fontsize=9)

# (d) heading vs time
ax_th.plot(odom.t, np.degrees(odom.theta), color=C_ODOM)
ax_th.plot(gyrodom.t, np.degrees(gyrodom.theta), color=C_GYRO)
ax_th.plot(gyrodom.t, np.degrees(gyro_theta), ":", color=C_GREF, lw=1.1, label="Integrated gyro (bias-corrected)")
ax_th.set_ylabel(r"$\theta$ [deg]")
ax_th.set_yticks(range(0, 361, 90))
ax_th.grid(True)
ax_th.text(0.01, 0.92, "(d)", transform=ax_th.transAxes, va="top", fontsize=9)
ax_th.legend(loc="upper left", bbox_to_anchor=(0.07, 1.0), frameon=False)

# (e) heading relative to the integrated gyro, to make the small differences visible
ax_err.plot(odom.t, np.degrees(odom.theta - gyro_theta_on_odom), color=C_ODOM)
ax_err.plot(gyrodom.t, np.degrees(gyrodom.theta - gyro_theta), color=C_GYRO)
ax_err.axhline(0, color=C_GREF, lw=1.1, ls=":")
ax_err.set_ylabel(r"$\theta - \theta_{\mathrm{gyro}}$ [deg]")
ax_err.set_xlabel("Time [s]")
ax_err.grid(True)
ax_err.text(0.01, 0.92, "(e)", transform=ax_err.transAxes, va="top", fontsize=9)

ax_x.set_xlim(0, max(odom.t.iloc[-1], gyrodom.t.iloc[-1]))
for ax in (ax_x, ax_y, ax_th):
    ax.tick_params(labelbottom=False)
ax_x.set_title("Pose vs. time")

for ext in ("png", "pdf"):
    fig.savefig(os.path.join(HERE, f"{OUT_NAME}.{ext}"), dpi=300)
print(f"Saved {OUT_NAME}.png and {OUT_NAME}.pdf in {HERE}")

# ---------------------------------------------------------------- table -----
print(f"\nGyro bias removed: {bias:.5f} rad/s")
print(f"{'Estimator':<28}{'End pos. error [cm]':>20}{'End heading error [deg]':>26}")
print(f"{'Odometry (wheels)':<28}{pos_err['odom'] * 100:>20.1f}{head_err['odom']:>+26.1f}")
print(f"{'Gyrodometry (wheels + IMU)':<28}{pos_err['gyrodom'] * 100:>20.1f}{head_err['gyrodom']:>+26.1f}")
print(f"{'Integrated gyro (reference)':<28}{'-':>20}{head_err['gyro']:>+26.1f}")

plt.show()
