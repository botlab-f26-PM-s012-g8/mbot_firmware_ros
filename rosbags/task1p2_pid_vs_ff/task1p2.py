"""Task 1.2: step response of FF vs FF+PID wheel controllers to a 0.5 m/s cmd_vel.

Robot velocity comes from csv_output/odom.csv. Time is relative to when the
robot starts moving; the 0.5 m/s command is drawn from motion start until the
robot starts decelerating (cmd_vel.csv timestamps don't line up with odom).
"""
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd

HERE = Path(__file__).resolve().parent
RUNS = {"Feedforward": "task1p2_ff_graph", "Feedforward + PID": "task1p2_ff_pid_graph"}
CMD_VX = 0.5  # m/s
MOVING_THRESH = 0.01  # m/s, odom speed that counts as "moving"
CMD_COLOR, ACTUAL_COLOR = "black", "#2a6fdb"

plt.rcParams.update({
    "font.family": "serif",
    "mathtext.fontset": "dejavuserif",
    "font.size": 11,
    "axes.titlesize": 12,
    "axes.labelsize": 11,
    "legend.fontsize": 10,
    "xtick.labelsize": 10,
    "ytick.labelsize": 10,
    "axes.linewidth": 0.8,
    "savefig.bbox": "tight",
})


def read_odom(csv_path):
    odom = pd.read_csv(csv_path)
    odom["t"] = odom["header.stamp.sec"] + odom["header.stamp.nanosec"] * 1e-9
    return odom.sort_values("t").reset_index(drop=True)


def main():
    fig, axes = plt.subplots(2, len(RUNS), figsize=(7.5, 5.5), sharex=True, sharey="row")

    for col, (label, folder) in enumerate(RUNS.items()):
        odom = read_odom(HERE / folder / "csv_output" / "odom.csv")
        vx = odom["twist.twist.linear.x"]

        # Motion starts at the first moving sample; command ends where the
        # robot starts decelerating (last sample before vx drops below 90% of cmd)
        start = odom.index[vx.abs() > MOVING_THRESH][0]
        t0 = odom.at[start, "t"]
        stop = odom.index[(odom.index > start) & (vx >= 0.9 * CMD_VX)][-1]
        t_cmd_end = odom.at[stop, "t"] - t0
        odom["dt"] = odom["t"] - t0
        odom = odom[(odom["dt"] >= -0.5) & (odom["dt"] <= t_cmd_end + 1.5)]

        cmd_t = [odom["dt"].iloc[0], 0.0, t_cmd_end, odom["dt"].iloc[-1]]
        for row, (odom_key, cmd_vals, ylabel) in enumerate(
            [("twist.twist.linear.x", [0, CMD_VX, 0, 0], r"$v_x$ [m/s]"),
             ("twist.twist.angular.z", [0, 0, 0, 0], r"$\omega_z$ [rad/s]")]
        ):
            ax = axes[row, col]
            ax.step(cmd_t, cmd_vals, where="post", color=CMD_COLOR, linestyle="--",
                    linewidth=1.2, label="Commanded")
            ax.plot(odom["dt"], odom[odom_key], color=ACTUAL_COLOR, linewidth=1.5,
                    label="Measured (odometry)")
            ax.grid(True, color="#e0e0e0", linewidth=0.6)
            ax.set_axisbelow(True)
            ax.text(0.02, 0.95, f"({'abcd'[row * len(RUNS) + col]})", transform=ax.transAxes,
                    va="top", fontweight="bold")
            ax.spines[["top", "right"]].set_visible(False)
            if col == 0:
                ax.set_ylabel(ylabel)
            if row == 0:
                ax.set_title(label)
            else:
                ax.set_xlabel("Time since motion onset [s]")

    axes[0, 0].set_ylim(-0.03, 0.65)
    axes[1, 0].set_ylim(-0.4, 0.15)
    axes[0, 0].set_xlim(-0.5, None)

    handles, labels = axes[0, 0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", ncol=2, frameon=False,
               bbox_to_anchor=(0.5, 1.04))
    fig.tight_layout()
    for ext in ("pdf", "png"):
        out = HERE / f"task1p2_step_response.{ext}"
        fig.savefig(out, dpi=300)
        print(f"saved {out}")
    plt.show()


if __name__ == "__main__":
    main()
