#!/usr/bin/env python3
"""
Create a static figure similar to the online real-time plot for an EKF CSV.

Inputs: EKF CSV produced by online or offline 8-DOF (columns: time_rel/timestamp, x,y,theta,vx,vy,accel_*,gyro_*)
Output: PNG with 2D trajectory, position/orientation/velocity vs time, IMU plots.
"""

import os
import sys
import argparse
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


def plot_estimation(csv_file: str, output_png: str, pre_delay: float = 5.0, calibration_duration: float = 5.0) -> str:
    df = pd.read_csv(csv_file)

    # Time base
    if 'time_rel' in df.columns:
        t = df['time_rel'].to_numpy()
    elif 'timestamp' in df.columns:
        t0 = float(df['timestamp'].iloc[0])
        t = df['timestamp'].to_numpy() - t0
    else:
        raise ValueError('CSV must contain time_rel or timestamp')

    # Mask out pre-delay + calibration
    start_time = pre_delay + calibration_duration
    mask = t >= start_time
    t = t[mask]

    x = df['x'].to_numpy()[mask] if 'x' in df.columns else np.zeros(len(t))
    y = df['y'].to_numpy()[mask] if 'y' in df.columns else np.zeros(len(t))
    theta = df['theta'].to_numpy()[mask] if 'theta' in df.columns else np.zeros(len(t))
    vx = df['vx'].to_numpy()[mask] if 'vx' in df.columns else np.zeros(len(t))
    vy = df['vy'].to_numpy()[mask] if 'vy' in df.columns else np.zeros(len(t))
    ax = df['accel_x'].to_numpy()[mask] if 'accel_x' in df.columns else None
    ay = df['accel_y'].to_numpy()[mask] if 'accel_y' in df.columns else None
    az = df['accel_z'].to_numpy()[mask] if 'accel_z' in df.columns else None
    gz = df['gyro_z'].to_numpy()[mask] if 'gyro_z' in df.columns else None

    plt.style.use('dark_background')
    fig, ((ax1, ax2), (ax3, ax4), (ax5, ax6)) = plt.subplots(3, 2, figsize=(15, 12))

    # 2D Trajectory
    ax1.set_title('2D Trajectory (XY)')
    ax1.plot(x, y, 'g-', linewidth=2, label='EKF')
    if len(x) > 0:
        ax1.scatter([x[0]], [y[0]], c='blue', s=60, label='Start')
        ax1.scatter([x[-1]], [y[-1]], c='yellow', s=60, label='End')
    ax1.set_xlabel('X (m)'); ax1.set_ylabel('Y (m)'); ax1.grid(True, alpha=0.3); ax1.legend(); ax1.set_aspect('equal')

    # Position vs time (Z unavailable in 8-DOF)
    ax2.set_title('Position vs Time')
    ax2.plot(t, x, 'r-', label='x')
    ax2.plot(t, y, 'g-', label='y')
    ax2.set_xlabel('Time (s)'); ax2.set_ylabel('Position (m)'); ax2.grid(True, alpha=0.3); ax2.legend()

    # Orientation vs time (theta)
    ax3.set_title('Orientation vs Time')
    ax3.plot(t, theta, 'b-', label='yaw (theta)')
    ax3.set_xlabel('Time (s)'); ax3.set_ylabel('Angle (rad)'); ax3.grid(True, alpha=0.3); ax3.legend()

    # Velocity vs time (vx, vy)
    ax4.set_title('Velocity vs Time')
    ax4.plot(t, vx, 'r-', label='vx')
    ax4.plot(t, vy, 'g-', label='vy')
    ax4.set_xlabel('Time (s)'); ax4.set_ylabel('Velocity (m/s)'); ax4.grid(True, alpha=0.3); ax4.legend()

    # IMU Acceleration
    ax5.set_title('IMU Acceleration')
    if ax is not None and ay is not None and az is not None:
        ax5.plot(t, ax, 'r-', label='ax')
        ax5.plot(t, ay, 'g-', label='ay')
        ax5.plot(t, az, 'b-', label='az')
        ax5.legend()
    ax5.set_xlabel('Time (s)'); ax5.set_ylabel('Accel (m/s^2)'); ax5.grid(True, alpha=0.3)

    # Gyro z (proxy for covariance panel)
    ax6.set_title('Gyro z (rad/s)')
    if gz is not None:
        ax6.plot(t, gz, color='orange')
    ax6.set_xlabel('Time (s)'); ax6.set_ylabel('wz (rad/s)'); ax6.grid(True, alpha=0.3)

    plt.tight_layout()
    out_dir = os.path.dirname(output_png)
    if out_dir and not os.path.exists(out_dir):
        os.makedirs(out_dir, exist_ok=True)
    fig.savefig(output_png, dpi=300, bbox_inches='tight', facecolor='black')
    plt.close(fig)
    return output_png


def main():
    ap = argparse.ArgumentParser(description='Plot offline EKF estimation figure similar to online plotter')
    ap.add_argument('--ekf-file', required=True, help='Path to EKF CSV')
    ap.add_argument('--output', required=False, help='Output PNG path')
    ap.add_argument('--pre-delay', type=float, default=5.0, help='Pre-delay seconds to exclude')
    ap.add_argument('--calibration-duration', type=float, default=5.0, help='Calibration seconds to exclude')
    args = ap.parse_args()

    if args.output:
        out = args.output
    else:
        base = os.path.splitext(os.path.basename(args.ekf_file))[0]
        out_dir = os.path.join(os.path.dirname(args.ekf_file), '..', 'analysis_results')
        out = os.path.join(out_dir, f'estimation_{base}.png')

    produced = plot_estimation(args.ekf_file, out, pre_delay=args.pre_delay, calibration_duration=args.calibration_duration)
    print(f'Wrote: {produced}')


if __name__ == '__main__':
    sys.exit(main())


