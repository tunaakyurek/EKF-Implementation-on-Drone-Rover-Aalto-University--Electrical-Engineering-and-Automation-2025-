#!/usr/bin/env python3
"""
Offline EKF Replay from Raw Logs

Reads a raw RoboMaster CSV (accel, gyro, mag, optional GPS), runs the EKF with
configurable alignment and gating, and writes a new EKF CSV compatible with
the diagnostics script.
"""

import argparse
import os
import numpy as np
import pandas as pd

from src.ekf.ekf_core import ExtendedKalmanFilter, SensorData


def build_rotation_matrix(yaw_deg: float, pitch_deg: float, roll_deg: float) -> np.ndarray:
    ry = np.radians(yaw_deg)
    rp = np.radians(pitch_deg)
    rr = np.radians(roll_deg)
    cy, sy = np.cos(ry), np.sin(ry)
    cp, sp = np.cos(rp), np.sin(rp)
    cr, sr = np.cos(rr), np.sin(rr)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=float)
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=float)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=float)
    # Apply Z (yaw), then Y (pitch), then X (roll)
    return Rx @ Ry @ Rz


def replay(raw_csv: str, output_csv: str, yaw_mount_offset_deg: float, mag_disabled: bool, r_imu_to_body_euler: tuple):
    df = pd.read_csv(raw_csv)
    if 'time_rel' not in df.columns:
        if 'timestamp' in df.columns:
            df['time_rel'] = df['timestamp'] - df['timestamp'].iloc[0]
        else:
            raise ValueError('raw CSV must contain timestamp or time_rel')

    ekf = ExtendedKalmanFilter()
    # Configure alignment and gating
    ekf.R_imu_to_body = build_rotation_matrix(*r_imu_to_body_euler)
    ekf.yaw_mount_offset = np.radians(yaw_mount_offset_deg)
    ekf.mag_heading_gate_deg = 10.0 if mag_disabled else 25.0
    ekf.mag_norm_gate_rel = 0.2
    ekf.gyro_gate_rate = 0.5
    if mag_disabled:
        # Effectively disable by pushing noise very high
        ekf.R_mag = np.eye(3) * 1e6

    # Output buffers
    out_rows = []

    for i, row in df.iterrows():
        t = float(row['time_rel'])
        ax = row.get('accel_x'); ay = row.get('accel_y'); az = row.get('accel_z')
        gx = row.get('gyro_x'); gy = row.get('gyro_y'); gz = row.get('gyro_z')
        mx = row.get('mag_x'); my = row.get('mag_y'); mz = row.get('mag_z')

        accel = np.array([ax, ay, az], dtype=float) if pd.notnull(ax) and pd.notnull(ay) and pd.notnull(az) else None
        gyro = np.array([gx, gy, gz], dtype=float) if pd.notnull(gx) and pd.notnull(gy) and pd.notnull(gz) else None
        mag = np.array([mx, my, mz], dtype=float) if pd.notnull(mx) and pd.notnull(my) and pd.notnull(mz) else None

        sd = SensorData(
            timestamp=t,
            accel=accel,
            gyro=gyro,
            mag=mag,
        )
        ekf.process_sensor_data(sd)
        state = ekf.get_state()

        out_rows.append({
            'timestamp': t,
            'time_rel': t,
            'x': state.position[0],
            'y': state.position[1],
            'theta': state.orientation[2],
            'vx': state.velocity[0],
            'vy': state.velocity[1],
            'accel_x': accel[0] if accel is not None else np.nan,
            'accel_y': accel[1] if accel is not None else np.nan,
            'accel_z': accel[2] if accel is not None else np.nan,
            'gyro_x': gyro[0] if gyro is not None else np.nan,
            'gyro_y': gyro[1] if gyro is not None else np.nan,
            'gyro_z': gyro[2] if gyro is not None else np.nan,
        })

    out_df = pd.DataFrame(out_rows)
    os.makedirs(os.path.dirname(output_csv), exist_ok=True)
    out_df.to_csv(output_csv, index=False)
    return output_csv


def main():
    ap = argparse.ArgumentParser(description='Offline EKF replay using raw logs')
    ap.add_argument('--raw-file', required=True, help='Path to raw CSV (robomaster_raw_log_*.csv)')
    ap.add_argument('--output-file', required=False, help='Output EKF CSV path')
    ap.add_argument('--yaw-mount-offset-deg', type=float, default=0.0, help='Static mounting yaw offset in degrees')
    ap.add_argument('--disable-mag', action='store_true', help='Disable magnetometer updates (for isolation)')
    ap.add_argument('--r-imu-to-body', type=float, nargs=3, default=[0.0, 0.0, 0.0], metavar=('YAW_DEG','PITCH_DEG','ROLL_DEG'),
                    help='Euler angles (deg) Z-Y-X to rotate IMU frame into body frame')
    args = ap.parse_args()

    raw_base = os.path.splitext(os.path.basename(args.raw_file))[0]
    if args.output_file:
        out_path = args.output_file
    else:
        out_dir = os.path.join(os.path.dirname(args.raw_file))
        suffix = '_replay_mag_off' if args.disable_mag else '_replay'
        out_path = os.path.join(out_dir, raw_base.replace('robomaster_raw_log_', 'robomaster_ekf_log_') + suffix + '.csv')

    produced = replay(
        raw_csv=args.raw_file,
        output_csv=out_path,
        yaw_mount_offset_deg=args.yaw_mount_offset_deg,
        mag_disabled=args.disable_mag,
        r_imu_to_body_euler=tuple(args.r_imu_to_body),
    )
    print(f"Wrote EKF replay to: {produced}")


if __name__ == '__main__':
    main()


