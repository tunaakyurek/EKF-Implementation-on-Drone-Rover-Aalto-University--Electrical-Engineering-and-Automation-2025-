#!/usr/bin/env python3
"""
Offline replay for RoboMaster 8-DOF EKF (exact formulary) using raw CSV logs.

Reads a raw CSV (robomaster_raw_log_*.csv), feeds aligned IMU/GPS to the
RoboMasterEKF8DOF, and writes an EKF CSV suitable for diagnostics.
"""

import argparse
import os
import sys
import math
import numpy as np
import pandas as pd

# Ensure project root is on sys.path for absolute imports
_HERE = os.path.dirname(__file__)
_PROJECT_ROOT = os.path.abspath(os.path.join(_HERE, '..', '..', '..'))
if _PROJECT_ROOT not in sys.path:
    sys.path.append(_PROJECT_ROOT)

from iphone_integration.pi_phone_connection.ekf_robomaster_8dof import RoboMasterEKF8DOF, RoboMasterState


def build_online_alignment_matrix() -> np.ndarray:
    # Match online: R_imu_to_body = Rz(60°) @ RswapXY
    cz, sz = np.cos(np.radians(60.0)), np.sin(np.radians(60.0))
    Rz = np.array([[cz, -sz, 0.0], [sz, cz, 0.0], [0.0, 0.0, 1.0]], dtype=float)
    Rswap = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]], dtype=float)
    return Rz @ Rswap


def local_from_gps(lat0: float, lon0: float, lat: float, lon: float) -> tuple:
    lat_to_m = 111320.0
    lon_to_m = 111320.0 * np.cos(np.radians(lat0))
    x = (lon - lon0) * lon_to_m
    y = (lat - lat0) * lat_to_m
    return x, y


def replay_8dof(raw_csv: str, output_csv: str) -> str:
    df = pd.read_csv(raw_csv)
    if 'timestamp' not in df.columns:
        raise ValueError('raw CSV must contain timestamp column')
    t0 = float(df['timestamp'].iloc[0])
    time_rel = df['timestamp'].to_numpy() - t0

    # Match online EKF config
    ekf_config = {
        'q_accel': 0.1,
        'q_gyro': 0.005,
        'q_accel_bias': 1e-5,
        'q_gyro_bias': 1e-4,
        'r_accel': 0.05,
        'r_gyro': 0.005,
        'r_gps_pos': 0.5,
        'r_gps_vel': 0.5,  # GPS velocity fusion disabled below; keep conservative
        # Tuned measurement/constraint noises for improved precision
        'r_yaw': 0.1,
        'r_nhc': 0.02,
        'r_zupt': 0.003,
        'r_zaru': 0.0003,
        'init_pos_var': 0.5,
        'init_theta_var': 0.05,
        'init_vel_var': 0.2,
        'init_accel_bias_var': 0.005,
        'init_gyro_bias_var': 0.005,
    }
    ekf = RoboMasterEKF8DOF(ekf_config)

    # Alignment identical to online
    R = build_online_alignment_matrix()
    yaw_mount_offset_rad = np.radians(60.0)

    # GPS reference
    lat0 = None
    lon0 = None

    out_rows = []
    last_time = None

    for i in range(len(df)):
        t = float(time_rel[i])
        if last_time is None:
            dt = 0.02
        else:
            dt = max(1e-3, min(t - last_time, 0.1))
        last_time = t

        # Build aligned IMU control input
        ax = df.at[i, 'accel_x'] if 'accel_x' in df.columns else np.nan
        ay = df.at[i, 'accel_y'] if 'accel_y' in df.columns else np.nan
        az = df.at[i, 'accel_z'] if 'accel_z' in df.columns else np.nan
        gx = df.at[i, 'gyro_x'] if 'gyro_x' in df.columns else np.nan
        gy = df.at[i, 'gyro_y'] if 'gyro_y' in df.columns else np.nan
        gz = df.at[i, 'gyro_z'] if 'gyro_z' in df.columns else np.nan

        control = None
        if np.isfinite(ax) and np.isfinite(ay) and np.isfinite(az) and np.isfinite(gx) and np.isfinite(gy) and np.isfinite(gz):
            imu = np.array([ax, ay, az], dtype=float)
            gyro = np.array([gx, gy, gz], dtype=float)
            imu_aligned = R @ imu
            gyro_aligned = R @ gyro
            control = np.array([imu_aligned[0], imu_aligned[1], gyro_aligned[2]], dtype=float)

        ekf.predict(dt, control)

        # Magnetometer yaw update with simple gating
        if {'mag_x','mag_y'}.issubset(df.columns):
            mx = df.at[i, 'mag_x']
            my = df.at[i, 'mag_y']
            mz = df.at[i, 'mag_z'] if 'mag_z' in df.columns else 0.0
            if np.isfinite(mx) and np.isfinite(my):
                mag_body = R @ np.array([mx, my, mz], dtype=float)
                # Gate during high rotation or high acceleration
                use_mag = True
                if control is not None and (abs(control[2]) > 0.5 or abs(np.linalg.norm(imu_aligned) - 9.81) > 0.25 * 9.81):
                    use_mag = False
                if use_mag:
                    yaw_mag = math.atan2(mag_body[1], mag_body[0]) - yaw_mount_offset_rad
                    # Normalize to [-pi,pi]
                    yaw_mag = (yaw_mag + math.pi) % (2 * math.pi) - math.pi
                    ekf.update_yaw(yaw_mag)

        # GPS position update (velocity/course disabled to match online defaults)
        has_gps = {'gps_lat','gps_lon'}.issubset(df.columns)
        if has_gps:
            lat = df.at[i, 'gps_lat']
            lon = df.at[i, 'gps_lon']
            if np.isfinite(lat) and np.isfinite(lon):
                if lat0 is None:
                    lat0 = float(lat)
                    lon0 = float(lon)
                x_gps, y_gps = local_from_gps(lat0, lon0, lat, lon)
                ekf.update_gps_position(np.array([x_gps, y_gps], dtype=float))

        # Apply constraints (NHC, ZUPT, ZARU, GPS-course)
        ekf.apply_constraint_updates()

        st = ekf.get_state()
        out_rows.append({
            'timestamp': float(df.at[i, 'timestamp']),
            'time_rel': t,
            'x': st.x,
            'y': st.y,
            'theta': st.theta,
            'vx': st.vx,
            'vy': st.vy,
            'bias_accel_x': st.bias_accel_x,
            'bias_accel_y': st.bias_accel_y,
            'bias_angular_velocity': st.bias_angular_velocity,
            'accel_x': ax,
            'accel_y': ay,
            'accel_z': az,
            'gyro_x': gx,
            'gyro_y': gy,
            'gyro_z': gz,
        })

    out_df = pd.DataFrame(out_rows)
    os.makedirs(os.path.dirname(output_csv), exist_ok=True)
    out_df.to_csv(output_csv, index=False)
    return output_csv


def main():
    ap = argparse.ArgumentParser(description='Offline 8-DOF EKF replay using raw logs (matches online pipeline)')
    ap.add_argument('--raw-file', required=True, help='Path to raw CSV (robomaster_raw_log_*.csv)')
    ap.add_argument('--output-file', required=True, help='Output EKF CSV path')
    args = ap.parse_args()

    produced = replay_8dof(args.raw_file, args.output_file)
    print(f"Wrote 8DOF EKF replay to: {produced}")


if __name__ == '__main__':
    main()


