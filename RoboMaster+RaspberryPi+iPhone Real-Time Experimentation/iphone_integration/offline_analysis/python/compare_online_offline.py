#!/usr/bin/env python3
"""
Compare latest online EKF output with offline EKF replay on the same raw data.

Steps:
- Find the latest robomaster_raw_log_*.csv in iphone_integration/data
- Locate the matching robomaster_ekf_log_*.csv (online)
- Run offline 8-DOF EKF replay on the raw log with RoboMaster alignment params
- Run diagnostics on both (online/offline) against the raw log
- Save a compact comparison JSON and figures under iphone_integration/analysis_results
"""

import os
import sys
import json
import glob
import argparse
from datetime import datetime

# Make local directory importable when run as a script
_HERE = os.path.dirname(__file__)
if _HERE not in sys.path:
    sys.path.append(_HERE)
# Ensure project root is on path so 'iphone_integration' absolute imports work
_PROJECT_ROOT = os.path.abspath(os.path.join(_HERE, '..', '..', '..'))
if _PROJECT_ROOT not in sys.path:
    sys.path.append(_PROJECT_ROOT)

# Local imports
from offline_replay_ekf_8dof import replay_8dof
from offline_replay_ekf import replay as replay_12dof
from diagnostics_metrics import run_diagnostics


def find_latest_logs(data_dir: str) -> tuple[str, str]:
    raw_files = sorted(glob.glob(os.path.join(data_dir, 'robomaster_raw_log_*.csv')))
    if not raw_files:
        raise FileNotFoundError(f"No raw logs found in {data_dir}")
    latest_raw = raw_files[-1]

    # Match ekf log by timestamp suffix
    base = os.path.basename(latest_raw).replace('robomaster_raw_log_', '').replace('.csv', '')
    online_ekf = os.path.join(data_dir, f"robomaster_ekf_log_{base}.csv")
    if not os.path.exists(online_ekf):
        raise FileNotFoundError(f"Online EKF log not found for raw base {base} at {online_ekf}")
    return latest_raw, online_ekf


def main():
    ap = argparse.ArgumentParser(description='Compare online EKF vs offline EKF replay diagnostics')
    ap.add_argument('--data-dir', default=os.path.join('iphone_integration', 'data'), help='Directory containing logs')
    ap.add_argument('--results-dir', default=os.path.join('iphone_integration', 'analysis_results'), help='Output directory for results')
    ap.add_argument('--yaw-mount-offset-deg', type=float, default=60.0, help='Mounting yaw offset used online')
    ap.add_argument('--r-imu-to-body', type=float, nargs=3, default=[60.0, 0.0, 0.0], metavar=('YAW_DEG','PITCH_DEG','ROLL_DEG'),
                    help='Euler angles (deg) for IMU->body used for replay alignment')
    args = ap.parse_args()

    os.makedirs(args.results_dir, exist_ok=True)

    # 1) Find latest logs
    raw_file, online_ekf_file = find_latest_logs(args.data_dir)

    # 2) Produce offline EKF replays (8-DOF and 12-DOF)
    base = os.path.splitext(os.path.basename(raw_file))[0]
    offline_out = os.path.join(args.data_dir, base.replace('robomaster_raw_log_', 'robomaster_ekf_log_') + '_offline8dof.csv')
    offline_ekf_file = replay_8dof(
        raw_csv=raw_file,
        output_csv=offline_out,
        r_imu_to_body_euler=tuple(args.r_imu_to_body),
        yaw_mount_offset_deg=args.yaw_mount_offset_deg,
    )
    offline12_out = os.path.join(args.data_dir, base.replace('robomaster_raw_log_', 'robomaster_ekf_log_') + '_offline12dof.csv')
    offline_ekf_12_file = replay_12dof(
        raw_csv=raw_file,
        output_csv=offline12_out,
        yaw_mount_offset_deg=args.yaw_mount_offset_deg,
        mag_disabled=False,
        r_imu_to_body_euler=tuple(args.r_imu_to_body),
    )

    # 3) Diagnostics for both
    # Use pre-delay and calibration durations consistent with online defaults (5s each)
    pre_delay = 5.0
    calib_dur = 5.0

    tag_time = datetime.now().strftime('%Y%m%d_%H%M%S')
    online_report = run_diagnostics(
        ekf_file=online_ekf_file,
        raw_file=raw_file,
        output_dir=args.results_dir,
        pre_delay=pre_delay,
        calib_dur=calib_dur,
    )
    offline_report = run_diagnostics(
        ekf_file=offline_ekf_file,
        raw_file=raw_file,
        output_dir=args.results_dir,
        pre_delay=pre_delay,
        calib_dur=calib_dur,
    )
    offline12_report = run_diagnostics(
        ekf_file=offline_ekf_12_file,
        raw_file=raw_file,
        output_dir=args.results_dir,
        pre_delay=pre_delay,
        calib_dur=calib_dur,
    )

    # 4) Compact comparison
    comparison = {
        'raw_file': raw_file,
        'online_ekf_file': online_ekf_file,
        'offline_ekf_file': offline_ekf_file,
        'pre_delay_s': pre_delay,
        'calibration_duration_s': calib_dur,
        'online_metrics': online_report,
        'offline8dof_metrics': offline_report,
        'offline12dof_metrics': offline12_report,
    }

    report_path = os.path.join(args.results_dir, f'comparison_report_{tag_time}.json')
    with open(report_path, 'w') as f:
        json.dump(comparison, f, indent=2)

    print(f"Saved comparison report to: {report_path}")
    print(f"Online EKF: {online_ekf_file}")
    print(f"Offline EKF: {offline_ekf_file}")


if __name__ == '__main__':
    main()


