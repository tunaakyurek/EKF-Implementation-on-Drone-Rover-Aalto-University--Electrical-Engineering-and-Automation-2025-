#!/usr/bin/env python3
import os
import sys
import glob
from pathlib import Path


def main():
    project_root = Path(__file__).resolve().parents[1]
    iph = project_root / 'iphone_integration'

    # 1) Keep only last two online 8-DOF ekf logs and matching raw logs
    data_dir = iph / 'data'
    keep_paths = set()
    ekf_files = sorted(
        [p for p in data_dir.glob('robomaster_ekf_log_*.csv') if '_offline' not in p.name and '_replay' not in p.name],
        key=lambda p: p.stat().st_mtime,
    )[-2:]
    for ekf in ekf_files:
        keep_paths.add(ekf)
        ts = ekf.stem.replace('robomaster_ekf_log_', '')
        raw = data_dir / f'robomaster_raw_log_{ts}.csv'
        if raw.exists():
            keep_paths.add(raw)
    for f in data_dir.glob('*.csv'):
        if f not in keep_paths:
            try:
                f.unlink()
            except Exception:
                pass

    # 2) Remove analysis/offline/docs/tests/demos and extra files
    dirs_to_remove = [
        iph / 'analysis_results',
        iph / 'offline_analysis',
        iph / 'docs',
        iph / 'dynamic_analysis_results',
        iph / 'robomaster_control',
        iph / 'trajectory_plots',
        iph / 'tests',
    ]
    for d in dirs_to_remove:
        if d.exists():
            for root, dirs, files in os.walk(d, topdown=False):
                for name in files:
                    try:
                        os.remove(os.path.join(root, name))
                    except Exception:
                        pass
                for name in dirs:
                    try:
                        os.rmdir(os.path.join(root, name))
                    except Exception:
                        pass
            try:
                os.rmdir(d)
            except Exception:
                pass

    files_to_remove = [
        iph / 'demo_realistic_yaw_fix.py',
        iph / 'demo_yaw_fix.py',
        iph / 'test_improved_ekf.py',
        iph / 'test_live_connection.py',
        iph / 'test_network_connection.py',
        iph / 'test_robomaster_ekf.py',
        iph / 'visualize_trajectory.py',
        iph / 'visualize_trajectory_simple.py',
        iph / 'README_YAW_FIX.md',
        iph / 'IMPLEMENTATION_SUMMARY.md',
    ]
    for f in files_to_remove:
        try:
            if f.exists():
                f.unlink()
        except Exception:
            pass

    # 3) Ensure we keep real-time EKF code in pi_phone_connection (8-DOF and 9-DOF)
    # No action needed unless cleaning subfiles; we simply avoid deleting that directory

    print('Cleanup complete.')


if __name__ == '__main__':
    sys.exit(main())


