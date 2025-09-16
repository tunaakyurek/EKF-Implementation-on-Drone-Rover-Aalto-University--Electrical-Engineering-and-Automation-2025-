#!/usr/bin/env python3
"""
Diagnostics and Metrics for RoboMaster EKF and Raw Sensor Logs

Computes and saves:
- Correlation between gyro_z and d(theta)/dt
- Corner integrals (integrated gyro_z) vs EKF theta change
- Straight-leg alignment: heading(theta) vs arctan2(vy, vx)
- Magnetometer health: norm stability, heading residual, gating percentage
- Stationary bias during pre-delay + calibration

Outputs figures to the specified output directory and prints a concise metrics report.
"""

import argparse
import os
import math
from typing import Dict, List, Tuple

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


def wrap_angle(angle_rad: np.ndarray) -> np.ndarray:
    return (angle_rad + np.pi) % (2 * np.pi) - np.pi


def compute_time_derivative(values: np.ndarray, times: np.ndarray) -> np.ndarray:
    dv = np.diff(values)
    dt = np.diff(times)
    dt[dt == 0] = np.nan
    deriv = dv / dt
    # Pad to original length
    deriv = np.concatenate([deriv[:1], deriv])
    return deriv


def detect_phases(df: pd.DataFrame, pre_delay: float, calib_dur: float) -> Tuple[pd.DataFrame, Dict[str, float]]:
    df = df.copy()
    if 'time_rel' not in df.columns:
        if 'timestamp' in df.columns:
            df['time_rel'] = df['timestamp'] - df['timestamp'].iloc[0]
        else:
            raise ValueError('Required time column missing: time_rel or timestamp')

    # Filter: keep from calibration start onward
    start_time = pre_delay
    df = df[df['time_rel'] >= start_time].copy()
    df['time_rel'] = df['time_rel'] - start_time
    phase_info = {
        'calibration_start': 0.0,
        'calibration_end': calib_dur,
        'pre_delay': pre_delay
    }
    return df, phase_info


def correlate_gyro_dtheta(gyro_z: np.ndarray, theta: np.ndarray, t: np.ndarray) -> Tuple[float, float]:
    dtheta_dt = compute_time_derivative(theta, t)
    # Align lengths if needed
    m = np.isfinite(gyro_z) & np.isfinite(dtheta_dt)
    if m.sum() < 3:
        return float('nan'), float('nan')
    corr = np.corrcoef(gyro_z[m], dtheta_dt[m])[0, 1]
    sign = np.sign(corr)
    return float(corr), float(sign)


def detect_corners_by_gyro(t: np.ndarray, gyro_z: np.ndarray, rate_thresh: float = 0.4, min_duration: float = 0.2) -> List[Tuple[int, int]]:
    # Identify contiguous regions where |gyro_z| > threshold
    mask = np.abs(gyro_z) > rate_thresh
    idx = np.where(mask)[0]
    if idx.size == 0:
        return []
    segments = []
    start = idx[0]
    prev = idx[0]
    for i in idx[1:]:
        if i == prev + 1:
            prev = i
            continue
        segments.append((start, prev))
        start = i
        prev = i
    segments.append((start, prev))
    # Filter by duration
    filtered = []
    for s, e in segments:
        dur = t[e] - t[s]
        if dur >= min_duration:
            filtered.append((s, e))
    return filtered


def integrate_gyro_over_segments(t: np.ndarray, gyro_z: np.ndarray, segments: List[Tuple[int, int]]) -> List[float]:
    integrals = []
    for s, e in segments:
        dt = np.diff(t[s:e + 1])
        gz = gyro_z[s:e]
        integrals.append(float(np.nansum(gz * dt)))
    return integrals


def theta_change_over_segments(theta: np.ndarray, segments: List[Tuple[int, int]], pre_pad: int = 3, post_pad: int = 3) -> List[float]:
    changes = []
    n = len(theta)
    for s, e in segments:
        s0 = max(0, s - pre_pad)
        e1 = min(n - 1, e + post_pad)
        d = wrap_angle(theta[e1] - theta[s0])
        changes.append(float(d))
    return changes


def straight_segments_between(t: np.ndarray, corner_segments: List[Tuple[int, int]], min_length: float = 0.5) -> List[Tuple[int, int]]:
    if not corner_segments:
        return [(0, len(t) - 1)]
    segs = []
    last_end = 0
    for s, e in corner_segments:
        if t[s] - t[last_end] >= min_length:
            segs.append((last_end, s))
        last_end = e + 1
    if t[-1] - t[last_end] >= min_length:
        segs.append((last_end, len(t) - 1))
    return segs


def straight_alignment_metrics(theta: np.ndarray, vx: np.ndarray, vy: np.ndarray, t: np.ndarray, straights: List[Tuple[int, int]], speed_thresh: float = 0.05) -> List[Dict[str, float]]:
    results = []
    for s, e in straights:
        vmag = np.sqrt(vx[s:e + 1] ** 2 + vy[s:e + 1] ** 2)
        sel = (vmag > speed_thresh)
        if sel.sum() < 3:
            results.append({'t_start': float(t[s]), 't_end': float(t[e]), 'mean_abs_deg': float('nan'), 'median_abs_deg': float('nan')})
            continue
        hdg_v = np.arctan2(vy[s:e + 1][sel], vx[s:e + 1][sel])
        hdg_err = wrap_angle(theta[s:e + 1][sel] - hdg_v)
        deg_err = np.degrees(np.abs(hdg_err))
        results.append({
            't_start': float(t[s]),
            't_end': float(t[e]),
            'mean_abs_deg': float(np.nanmean(deg_err)),
            'median_abs_deg': float(np.nanmedian(deg_err))
        })
    return results


def magnetometer_health(raw_df: pd.DataFrame, theta: np.ndarray, t: np.ndarray, heading_gate_deg: float = 30.0, norm_rel_gate: float = 0.2) -> Dict[str, float]:
    result = {
        'available': 0,
        'mag_norm_median': float('nan'),
        'mag_norm_mad': float('nan'),
        'heading_residual_mean_deg': float('nan'),
        'heading_residual_mad_deg': float('nan'),
        'pct_heading_gated': float('nan'),
        'pct_norm_gated': float('nan')
    }
    if raw_df is None:
        return result
    cols = set(raw_df.columns)
    if not {'mag_x', 'mag_y'}.issubset(cols):
        return result

    # Align by time: assume both have relative time starting from same origin
    if 'time_rel' not in raw_df.columns:
        if 'timestamp' in raw_df.columns:
            raw_df = raw_df.copy()
            raw_df['time_rel'] = raw_df['timestamp'] - raw_df['timestamp'].iloc[0]
        else:
            return result

    # Interpolate raw mag to EKF time grid
    mx = np.interp(t, raw_df['time_rel'].to_numpy(), raw_df['mag_x'].to_numpy())
    my = np.interp(t, raw_df['time_rel'].to_numpy(), raw_df['mag_y'].to_numpy())
    mz = None
    if 'mag_z' in cols:
        mz = np.interp(t, raw_df['time_rel'].to_numpy(), raw_df['mag_z'].to_numpy())

    norm = np.sqrt(mx ** 2 + my ** 2 + (mz ** 2 if mz is not None else 0.0))
    median_norm = np.nanmedian(norm)
    mad_norm = np.nanmedian(np.abs(norm - median_norm))

    # Heading from mag in sensor frame; note: sign/axis may differ, this is a health proxy
    mag_heading = np.arctan2(my, mx)
    heading_residual = wrap_angle(mag_heading - theta)
    heading_residual_deg = np.degrees(heading_residual)

    gate_hdg = np.abs(heading_residual_deg) > heading_gate_deg
    gate_norm = np.abs(norm - median_norm) > (norm_rel_gate * median_norm)

    result.update({
        'available': 1,
        'mag_norm_median': float(median_norm),
        'mag_norm_mad': float(mad_norm),
        'heading_residual_mean_deg': float(np.nanmean(heading_residual_deg)),
        'heading_residual_mad_deg': float(np.nanmedian(np.abs(heading_residual_deg))),
        'pct_heading_gated': float(100.0 * np.mean(gate_hdg)),
        'pct_norm_gated': float(100.0 * np.mean(gate_norm))
    })
    return result


def plot_figures(output_dir: str, tag: str, t: np.ndarray, df_ekf: pd.DataFrame, raw_df: pd.DataFrame, corner_segments: List[Tuple[int, int]], straights: List[Tuple[int, int]]):
    os.makedirs(output_dir, exist_ok=True)

    # Plot gyro_z vs dtheta/dt
    theta = df_ekf['theta'].to_numpy()
    gz = df_ekf['gyro_z'].to_numpy() if 'gyro_z' in df_ekf.columns else np.full_like(theta, np.nan)
    dtheta_dt = compute_time_derivative(theta, t)

    plt.figure(figsize=(10, 6))
    plt.plot(t, gz, label='gyro_z [rad/s]')
    plt.plot(t, dtheta_dt, label='d(theta)/dt [rad/s]')
    for s, e in corner_segments:
        plt.axvspan(t[s], t[e], color='orange', alpha=0.2)
    plt.xlabel('time [s]')
    plt.legend()
    plt.title(f'Gyro vs Theta Rate - {tag}')
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, f'gyro_vs_theta_rate_{tag}.png'))
    plt.close()

    # Plot magnetometer norm and residual if available
    if raw_df is not None and {'mag_x', 'mag_y'}.issubset(raw_df.columns):
        mx = np.interp(t, raw_df['time_rel'].to_numpy(), raw_df['mag_x'].to_numpy())
        my = np.interp(t, raw_df['time_rel'].to_numpy(), raw_df['mag_y'].to_numpy())
        mz = np.interp(t, raw_df['time_rel'].to_numpy(), raw_df['mag_z'].to_numpy()) if 'mag_z' in raw_df.columns else None
        norm = np.sqrt(mx ** 2 + my ** 2 + (mz ** 2 if mz is not None else 0.0))
        mag_heading = np.arctan2(my, mx)
        heading_residual = wrap_angle(mag_heading - df_ekf['theta'].to_numpy())
        plt.figure(figsize=(10, 6))
        ax1 = plt.gca()
        ax1.plot(t, norm, 'b-', label='|mag|')
        ax1.set_ylabel('|mag| [a.u.]', color='b')
        ax2 = ax1.twinx()
        ax2.plot(t, np.degrees(heading_residual), 'r-', label='mag heading residual [deg]')
        ax2.set_ylabel('heading residual [deg]', color='r')
        for s, e in corner_segments:
            ax1.axvspan(t[s], t[e], color='orange', alpha=0.2)
        plt.xlabel('time [s]')
        plt.title(f'Magnetometer Health - {tag}')
        plt.tight_layout()
        plt.savefig(os.path.join(output_dir, f'mag_health_{tag}.png'))
        plt.close()

    # Plot trajectory with straight/corner overlays (fallback: integrate velocity if x,y absent)
    traj_x = None
    traj_y = None
    if {'x', 'y'}.issubset(df_ekf.columns):
        x_vals = df_ekf['x'].to_numpy()
        y_vals = df_ekf['y'].to_numpy()
        if np.allclose(np.nanstd(x_vals), 0.0) and np.allclose(np.nanstd(y_vals), 0.0):
            traj_x = None  # force fallback
        else:
            traj_x, traj_y = x_vals, y_vals
    if traj_x is None and {'vx','vy'}.issubset(df_ekf.columns):
        vx = df_ekf['vx'].to_numpy()
        vy = df_ekf['vy'].to_numpy()
        dt = np.diff(t, prepend=t[0])
        traj_x = np.cumsum(vx * dt)
        traj_y = np.cumsum(vy * dt)
    if traj_x is not None and traj_y is not None:
        plt.figure(figsize=(6, 6))
        plt.plot(traj_x, traj_y, 'k-', label='trajectory')
        for s, e in corner_segments:
            plt.plot(traj_x[s:e + 1], traj_y[s:e + 1], 'r-', linewidth=2)
        for s, e in straights:
            plt.plot(traj_x[s:e + 1], traj_y[s:e + 1], 'g-', linewidth=2)
        plt.axis('equal')
        plt.legend()
        plt.title(f'Trajectory Segments - {tag}')
        plt.tight_layout()
        plt.savefig(os.path.join(output_dir, f'trajectory_segments_{tag}.png'))
        plt.close()


def run_diagnostics(ekf_file: str, raw_file: str, output_dir: str, pre_delay: float, calib_dur: float) -> Dict[str, any]:
    df = pd.read_csv(ekf_file)
    df, phase = detect_phases(df, pre_delay, calib_dur)

    raw_df = None
    if raw_file and os.path.exists(raw_file):
        raw_df = pd.read_csv(raw_file)
        if 'time_rel' not in raw_df.columns:
            if 'timestamp' in raw_df.columns:
                raw_df['time_rel'] = raw_df['timestamp'] - raw_df['timestamp'].iloc[0]

    t = df['time_rel'].to_numpy()
    theta = df['theta'].to_numpy() if 'theta' in df.columns else None
    gyro_z = df['gyro_z'].to_numpy() if 'gyro_z' in df.columns else None
    vx = df['vx'].to_numpy() if 'vx' in df.columns else None
    vy = df['vy'].to_numpy() if 'vy' in df.columns else None

    report: Dict[str, any] = {}

    # 1) Correlation gyro_z and dtheta/dt
    corr = float('nan')
    sign = float('nan')
    if gyro_z is not None and theta is not None:
        corr, sign = correlate_gyro_dtheta(gyro_z, theta, t)
    report['gyro_theta_rate_correlation'] = corr
    report['gyro_theta_rate_sign'] = sign

    # 2) Corners: integrate gyro over detected segments, compare to theta change
    corner_segments = []
    gyro_integrals = []
    theta_changes = []
    corner_errors = []
    if gyro_z is not None:
        corner_segments = detect_corners_by_gyro(t, gyro_z)
        gyro_integrals = integrate_gyro_over_segments(t, gyro_z, corner_segments)
        if theta is not None:
            theta_changes = theta_change_over_segments(theta, corner_segments)
            corner_errors = [float(wrap_angle(tc - gi)) for gi, tc in zip(gyro_integrals, theta_changes)]
    report['corners_count'] = len(corner_segments)
    report['corner_integrals_rad'] = gyro_integrals
    report['corner_theta_changes_rad'] = theta_changes
    report['corner_errors_rad'] = corner_errors
    if corner_errors:
        report['corner_mean_abs_error_deg'] = float(np.degrees(np.nanmean(np.abs(corner_errors))))

    # 3) Straight-leg alignment
    straight_segs = straight_segments_between(t, corner_segments)
    straight_metrics = []
    if theta is not None and vx is not None and vy is not None:
        straight_metrics = straight_alignment_metrics(theta, vx, vy, t, straight_segs)
    report['straight_segments'] = straight_metrics

    # 4) Magnetometer health
    mag_health = magnetometer_health(raw_df, theta if theta is not None else np.full_like(t, np.nan), t)
    report['mag_health'] = mag_health

    # 5) Stationary bias using first calib_dur seconds of filtered data
    bias_window = df['time_rel'] <= calib_dur
    accel_x = df['accel_x'].to_numpy()[bias_window] if 'accel_x' in df.columns else None
    accel_y = df['accel_y'].to_numpy()[bias_window] if 'accel_y' in df.columns else None
    gyro_z_cal = df['gyro_z'].to_numpy()[bias_window] if 'gyro_z' in df.columns else None
    report['stationary_bias'] = {
        'accel_x_mean': float(np.nanmean(accel_x)) if accel_x is not None and accel_x.size else float('nan'),
        'accel_x_std': float(np.nanstd(accel_x)) if accel_x is not None and accel_x.size else float('nan'),
        'accel_y_mean': float(np.nanmean(accel_y)) if accel_y is not None and accel_y.size else float('nan'),
        'accel_y_std': float(np.nanstd(accel_y)) if accel_y is not None and accel_y.size else float('nan'),
        'gyro_z_mean': float(np.nanmean(gyro_z_cal)) if gyro_z_cal is not None and gyro_z_cal.size else float('nan'),
        'gyro_z_std': float(np.nanstd(gyro_z_cal)) if gyro_z_cal is not None and gyro_z_cal.size else float('nan'),
    }

    # Figures
    tag = os.path.splitext(os.path.basename(ekf_file))[0]
    plot_figures(output_dir, tag, t, df, raw_df, corner_segments, straight_segs)

    return report


def main():
    parser = argparse.ArgumentParser(description='Run diagnostics on RoboMaster EKF and raw logs')
    parser.add_argument('--file', required=True, help='Path to EKF CSV log file')
    parser.add_argument('--raw-file', default=None, help='Path to raw sensor CSV log file')
    parser.add_argument('--output', default='iphone_integration/analysis_results', help='Output directory for figures')
    parser.add_argument('--pre-delay', type=float, default=5.0, help='Pre-delay duration in seconds')
    parser.add_argument('--calibration-duration', type=float, default=5.0, help='Calibration duration in seconds')

    args = parser.parse_args()

    os.makedirs(args.output, exist_ok=True)
    report = run_diagnostics(args.file, args.raw_file, args.output, args.pre_delay, args.calibration_duration)

    # Print concise report
    print('\nDiagnostics Report:')
    print(f"  File: {args.file}")
    print(f"  Raw:  {args.raw_file}")
    sign_val = report.get('gyro_theta_rate_sign', float('nan'))
    try:
        sign_str = f"{float(sign_val):.0f}" if np.isfinite(sign_val) else "nan"
    except Exception:
        sign_str = str(sign_val)
    print(f"  Gyro vs dtheta/dt corr: {report.get('gyro_theta_rate_correlation', float('nan')):.3f} (sign {sign_str})")
    print(f"  Corners detected: {report.get('corners_count', 0)}")
    if report.get('corner_integrals_rad'):
        mean_abs = report.get('corner_mean_abs_error_deg', float('nan'))
        print(f"    Mean abs corner error: {mean_abs:.1f} deg")
    mh = report.get('mag_health', {}) or {}
    if mh.get('available', 0) == 1:
        print(f"  Mag norm median: {mh.get('mag_norm_median', float('nan')):.2f}, MAD: {mh.get('mag_norm_mad', float('nan')):.2f}")
        print(f"  Mag heading residual mean: {mh.get('heading_residual_mean_deg', float('nan')):.1f} deg, MAD: {mh.get('heading_residual_mad_deg', float('nan')):.1f} deg")
        print(f"  Gated by heading: {mh.get('pct_heading_gated', float('nan')):.1f}%  by norm: {mh.get('pct_norm_gated', float('nan')):.1f}%")
    sb = report.get('stationary_bias', {}) or {}
    print(f"  Stationary bias: ax {sb.get('accel_x_mean', float('nan')):.3f}±{sb.get('accel_x_std', float('nan')):.3f}, ay {sb.get('accel_y_mean', float('nan')):.3f}±{sb.get('accel_y_std', float('nan')):.3f}, gz {sb.get('gyro_z_mean', float('nan')):.4f}±{sb.get('gyro_z_std', float('nan')):.4f}")


if __name__ == '__main__':
    main()


