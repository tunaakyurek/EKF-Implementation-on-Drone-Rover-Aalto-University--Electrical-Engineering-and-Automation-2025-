## RoboMaster+iPhone EKF Run Guide (Pi → Online EKF-8 → Offline Analysis)

This guide shows the exact commands and which machine/terminal to use from start to finish.

### Prerequisites
- Raspberry Pi 4 with Python 3.9+ and network access to the iPhone.
- iPhone streaming the sensor data to the Pi (same LAN). Use your existing streaming app/config.

### 1) Raspberry Pi: Setup environment (Terminal on Raspberry Pi)
```bash
cd ~/RoboMaster_S1
python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install -r iphone_integration/requirements_raspberry_pi.txt
```

Optional (first time only):
```bash
sudo apt-get update && sudo apt-get install -y python3-dev libatlas-base-dev
```

### 2) Raspberry Pi: Run Online EKF-8 (main integration)
In this phase, the system performs: pre-delay → calibration → EKF start. EKF origin is anchored at calibration end.
```bash
cd ~/RoboMaster_S1
source .venv/bin/activate
python iphone_integration/pi_phone_connection/main_integration_robomaster.py
```

Notes:
- Logs are written under `iphone_integration/data/` as `robomaster_raw_log_*.csv` and `robomaster_ekf_log_*.csv`.
- Calibration default timing: pre-delay 5s + calibration 5s. EKF starts at (x,y)=(0,0).

### 3) Ground PC (or Pi): Offline EKF-8 Replay (matched to online)
You can run this on the Pi or your PC after copying the raw CSV.
```bash
cd RoboMaster_S1
python -m venv .venv
source .venv/bin/activate  # (Linux/macOS) or .venv\Scripts\activate (Windows)
pip install -r iphone_integration/requirements_iphone.txt

# Run offline replay on latest raw log (example timestamp below)
python iphone_integration/offline_analysis/python/offline_replay_ekf_8dof.py \
  --raw-file iphone_integration/data/robomaster_raw_log_YYYYMMDD_HHMMSS.csv \
  --output-file iphone_integration/data/robomaster_ekf_log_YYYYMMDD_HHMMSS_offline8dof_tuned.csv
```

### 4) Diagnostics and Figures (PC or Pi)
Run diagnostics comparing EKF and raw data (excludes calibration window by flags).
```bash
python iphone_integration/offline_analysis/python/diagnostics_metrics.py \
  --file iphone_integration/data/robomaster_ekf_log_YYYYMMDD_HHMMSS_offline8dof_tuned.csv \
  --raw-file iphone_integration/data/robomaster_raw_log_YYYYMMDD_HHMMSS.csv \
  --output iphone_integration/analysis_results \
  --pre-delay 5 --calibration-duration 5
```
Generated figures:
- `trajectory_segments_*.png`
- `gyro_vs_theta_rate_*.png`
- `mag_health_*.png`

### 5) Online-style Estimation Plot (PC or Pi)
Render a single consolidated PNG like the online plotter (excludes calibration window):
```bash
python iphone_integration/offline_analysis/python/plot_offline_estimation.py \
  --ekf-file iphone_integration/data/robomaster_ekf_log_YYYYMMDD_HHMMSS_offline8dof_tuned.csv \
  --output iphone_integration/analysis_results/estimation_YYYYMMDD_HHMMSS_offline8dof_tuned.png \
  --pre-delay 5 --calibration-duration 5
```

### Tips
- To disable GPS velocity/course fusion online: default is already disabled; see `fusion` flags in `main_integration_robomaster.py` config.
- Ensure the iPhone stream resumes if interrupted (Siri/recording). The receiver is resilient and will continue when data resumes.


