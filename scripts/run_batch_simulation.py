#!/usr/bin/env python3
"""
run_batch_simulation.py
========================
Automated script to run drone corridor exploration simulations N times.
Monitors drone X position in real-time and stops each run when X >= target_x (default: 20.0m).
Computes RMSE 3D, Max Error 3D, and Final Error 3D matching plot_results.py.
Saves logs and metrics summary to output directory.

Usage:
    python3 run_batch_simulation.py --num_runs 5 --target_x 20.0 --timeout 300
"""

import os
import sys
import time
import glob
import shutil
import signal
import argparse
import subprocess
import csv
import numpy as np
from datetime import datetime
from pathlib import Path

# ------------------------------------------------------------------------------
# Trajectory Alignment and Error Calculation (Identical to plot_results.py)
# ------------------------------------------------------------------------------

def load_log(filepath: Path) -> np.ndarray:
    """Loads text log file (Time X Y Z Roll Pitch Yaw)."""
    data = []
    if not filepath.exists():
        return np.array([])
    with open(filepath, "r") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("Time"):
                continue
            parts = line.split()
            if len(parts) >= 7:
                try:
                    data.append([float(p) for p in parts[:7]])
                except ValueError:
                    continue
    return np.array(data)

def align_trajectory(x_est, y_est, z_est, yaw_est, x_ref, y_ref, z_ref, yaw_ref):
    """Aligns estimated trajectory to reference (translation + initial yaw rotation)."""
    x_est = x_est - x_est[0] + x_ref[0]
    y_est = y_est - y_est[0] + y_ref[0]
    z_est = z_est - z_est[0] + z_ref[0]

    delta_yaw = yaw_ref[0] - yaw_est[0]
    x_shifted = x_est - x_ref[0]
    y_shifted = y_est - y_ref[0]

    x_aligned = x_shifted * np.cos(delta_yaw) - y_shifted * np.sin(delta_yaw) + x_ref[0]
    y_aligned = x_shifted * np.sin(delta_yaw) + y_shifted * np.cos(delta_yaw) + y_ref[0]
    yaw_aligned = yaw_est + delta_yaw

    return x_aligned, y_aligned, z_est, yaw_aligned

def compute_run_metrics(gt_file: Path, vio_file: Path) -> dict:
    """Computes RMSE, Max Error, and Final Error from GT and VIO log files."""
    gt_data = load_log(gt_file)
    vio_data = load_log(vio_file)

    if len(gt_data) < 10 or len(vio_data) < 10:
        return {
            "valid": False,
            "error_msg": "Insufficient log data (< 10 samples)",
            "rmse_3d": np.nan, "max_err_3d": np.nan, "final_err_3d": np.nan,
            "rmse_x": np.nan, "rmse_y": np.nan, "rmse_z": np.nan,
            "duration_s": 0.0, "samples_vio": len(vio_data), "samples_gt": len(gt_data)
        }

    t_gt = gt_data[:, 0]
    x_gt, y_gt, z_gt = gt_data[:, 1], gt_data[:, 2], gt_data[:, 3]
    yaw_gt = np.unwrap(gt_data[:, 6])

    t_vio = vio_data[:, 0]
    x_vio, y_vio, z_vio = vio_data[:, 1], vio_data[:, 2], vio_data[:, 3]
    yaw_vio = np.unwrap(vio_data[:, 6])

    # Time normalization
    start_time = min(t_vio[0], t_gt[0])
    t_vio -= start_time
    t_gt -= start_time

    # Interpolate Ground Truth to VIO timestamps
    x_gt_i = np.interp(t_vio, t_gt, x_gt)
    y_gt_i = np.interp(t_vio, t_gt, y_gt)
    z_gt_i = np.interp(t_vio, t_gt, z_gt)
    yaw_gt_i = np.interp(t_vio, t_gt, yaw_gt)

    # Spatial alignment
    x_vio_al, y_vio_al, z_vio_al, yaw_vio_al = align_trajectory(
        x_vio, y_vio, z_vio, yaw_vio,
        x_gt_i, y_gt_i, z_gt_i, yaw_gt_i
    )

    err_x = np.abs(x_vio_al - x_gt_i)
    err_y = np.abs(y_vio_al - y_gt_i)
    err_z = np.abs(z_vio_al - z_gt_i)
    err_3d = np.sqrt(err_x**2 + err_y**2 + err_z**2)

    rmse_3d = float(np.sqrt(np.mean(err_3d**2)))
    max_err_3d = float(np.max(err_3d))
    final_err_3d = float(err_3d[-1])

    # Try computing ATE with EVO library matching compute_metrics.py
    try:
        from evo.core import metrics
        from evo.core.trajectory import PoseTrajectory3D
        from scipy.spatial.transform import Rotation

        roll_gt_i = np.interp(t_vio, t_gt, gt_data[:, 4])
        pitch_gt_i = np.interp(t_vio, t_gt, gt_data[:, 5])
        roll_vio = vio_data[:, 4]
        pitch_vio = vio_data[:, 5]

        def euler_to_quat(roll, pitch, yaw):
            r = Rotation.from_euler("xyz", np.column_stack([roll, pitch, yaw]))
            return r.as_quat()

        def to_pose_traj(timestamps, xyz, quats):
            poses = []
            for i in range(len(timestamps)):
                R = Rotation.from_quat(quats[i]).as_matrix()
                T = np.eye(4)
                T[:3, :3] = R
                T[:3, 3] = xyz[i]
                poses.append(T)
            return PoseTrajectory3D(poses_se3=np.array(poses), timestamps=timestamps)

        quats_gt = euler_to_quat(roll_gt_i, pitch_gt_i, yaw_gt_i)
        quats_vio = euler_to_quat(roll_vio, pitch_vio, yaw_vio_al)

        traj_ref = to_pose_traj(t_vio, np.column_stack([x_gt_i, y_gt_i, z_gt_i]), quats_gt)
        traj_est = to_pose_traj(t_vio, np.column_stack([x_vio_al, y_vio_al, z_vio_al]), quats_vio)

        ate_metric = metrics.APE(metrics.PoseRelation.translation_part)
        ate_metric.process_data((traj_ref, traj_est))
        ate_stats = ate_metric.get_all_statistics()
        ate_errors = ate_metric.error

        rmse_3d = float(ate_stats["rmse"])
        max_err_3d = float(ate_stats["max"])
        final_err_3d = float(ate_errors[-1])
    except Exception:
        pass

    rmse_x = float(np.sqrt(np.mean(err_x**2)))
    rmse_y = float(np.sqrt(np.mean(err_y**2)))
    rmse_z = float(np.sqrt(np.mean(err_z**2)))

    return {
        "valid": True,
        "error_msg": "",
        "rmse_3d": round(rmse_3d, 4),
        "max_err_3d": round(max_err_3d, 4),
        "final_err_3d": round(final_err_3d, 4),
        "rmse_x": round(rmse_x, 4),
        "rmse_y": round(rmse_y, 4),
        "rmse_z": round(rmse_z, 4),
        "duration_s": round(float(t_vio[-1]), 1),
        "samples_vio": len(vio_data),
        "samples_gt": len(gt_data)
    }

# ------------------------------------------------------------------------------
# Process Management & Simulator Lifecycle
# ------------------------------------------------------------------------------

def graceful_stop_logger():
    """Sends SIGINT (signal 2) to flight_data_logger so rclcpp::on_shutdown executes save_data()."""
    sys.stdout.write("  [Save] Sending SIGINT to flight_data_logger to flush log files to disk...\n")
    sys.stdout.flush()

    subprocess.run(["pkill", "-2", "-f", "flight_data_logger"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-2", "-f", "vio_recovery"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(3.5)  # Wait for save_data() to finish writing log_*.txt files

def cleanup_simulation(yaml_path: str, socket_name: str = "drone_pathplanner", session_name: str = "vins_simulation"):
    """Terminates all simulation processes cleanly using tmux kill-server."""
    sys.stdout.write("  [Clean] Terminating simulation server and child processes...\n")
    sys.stdout.flush()

    subprocess.run(["tmux", "kill-server"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.run(["tmux", "-L", socket_name, "kill-server"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    procs_to_kill = [
        "px4", "gz sim", "gz-sim-gui", "gz", "ruby", "MicroXRCEAgent",
        "parameter_bridge", "move_manager", "ov_msckf", "path_planner", "traj_interp", "flight_data_logger"
    ]
    for proc in procs_to_kill:
        subprocess.run(["pkill", "-9", "-f", proc], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    subprocess.run(["ros2", "daemon", "stop"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(4)

def get_drone_x_position(domain_id: str = "56") -> float:
    """Queries current drone X position using ros2 topic echo with ROS_DOMAIN_ID."""
    env = os.environ.copy()
    if domain_id:
        env["ROS_DOMAIN_ID"] = str(domain_id)

    topics_to_try = [
        "/model/baby_k_0/odometry",
        "/px4/odometry/out",
        "/ov_msckf/odomimu"
    ]

    for topic in topics_to_try:
        try:
            cmd = ["ros2", "topic", "echo", topic, "--once"]
            res = subprocess.run(cmd, capture_output=True, text=True, timeout=2.0, env=env)
            if res.returncode == 0 and res.stdout:
                lines = res.stdout.splitlines()
                in_pos = False
                for line in lines:
                    if "position:" in line:
                        in_pos = True
                    elif in_pos and "x:" in line:
                        val_str = line.split("x:")[1].strip()
                        return float(val_str)
        except Exception:
            pass
    return None

def monitor_run(target_x: float, timeout_sec: int, domain_id: str = "56", min_warmup_sec: float = 25.0) -> tuple[str, float]:
    """
    Monitors simulation progress until drone reaches target_x or timeout occurs.
    Enforces min_warmup_sec before evaluating target_x to ignore initial startup odometry.
    Returns (status, max_x_reached).
    """
    start_time = time.time()
    max_x_reached = -999.0
    last_print = 0

    while True:
        elapsed = time.time() - start_time
        if elapsed > timeout_sec:
            return "TIMEOUT", max_x_reached

        current_x = get_drone_x_position(domain_id=domain_id)
        if current_x is not None:
            # Only update max_x_reached after warmup to ignore stale topics
            if elapsed >= min_warmup_sec:
                if current_x > max_x_reached:
                    max_x_reached = current_x

                if current_x >= target_x:
                    return "REACHED_END", max_x_reached

        # Print progress every 3 seconds with line-clearing escape sequence \033[K
        if time.time() - last_print >= 3.0:
            x_str = f"{current_x:.2f}m" if current_x is not None else "waiting..."
            warmup_status = " [Warmup]" if elapsed < min_warmup_sec else ""
            print(f"\r\033[K  ⌛ Progress: {elapsed:.0f}s / {timeout_sec}s | Drone X: {x_str} (Target: {target_x}m){warmup_status}", end="", flush=True)
            last_print = time.time()

        time.sleep(1.0)

# ------------------------------------------------------------------------------
# Main Batch Execution
# ------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Automated Drone Batch Simulation Runner")
    parser.add_argument("--num_runs", type=int, default=5, help="Number of simulation runs")
    parser.add_argument("--target_x", type=float, default=20.0, help="Target X coordinate indicating corridor end")
    parser.add_argument("--timeout", type=int, default=500, help="Max safety timeout per run in seconds")
    parser.add_argument("--yaml_path", type=str, default="", help="Path to tmuxp YAML configuration")
    parser.add_argument("--enable_rviz", action="store_true", help="Enable RViz (default: false for lightweight execution)")
    parser.add_argument("--save_all_logs", action="store_true", help="Save all log files including tactile/wrench (default: false, only GT and VIO saved)")
    parser.add_argument("--output_dir", type=str, default="", help="Directory to save batch metrics & logs")
    args = parser.parse_args()

    script_dir = Path(__file__).resolve().parent
    pkg_dir = script_dir.parent
    pkg_parent = pkg_dir.parent
    
    if not args.yaml_path:
        yaml_path = pkg_dir / "utils" / "exploration.yml"
    else:
        yaml_path = Path(args.yaml_path)

    if not args.output_dir:
        output_dir = pkg_parent / "vio_recovery" / "metrics_output"
    else:
        output_dir = Path(args.output_dir)

    flight_logs_dir = pkg_parent / "vio_recovery" / "flight_logs"

    timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    batch_dir = output_dir / f"batch_{timestamp_str}"
    batch_dir.mkdir(parents=True, exist_ok=True)

    print("=" * 60)
    print("🚀 AUTOMATED BATCH SIMULATION RUNNER")
    print(f"  • Total Runs:        {args.num_runs}")
    print(f"  • Target X Threshold:{args.target_x} m")
    print(f"  • Max Timeout/Run:   {args.timeout} s")
    print(f"  • RViz Enabled:      {args.enable_rviz}")
    print(f"  • YAML Config:       {yaml_path}")
    print(f"  • Output Directory:  {batch_dir}")
    print("=" * 60)

    # Prepare lightweight YAML if RViz is disabled
    effective_yaml = yaml_path
    if not args.enable_rviz:
        lightweight_yaml = yaml_path.parent / "exploration_batch_headless.yml"
        with open(yaml_path, "r") as f:
            content = f.read()
        content = content.replace("rviz_enable:=true", "rviz_enable:=false")
        with open(lightweight_yaml, "w") as f:
            f.write(content)
        effective_yaml = lightweight_yaml
        print(f"  [Info] Created lightweight YAML configuration (RViz disabled): {lightweight_yaml.name}")

    results = []

    def signal_handler(sig, frame):
        print("\n\n⚠️ Interrupted by user! Running final cleanup...")
        cleanup_simulation(str(effective_yaml))
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    for run_idx in range(1, args.num_runs + 1):
        print(f"\n--------------------------------------------------")
        print(f"▶️  STARTING RUN {run_idx} / {args.num_runs} [{datetime.now().strftime('%H:%M:%S')}]")
        print(f"--------------------------------------------------")

        # 1. Cleanup old session
        cleanup_simulation(str(effective_yaml))

        # 2. Launch simulation with tmuxp
        print("  [Launch] Starting simulation session via tmuxp...")
        subprocess.run(["tmuxp", "load", "-d", "-y", str(effective_yaml)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        print("  ⏳ Waiting 20s for PX4 SITL and ROS 2 node startup...")
        time.sleep(20)  # Wait for PX4 & ROS nodes initialization

        # 3. Monitor execution
        status, max_x = monitor_run(args.target_x, args.timeout)
        # 4. Flush logger data to disk before process cleanup
        graceful_stop_logger()

        # 5. Save only essential log files (Ground Truth & VIO) required to compute RMSE, max error, and final error
        run_log_dir = batch_dir / f"run_{run_idx}"
        run_log_dir.mkdir(parents=True, exist_ok=True)

        essential_files = ["log_ground_truth.txt", "log_vio.txt"]
        if args.save_all_logs:
            files_to_copy = [Path(p).name for p in glob.glob(str(flight_logs_dir / "*.txt")) + glob.glob(str(flight_logs_dir / "*.bt"))]
        else:
            files_to_copy = essential_files

        for fname in files_to_copy:
            src = flight_logs_dir / fname
            if src.exists():
                shutil.copy(src, run_log_dir / fname)

        # 6. Compute metrics
        gt_log = run_log_dir / "log_ground_truth.txt"
        vio_log = run_log_dir / "log_vio.txt"
        metrics = compute_run_metrics(gt_log, vio_log)

        # 7. Now clean up remaining simulation processes for next run
        cleanup_simulation(str(effective_yaml))

        # Cleanup flight_logs directory to prevent disk fill-up
        for f in flight_logs_dir.glob("log_*"):
            try:
                f.unlink()
            except Exception:
                pass

        # Check for VIO divergence (e.g. RMSE > 10.0m)
        run_status = status
        if metrics["valid"] and not np.isnan(metrics["rmse_3d"]) and metrics["rmse_3d"] > 10.0:
            run_status = "DIVERGED"

        run_result = {
            "run": run_idx,
            "status": run_status,
            "max_x_m": round(max_x, 2),
            "duration_s": metrics["duration_s"],
            "rmse_3d_m": metrics["rmse_3d"],
            "max_err_3d_m": metrics["max_err_3d"],
            "final_err_3d_m": metrics["final_err_3d"],
            "rmse_x_m": metrics["rmse_x"],
            "rmse_y_m": metrics["rmse_y"],
            "rmse_z_m": metrics["rmse_z"],
            "valid": metrics["valid"]
        }
        results.append(run_result)

        print(f"  📊 Metrics: Status = {run_status} | RMSE 3D = {metrics['rmse_3d']}m | Max Error = {metrics['max_err_3d']}m | Final Error = {metrics['final_err_3d']}m")

    # --------------------------------------------------------------------------
    # Save Master CSV & Markdown Summaries
    # --------------------------------------------------------------------------

    csv_path = batch_dir / "batch_summary.csv"
    if results:
        fieldnames = list(results[0].keys())
        with open(csv_path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(results)

    md_path = batch_dir / "batch_summary.md"
    with open(md_path, "w") as f:
        f.write(f"# Batch Simulation Summary ({timestamp_str})\n\n")
        f.write(f"- **Total Runs**: {args.num_runs}\n")
        f.write(f"- **Target X Threshold**: {args.target_x} m\n")
        f.write(f"- **Timeout per Run**: {args.timeout} s\n\n")

        f.write("## Per-Run Results\n\n")
        f.write("| Run | Status | Max X (m) | Duration (s) | RMSE 3D (m) | Max Error 3D (m) | Final Error 3D (m) |\n")
        f.write("|-----|--------|-----------|--------------|-------------|------------------|--------------------|\n")
        for r in results:
            f.write(f"| {r['run']} | {r['status']} | {r['max_x_m']} | {r['duration_s']} | {r['rmse_3d_m']} | {r['max_err_3d_m']} | {r['final_err_3d_m']} |\n")

        # Summary statistics using numpy (Filtering out diverged runs for accuracy statistics)
        successful_runs = [r for r in results if r["valid"] and r["status"] == "REACHED_END" and not np.isnan(r["rmse_3d_m"]) and r["rmse_3d_m"] < 10.0]
        diverged_runs = [r for r in results if r["status"] == "DIVERGED"]

        f.write("\n## Statistics (Successful Non-Diverged Runs)\n\n")
        if successful_runs:
            rmse_vals = [r["rmse_3d_m"] for r in successful_runs]
            max_err_vals = [r["max_err_3d_m"] for r in successful_runs]
            final_err_vals = [r["final_err_3d_m"] for r in successful_runs]

            f.write(f"- **Mean RMSE 3D**: {np.mean(rmse_vals):.4f} m (Std: {np.std(rmse_vals):.4f} m)\n")
            f.write(f"- **Mean Max Error**: {np.mean(max_err_vals):.4f} m\n")
            f.write(f"- **Mean Final Error**: {np.mean(final_err_vals):.4f} m\n")

        f.write(f"- **Success Rate (Normal Arrival)**: {len(successful_runs)} / {len(results)} ({len(successful_runs)/len(results)*100:.1f}%)\n")
        f.write(f"- **Divergence Rate**: {len(diverged_runs)} / {len(results)} ({len(diverged_runs)/len(results)*100:.1f}%)\n")

    print("\n" + "=" * 60)
    print("🎉 ALL BATCH SIMULATIONS COMPLETED SUCCESSFULLY!")
    print(f"📁 Results saved to: {batch_dir}")
    print(f"📄 Summary CSV:       {csv_path.name}")
    print(f"📄 Summary Markdown:  {md_path.name}")
    print("=" * 60)

if __name__ == "__main__":
    main()
