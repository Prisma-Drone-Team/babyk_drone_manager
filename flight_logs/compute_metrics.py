#!/usr/bin/env python3
"""
compute_metrics.py
==================
Calcola metriche RMSE per esperimenti in old_logs, con calcolo su
intervalli temporali (prima e dopo il drop).
"""

import argparse
import sys
import numpy as np
import pandas as pd
from pathlib import Path

# ── Configurazione ─────────────────────────────────────────────────────────────
SCRIPT_DIR = Path(__file__).parent
LOGS_DIR   = SCRIPT_DIR.parent / "old_logs"
OUTPUT_DIR = SCRIPT_DIR / "metrics_output"
OUTPUT_DIR.mkdir(exist_ok=True)

# ── Funzioni ───────────────────────────────────────────────────────────────────
def load_data_safe(filename):
    if not filename.exists():
        return None
    try:
        with open(filename, 'r') as f:
            first_line = f.readline()
            second_line = f.readline()
        if ',' in second_line:
            data = np.genfromtxt(filename, delimiter=',', skip_header=1)
        else:
            data = np.genfromtxt(filename, skip_header=1)
        if data.size == 0:
            return None
        if data.ndim == 1:
            data = data.reshape(1, -1)
            
        valid_rows = np.all(np.isfinite(data), axis=1)
        data = data[valid_rows]
        return data if data.size > 0 else None
    except Exception as e:
        print(f"[!] Error loading {filename}: {e}")
        return None

def align_trajectory(x_est, y_est, z_est, roll_est, pitch_est, yaw_est, x_ref, y_ref, z_ref, roll_ref, pitch_ref, yaw_ref):
    """Align estimated trajectory to reference using optimal yaw rotation (Kabsch)"""
    valid = ~(np.isnan(x_est) | np.isnan(y_est) | np.isnan(x_ref) | np.isnan(y_ref) | 
              np.isnan(roll_est) | np.isnan(roll_ref) | np.isnan(yaw_est) | np.isnan(yaw_ref))
    if not np.any(valid):
        return x_est, y_est, z_est, roll_est, pitch_est, yaw_est
        
    idx = np.where(valid)[0][0]

    x_est_c = x_est - x_est[idx]
    y_est_c = y_est - y_est[idx]
    x_ref_c = x_ref - x_ref[idx]
    y_ref_c = y_ref - y_ref[idx]

    H = np.zeros((2, 2))
    H[0, 0] = np.sum(x_est_c[valid] * x_ref_c[valid])
    H[0, 1] = np.sum(x_est_c[valid] * y_ref_c[valid])
    H[1, 0] = np.sum(y_est_c[valid] * x_ref_c[valid])
    H[1, 1] = np.sum(y_est_c[valid] * y_ref_c[valid])

    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T

    if np.linalg.det(R) < 0:
        Vt[1, :] *= -1
        R = Vt.T @ U.T

    delta_yaw = np.arctan2(R[1, 0], R[0, 0])

    x_est_aligned = x_est_c * R[0, 0] + y_est_c * R[0, 1] + x_ref[idx]
    y_est_aligned = x_est_c * R[1, 0] + y_est_c * R[1, 1] + y_ref[idx]
    z_est_aligned = z_est - z_est[idx] + z_ref[idx]

    return x_est_aligned, y_est_aligned, z_est_aligned

def process_experiment(name: str, exp_dir: Path, args) -> dict:
    results = {"experiment": name}

    vio = load_data_safe(exp_dir / "log_vio.txt")
    gt  = load_data_safe(exp_dir / "log_ground_truth.txt")
    opti = load_data_safe(exp_dir / "log_optitrack.txt")

    if vio is None:
        print(f"  [SKIP] Dati VIO mancanti")
        return None

    if gt is None or len(gt) == 0:
        if opti is not None and len(opti) > 0:
            gt = opti
        else:
            print(f"  [SKIP] Dati GT/Optitrack mancanti")
            return None

    t_vio = vio[:, 0]
    x_vio, y_vio, z_vio = vio[:, 1], vio[:, 2], vio[:, 3]
    roll_vio, pitch_vio, yaw_vio = vio[:, 4], vio[:, 5], np.unwrap(vio[:, 6])
    
    # --- HACK PER CAPOVOLGERE LA Y DI OPENVINS ---
    y_vio = -y_vio
    # ---------------------------------------------

    t_gt = gt[:, 0]
    x_gt, y_gt, z_gt = gt[:, 1], gt[:, 2], gt[:, 3]
    roll_gt, pitch_gt, yaw_gt = gt[:, 4], gt[:, 5], np.unwrap(gt[:, 6])

    start_time_offset = min(t_vio[0], t_gt[0])
    t_vio -= start_time_offset
    t_gt  -= start_time_offset

    x_gt_i     = np.interp(t_vio, t_gt, x_gt)
    y_gt_i     = np.interp(t_vio, t_gt, y_gt)
    z_gt_i     = np.interp(t_vio, t_gt, z_gt)
    roll_gt_i  = np.interp(t_vio, t_gt, roll_gt)
    pitch_gt_i = np.interp(t_vio, t_gt, pitch_gt)
    yaw_gt_i   = np.interp(t_vio, t_gt, yaw_gt)

    x_vio_al, y_vio_al, z_vio_al = align_trajectory(
        x_vio, y_vio, z_vio, roll_vio, pitch_vio, yaw_vio,
        x_gt_i, y_gt_i, z_gt_i, roll_gt_i, pitch_gt_i, yaw_gt_i)

    err_x   = np.abs(x_vio_al - x_gt_i)
    err_y   = np.abs(y_vio_al - y_gt_i)
    err_z   = np.abs(z_vio_al - z_gt_i)
    err_3d  = np.sqrt(err_x**2 + err_y**2 + err_z**2)

    results["duration_s"]      = round(t_vio[-1], 1)
    results["mean_3d_err_m"]   = round(float(np.mean(err_3d)), 4)
    results["rmse_3d_m"]       = round(float(np.sqrt(np.mean(err_3d**2))), 4)
    
    # ── Interval analysis ───────────────────────────────────────────────────
    idx_start = 0
    if args.start_time is not None:
        idx_start = np.searchsorted(t_vio, args.start_time)
        
    idx_end = len(t_vio)
    if args.end_time is not None:
        idx_end = np.searchsorted(t_vio, args.end_time)
        
    axes = getattr(args, 'axes', 'z')
    if axes is not None:
        axes = str(axes).lower()
    
    if axes is not None and axes.startswith('xy'):
        err_target = np.sqrt(err_x**2 + err_y**2)
        axis_name = "XY"
    elif axes is not None and (axes.startswith('all') or axes.startswith('xyz')):
        err_target = err_3d
        axis_name = "3D"
    else: # default 'z'
        err_target = err_z
        axis_name = "Z"

    results["target_axes"] = axis_name

    # Global RMSE in the valid interval
    if idx_start < idx_end:
        results["target_rmse_overall_m"] = round(float(np.sqrt(np.mean(err_target[idx_start:idx_end]**2))), 4)
    else:
        results["target_rmse_overall_m"] = None

    # Before / After drop analysis
    if args.drop_time is not None:
        idx_drop = np.searchsorted(t_vio, args.drop_time)
        if idx_start < idx_drop < idx_end:
            rmse_before = np.sqrt(np.mean(err_target[idx_start:idx_drop]**2))
            rmse_after  = np.sqrt(np.mean(err_target[idx_drop:idx_end]**2))
            improvement = ((rmse_before - rmse_after) / rmse_before) * 100.0
            
            results["target_rmse_before_m"] = round(float(rmse_before), 4)
            results["target_rmse_after_m"]  = round(float(rmse_after), 4)
            results["improvement_%"]        = round(float(improvement), 2)
        else:
            results["target_rmse_before_m"] = None
            results["target_rmse_after_m"]  = None
            results["improvement_%"]        = None

    print(f"  RMSE 3D Globale = {results['rmse_3d_m']:.4f} m")
    if args.drop_time is not None and results.get("improvement_%") is not None:
        print(f"  RMSE {axis_name} prima = {results['target_rmse_before_m']:.4f} m, dopo = {results['target_rmse_after_m']:.4f} m (Miglioramento: {results['improvement_%']:.1f}%)")

    return results

def main():
    parser = argparse.ArgumentParser(description="Calcola metriche RMSE per esperimenti in old_logs.")
    parser.add_argument("experiments", nargs="*", help="Nomi delle cartelle in old_logs da elaborare (default: tutte)")
    parser.add_argument("--start_time", type=float, default=None, help="Start time in seconds")
    parser.add_argument("--drop_time", type=float, default=None, help="Drop time in seconds")
    parser.add_argument("--end_time", type=float, default=None, help="End time in seconds")
    parser.add_argument("--axes", type=str, default="z", help="Axes to use for interval RMSE (z, xy, or all)")
    parser.add_argument("--times_csv", type=str, default=None, help="CSV file mapping exp_name -> start_time,drop_time,end_time,axes")
    args = parser.parse_args()

    if not LOGS_DIR.exists():
        print(f"[WARN] Cartella log non trovata: {LOGS_DIR}. Assicurati di eseguire dalla cartella giusta.")
        sys.exit(1)

    if args.experiments:
        experiments = []
        for name in args.experiments:
            d = LOGS_DIR / name
            if not d.is_dir():
                print(f"[WARN] Cartella non trovata: {d} — salto")
            else:
                experiments.append(d)
    else:
        experiments = sorted([d for d in LOGS_DIR.iterdir() if d.is_dir() and not d.name.startswith(".")])

    print(f"Elaboro {len(experiments)} esperimenti:\n")
    
    exp_times = {}
    if args.times_csv is not None:
        try:
            # comment='#' ignora tutte le righe o parti di riga che iniziano con #
            df_times = pd.read_csv(args.times_csv, comment='#')
            for _, row in df_times.iterrows():
                exp_times[str(row['experiment'])] = {
                    'start_time': float(row['start_time']) if 'start_time' in row and pd.notna(row['start_time']) else None,
                    'drop_time': float(row['drop_time']) if 'drop_time' in row and pd.notna(row['drop_time']) else None,
                    'end_time': float(row['end_time']) if 'end_time' in row and pd.notna(row['end_time']) else None,
                    'axes': str(row['axes']).strip().lower() if 'axes' in row and pd.notna(row['axes']) else None,
                }
            print(f"[INFO] Caricati tempi personalizzati per {len(exp_times)} esperimenti da {args.times_csv}")
        except Exception as e:
            print(f"[WARN] Impossibile leggere {args.times_csv}: {e}")

    all_results = []
    for exp_dir in experiments:
        print(f"{'='*60}\nElaborazione: {exp_dir.name}")
        
        # Override args with custom times if available
        class ExpArgs: pass
        exp_args = ExpArgs()
        t_conf = exp_times.get(exp_dir.name, {})
        exp_args.start_time = t_conf.get('start_time', args.start_time)
        exp_args.drop_time  = t_conf.get('drop_time', args.drop_time)
        exp_args.end_time   = t_conf.get('end_time', args.end_time)
        exp_args.axes       = t_conf.get('axes', None) or args.axes
        
        res = process_experiment(exp_dir.name, exp_dir, exp_args)
        if res:
            all_results.append(res)

    if not all_results:
        print("\n[ERRORE] Nessun risultato calcolato.")
        sys.exit(1)

    df = pd.DataFrame(all_results).set_index("experiment")

    csv_path = OUTPUT_DIR / "metrics_summary.csv"
    df.to_csv(csv_path)
    print(f"\nCSV salvato: {csv_path}")

    md_path = OUTPUT_DIR / "metrics_summary.md"
    with open(md_path, "w") as f:
        f.write("# Metriche di Odometria (OpenVINS)\n\n")
        
        start_str = f"{args.start_time}s" if args.start_time is not None else "inizio"
        drop_str  = f"{args.drop_time}s" if args.drop_time is not None else "N/D"
        end_str   = f"{args.end_time}s" if args.end_time is not None else "fine"
        
        f.write(f"**Impostazioni di tempo globali (sovrascritte se presenti in {args.times_csv}):** Start={start_str}, Drop={drop_str}, End={end_str}\n\n")
        f.write(df.to_markdown())
        f.write("\n")
    print(f"Markdown salvato: {md_path}")

    print(f"\n{'='*60}\nRIASSUNTO\n")
    print(df.to_string())
    print()

if __name__ == "__main__":
    main()
