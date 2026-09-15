#!/usr/bin/env python3
import argparse
import numpy as np
import matplotlib.pyplot as plt
import os

def add_health_background(ax, t_health, health_states, t_min, t_max):
    """Add colored background based on VIO health state."""
    colors = {
        0: ('#81C784', 0.6),  # CONSISTENT - Green
        1: ('#81D4FA', 0.6),  # POTENTIALLY_CONSISTENT - Light blue
        2: ('#FFB74D', 0.6),  # POTENTIALLY_INCONSISTENT - Orange
        3: ('#E57373', 0.6)   # INCONSISTENT - Red
    }
    if len(t_health) == 0:
        return
    for i in range(len(t_health) - 1):
        state = int(health_states[i])
        color, alpha = colors.get(state, ('gray', 0.1))
        ax.axvspan(t_health[i], t_health[i+1], facecolor=color, alpha=alpha, linewidth=0)
    state = int(health_states[-1])
    color, alpha = colors.get(state, ('gray', 0.1))
    ax.axvspan(t_health[-1], t_max, facecolor=color, alpha=alpha, linewidth=0)

def add_fsm_background(ax, t_fsm, fsm_states, t_min, t_max):
    """Add colored background based on FSM state."""
    colors = {
        0: ('white', 0.0),    # NAVIGATE
        1: ('#FFF59D', 0.5),  # STRAFE - Yellow
        2: ('#CE93D8', 0.5),  # SWIPE - Purple
        3: ('#80CBC4', 0.5)   # RETURN - Teal
    }
    if len(t_fsm) == 0:
        return
    for i in range(len(t_fsm) - 1):
        state = int(fsm_states[i])
        if state in colors:
            color, alpha = colors[state]
            if alpha > 0:
                ax.axvspan(t_fsm[i], t_fsm[i+1], facecolor=color, alpha=alpha, linewidth=0)
    state = int(fsm_states[-1])
    if state in colors:
        color, alpha = colors[state]
        if alpha > 0:
            ax.axvspan(t_fsm[-1], t_max, facecolor=color, alpha=alpha, linewidth=0)

def myPlot(time, data_list, labels, title, ncols=2, use_tex=False, t_health=None, health_states=None, t_fsm=None, fsm_states=None):
    plt.rcParams.update({"text.usetex": use_tex, "font.family": "serif"})
    n = len(data_list)
    nrows = int(np.ceil(n / ncols))
    fig, axes = plt.subplots(nrows, ncols, figsize=(12, 3.5 * nrows), squeeze=False)
    axes = axes.flatten()
    
    for i in range(n):
        time_plot = time[:len(data_list[i]['vio'])]
        
        axes[i].plot(time_plot, data_list[i]['vio'], 'k-', label='OpenVINS (VIO)', linewidth=1.5)
        
        if 'px4' in data_list[i] and data_list[i]['px4'] is not None:
            axes[i].plot(time_plot, data_list[i]['px4'][:len(time_plot)], 'b--', label='PX4 EKF2', linewidth=1.5)
            
        if 'tactile' in data_list[i] and data_list[i]['tactile'] is not None:
            axes[i].plot(time_plot, data_list[i]['tactile'][:len(time_plot)], 'g-.', label='PX4 VIO Echo', linewidth=1.5)
            
        if 'opti' in data_list[i] and data_list[i]['opti'] is not None:
            axes[i].plot(time_plot, data_list[i]['opti'][:len(time_plot)], 'm:', label='OptiTrack', linewidth=1.5)
        
        if 'ref' in data_list[i] and data_list[i]['ref'] is not None:
            ref_data = data_list[i]['ref']
            if np.isscalar(ref_data):
                axes[i].axhline(y=ref_data, color='r', linestyle='--', label='Reference')
            else:
                axes[i].plot(time_plot, ref_data[:len(time_plot)], 'r--', label='Reference (OptiTrack/GT)', linewidth=1.2)
        
        axes[i].set_title(labels[i])
        axes[i].set_xlabel(r"$t$ [s]")
        axes[i].grid(True, linestyle='-', alpha=0.3)

        if t_health is not None and health_states is not None:
            add_health_background(axes[i], t_health, health_states, time_plot[0], time_plot[-1])
        if t_fsm is not None and fsm_states is not None:
            add_fsm_background(axes[i], t_fsm, fsm_states, time_plot[0], time_plot[-1])

        if "Lambda" in labels[i]:
            axes[i].axhline(y=350, color='r', linestyle='--', alpha=0.8, label=r'$K_{j,1}$')
            axes[i].axhline(y=450, color='b', linestyle='--', alpha=0.8, label=r'$K_{j,2}$')

        axes[i].legend(loc='best', fontsize='small')
    
    for j in range(n, len(axes)):
        fig.delaxes(axes[j])
        
    fig.suptitle(title, fontsize=16)
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    return fig

def load_data_safe(filename):
    if not os.path.exists(filename):
        print(f"[!] File not found: {filename}")
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
            
        # Ignore all rows with NaN or Inf
        valid_rows = np.all(np.isfinite(data), axis=1)
        data = data[valid_rows]
        
        if data.size == 0:
            print(f"[!] All rows in {filename} were invalid (NaN/Inf).")
            return None
            
        return data
    except Exception as e:
        print(f"[!] Error loading {filename}: {e}")
        return None

def apply_ned_to_enu(x, y, z, roll, pitch, yaw):
    """Convert position and orientation from NED to ENU frame."""
    R_ned_to_enu = np.array([
        [0, 1, 0],
        [1, 0, 0],
        [0, 0, -1]
    ])
    pos_ned = np.column_stack((x, y, z))
    pos_enu = np.dot(pos_ned, R_ned_to_enu.T)
    x_enu = pos_enu[:, 0]
    y_enu = pos_enu[:, 1]
    z_enu = pos_enu[:, 2]
    yaw_enu = np.pi / 2.0 - yaw
    yaw_enu = (yaw_enu + np.pi) % (2 * np.pi) - np.pi
    pitch_enu = -pitch
    roll_enu = roll
    return x_enu, y_enu, z_enu, roll_enu, pitch_enu, yaw_enu

def align_trajectory(x_est, y_est, z_est, roll_est, pitch_est, yaw_est, x_ref, y_ref, z_ref, roll_ref, pitch_ref, yaw_ref):
    """Align estimated trajectory to reference using optimal yaw rotation (Kabsch) for position,
       and constant SO(3) offset for orientation calibration using the first frame."""
    
    # Ignore any rows where data is NaN
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

    def rpy_to_R(r, p, y):
        cz, sz = np.cos(y), np.sin(y)
        cy, sy = np.cos(p), np.sin(p)
        cx, sx = np.cos(r), np.sin(r)
        R_mat = np.zeros((len(r), 3, 3))
        R_mat[:,0,0] = cz*cy
        R_mat[:,0,1] = cz*sy*sx - sz*cx
        R_mat[:,0,2] = cz*sy*cx + sz*sx
        R_mat[:,1,0] = sz*cy
        R_mat[:,1,1] = sz*sy*sx + cz*cx
        R_mat[:,1,2] = sz*sy*cx - cz*sx
        R_mat[:,2,0] = -sy
        R_mat[:,2,1] = cy*sx
        R_mat[:,2,2] = cy*cx
        return R_mat

    def R_to_rpy(R_mat):
        sy = np.sqrt(R_mat[:,0,0]**2 + R_mat[:,1,0]**2)
        singular = sy < 1e-6
        roll = np.zeros(len(R_mat))
        pitch = np.zeros(len(R_mat))
        yaw = np.zeros(len(R_mat))
        ns = ~singular
        roll[ns] = np.arctan2(R_mat[ns,2,1], R_mat[ns,2,2])
        pitch[ns] = np.arctan2(-R_mat[ns,2,0], sy[ns])
        yaw[ns] = np.arctan2(R_mat[ns,1,0], R_mat[ns,0,0])
        s = singular
        roll[s] = np.arctan2(-R_mat[s,1,2], R_mat[s,1,1])
        pitch[s] = np.arctan2(-R_mat[s,2,0], sy[s])
        yaw[s] = 0
        return np.unwrap(roll), np.unwrap(pitch), np.unwrap(yaw)

    R_est = rpy_to_R(roll_est, pitch_est, yaw_est)
    R_ref = rpy_to_R(roll_ref, pitch_ref, yaw_ref)
    
    A = np.array([
        [np.cos(delta_yaw), -np.sin(delta_yaw), 0],
        [np.sin(delta_yaw),  np.cos(delta_yaw), 0],
        [0, 0, 1]
    ])
    
    R_est_0_aligned_world = A @ R_est[idx]
    B = R_est_0_aligned_world.T @ R_ref[idx]
    
    R_est_aligned_world = np.einsum('ij,njk->nik', A, R_est)
    R_aligned = np.einsum('nij,jk->nik', R_est_aligned_world, B)
    
    roll_aligned, pitch_aligned, yaw_aligned = R_to_rpy(R_aligned)

    return x_est_aligned, y_est_aligned, z_est_aligned, roll_aligned, pitch_aligned, yaw_aligned

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tex", action="store_true", help="Use LaTeX fonts")
    ap.add_argument("--save", action="store_true", help="Save figures as PNG")
    ap.add_argument("--folder", type=str, default="", help="Folder in old_logs containing the txt files")
    ap.add_argument("--start_time", type=float, default=None, help="Start time of the test in seconds (to crop Z RMSE calculation)")
    ap.add_argument("--drop_time", type=float, default=None, help="Time of the drop in seconds (to split Z RMSE calculation)")
    ap.add_argument("--end_time", type=float, default=None, help="End time of the test in seconds (to crop Z RMSE calculation)")
    ap.add_argument("--all_plots", action="store_true", help="Show all plots instead of just the main tracking ones")
    args = ap.parse_args()

    base_path = ""
    if args.folder:
        base_path = os.path.join("../old_logs", args.folder)
        if not os.path.exists(base_path):
            print(f"[!] Directory not found: {base_path}")
            return

    print(f"[*] Loading data files from {'current directory' if not args.folder else base_path}...")
    px4     = load_data_safe(os.path.join(base_path, 'log_px4.txt') if base_path else 'log_px4.txt')
    tactile = load_data_safe(os.path.join(base_path, 'log_tactile.txt') if base_path else 'log_tactile.txt')
    vio     = load_data_safe(os.path.join(base_path, 'log_vio.txt') if base_path else 'log_vio.txt')
    gt      = load_data_safe(os.path.join(base_path, 'log_ground_truth.txt') if base_path else 'log_ground_truth.txt')
    lam     = load_data_safe(os.path.join(base_path, 'log_eigenvalues.txt') if base_path else 'log_eigenvalues.txt')
    health  = load_data_safe(os.path.join(base_path, 'log_health.txt') if base_path else 'log_health.txt')
    wrench  = load_data_safe(os.path.join(base_path, 'log_wrench.txt') if base_path else 'log_wrench.txt')
    teleop  = load_data_safe(os.path.join(base_path, 'log_teleop.txt') if base_path else 'log_teleop.txt')
    fsm     = load_data_safe(os.path.join(base_path, 'log_fsm.txt') if base_path else 'log_fsm.txt')
    opti    = load_data_safe(os.path.join(base_path, 'log_optitrack.txt') if base_path else 'log_optitrack.txt')

    has_opti = opti is not None and len(opti) > 0

    if vio is None or len(vio) == 0:
        print("[!] Critical error: missing VIO data (log_vio.txt is empty).")
        return

    has_gt = gt is not None and len(gt) > 0
    if not has_gt:
        if has_opti:
            print("[*] No Gazebo Ground Truth found. Using OptiTrack as Ground Truth reference.")
            gt = opti
            has_gt = True
            # We don't want to plot OptiTrack twice, so we disable it as a separate line
            has_opti = False
        else:
            print("[!] Critical error: missing both Ground Truth and OptiTrack data for reference.")
            return

    # Extract VIO data
    t_vio = vio[:, 0]
    x_vio, y_vio, z_vio = vio[:, 1], vio[:, 2], vio[:, 3]
    roll_vio, pitch_vio, yaw_vio = vio[:, 4], vio[:, 5], np.unwrap(vio[:, 6])

    # --- HACK PER CAPOVOLGERE LA Y DI OPENVINS ---
    # Rimuovi la riga qui sotto per tornare alla normalità
    #y_vio = -y_vio
    # ---------------------------------------------

    # Extract ground truth data
    t_gt = gt[:, 0]
    x_gt, y_gt, z_gt = gt[:, 1], gt[:, 2], gt[:, 3]
    roll_gt, pitch_gt, yaw_gt = gt[:, 4], gt[:, 5], np.unwrap(gt[:, 6])

    # Extract PX4 data
    has_px4 = px4 is not None and len(px4) > 0
    if has_px4:
        t_px4 = px4[:, 0]
        x_px4, y_px4, z_px4 = px4[:, 1], px4[:, 2], px4[:, 3]
        roll_px4, pitch_px4, yaw_px4 = px4[:, 4], px4[:, 5], px4[:, 6]
        print("[*] Converting PX4 odometry from NED to ENU...")
        x_px4, y_px4, z_px4, roll_px4, pitch_px4, yaw_px4 = apply_ned_to_enu(
            x_px4, y_px4, z_px4, roll_px4, pitch_px4, yaw_px4)
        yaw_px4 = np.unwrap(yaw_px4)

    # Extract Tactile data
    has_tactile = tactile is not None and len(tactile) > 0
    if has_tactile:
        t_tactile = tactile[:, 0]
        x_tactile, y_tactile, z_tactile = tactile[:, 1], tactile[:, 2], tactile[:, 3]
        roll_tactile, pitch_tactile, yaw_tactile = tactile[:, 4], tactile[:, 5], tactile[:, 6]
        print("[*] Converting Tactile odometry from NED to ENU...")
        x_tactile, y_tactile, z_tactile, roll_tactile, pitch_tactile, yaw_tactile = apply_ned_to_enu(
            x_tactile, y_tactile, z_tactile, roll_tactile, pitch_tactile, yaw_tactile)
        yaw_tactile = np.unwrap(yaw_tactile)

    # Extract OptiTrack data (only if not used as GT)
    if has_opti:
        t_opti = opti[:, 0]
        x_opti, y_opti, z_opti = opti[:, 1], opti[:, 2], opti[:, 3]
        roll_opti, pitch_opti, yaw_opti = opti[:, 4], opti[:, 5], np.unwrap(opti[:, 6])
        print(f"[*] Loaded {len(t_opti)} OptiTrack records.")

    # Extract health data
    has_health = health is not None and len(health) > 0
    t_health, health_states = None, None
    if has_health:
        t_health = health[:, 0]
        health_states = health[:, 1]
        print(f"[*] Loaded {len(t_health)} VIO health records.")

    has_wrench = wrench is not None and len(wrench) > 0
    if has_wrench:
        t_wrench = wrench[:, 0]
        fx, fy, fz = wrench[:, 1], wrench[:, 2], wrench[:, 3]
        tx, ty, tz = wrench[:, 4], wrench[:, 5], wrench[:, 6]
        print(f"[*] Loaded {len(t_wrench)} Wrench records.")

    # Extract Teleop data
    has_teleop = teleop is not None and len(teleop) > 0
    if has_teleop:
        t_teleop = teleop[:, 0]
        vx_tel, vy_tel, vz_tel = teleop[:, 1], teleop[:, 2], teleop[:, 3]
        wx_tel, wy_tel, wz_tel = teleop[:, 4], teleop[:, 5], teleop[:, 6]
        print(f"[*] Loaded {len(t_teleop)} Teleop records.")

    # Extract FSM data
    has_fsm = fsm is not None and len(fsm) > 0
    t_fsm, fsm_states = None, None
    if has_fsm:
        t_fsm = fsm[:, 0]
        fsm_states = fsm[:, 1]
        print(f"[*] Loaded {len(t_fsm)} FSM records.")

    # Time alignment
    start_time = min(t_vio[0], t_gt[0])
    t_vio -= start_time
    t_gt  -= start_time
    if has_px4:      t_px4     -= start_time
    if has_tactile:  t_tactile -= start_time
    if has_health:   t_health  -= start_time
    if has_wrench:   t_wrench  -= start_time
    if has_teleop:   t_teleop  -= start_time
    if has_fsm:      t_fsm     -= start_time
    if has_opti:     t_opti    -= start_time

    # Interpolate GT and PX4/Tactile onto VIO timestamps
    x_gt_interp     = np.interp(t_vio, t_gt, x_gt)
    y_gt_interp     = np.interp(t_vio, t_gt, y_gt)
    z_gt_interp     = np.interp(t_vio, t_gt, z_gt)
    roll_gt_interp  = np.interp(t_vio, t_gt, roll_gt)
    pitch_gt_interp = np.interp(t_vio, t_gt, pitch_gt)
    yaw_gt_interp   = np.interp(t_vio, t_gt, yaw_gt)

    if has_px4:
        x_px4_interp     = np.interp(t_vio, t_px4, x_px4)
        y_px4_interp     = np.interp(t_vio, t_px4, y_px4)
        z_px4_interp     = np.interp(t_vio, t_px4, z_px4)
        roll_px4_interp  = np.interp(t_vio, t_px4, roll_px4)
        pitch_px4_interp = np.interp(t_vio, t_px4, pitch_px4)
        yaw_px4_interp   = np.interp(t_vio, t_px4, yaw_px4)

    if has_tactile:
        x_tactile_interp     = np.interp(t_vio, t_tactile, x_tactile)
        y_tactile_interp     = np.interp(t_vio, t_tactile, y_tactile)
        z_tactile_interp     = np.interp(t_vio, t_tactile, z_tactile)
        roll_tactile_interp  = np.interp(t_vio, t_tactile, roll_tactile)
        pitch_tactile_interp = np.interp(t_vio, t_tactile, pitch_tactile)
        yaw_tactile_interp   = np.interp(t_vio, t_tactile, yaw_tactile)

    if has_opti:
        x_opti_interp     = np.interp(t_vio, t_opti, x_opti)
        y_opti_interp     = np.interp(t_vio, t_opti, y_opti)
        z_opti_interp     = np.interp(t_vio, t_opti, z_opti)
        roll_opti_interp  = np.interp(t_vio, t_opti, roll_opti)
        pitch_opti_interp = np.interp(t_vio, t_opti, pitch_opti)
        yaw_opti_interp   = np.interp(t_vio, t_opti, yaw_opti)

    # Truncate all data if end_time is specified
    if args.end_time is not None:
        print(f"[*] Truncating data up to t = {args.end_time}s")
        idx_vio = t_vio <= args.end_time
        t_vio = t_vio[idx_vio]
        x_vio, y_vio, z_vio = x_vio[idx_vio], y_vio[idx_vio], z_vio[idx_vio]
        roll_vio, pitch_vio, yaw_vio = roll_vio[idx_vio], pitch_vio[idx_vio], yaw_vio[idx_vio]
        
        x_gt_interp, y_gt_interp, z_gt_interp = x_gt_interp[idx_vio], y_gt_interp[idx_vio], z_gt_interp[idx_vio]
        roll_gt_interp, pitch_gt_interp, yaw_gt_interp = roll_gt_interp[idx_vio], pitch_gt_interp[idx_vio], yaw_gt_interp[idx_vio]

        if has_px4:
            x_px4_interp, y_px4_interp, z_px4_interp = x_px4_interp[idx_vio], y_px4_interp[idx_vio], z_px4_interp[idx_vio]
            roll_px4_interp, pitch_px4_interp, yaw_px4_interp = roll_px4_interp[idx_vio], pitch_px4_interp[idx_vio], yaw_px4_interp[idx_vio]

        if has_tactile:
            x_tactile_interp, y_tactile_interp, z_tactile_interp = x_tactile_interp[idx_vio], y_tactile_interp[idx_vio], z_tactile_interp[idx_vio]
            roll_tactile_interp, pitch_tactile_interp, yaw_tactile_interp = roll_tactile_interp[idx_vio], pitch_tactile_interp[idx_vio], yaw_tactile_interp[idx_vio]

        if has_opti:
            x_opti_interp, y_opti_interp, z_opti_interp = x_opti_interp[idx_vio], y_opti_interp[idx_vio], z_opti_interp[idx_vio]
            roll_opti_interp, pitch_opti_interp, yaw_opti_interp = roll_opti_interp[idx_vio], pitch_opti_interp[idx_vio], yaw_opti_interp[idx_vio]

        if has_teleop:
            idx_tel = t_teleop <= args.end_time
            t_teleop = t_teleop[idx_tel]
            vx_tel, vy_tel, vz_tel = vx_tel[idx_tel], vy_tel[idx_tel], vz_tel[idx_tel]
            wx_tel, wy_tel, wz_tel = wx_tel[idx_tel], wy_tel[idx_tel], wz_tel[idx_tel]
            
        if has_fsm:
            idx_fsm = t_fsm <= args.end_time
            t_fsm = t_fsm[idx_fsm]
            fsm_states = fsm_states[idx_fsm]
            
        if has_wrench:
            idx_wr = t_wrench <= args.end_time
            t_wrench = t_wrench[idx_wr]
            fx, fy, fz = fx[idx_wr], fy[idx_wr], fz[idx_wr]
            tx, ty, tz = tx[idx_wr], ty[idx_wr], tz[idx_wr]
            
        if has_health:
            idx_hl = t_health <= args.end_time
            t_health = t_health[idx_hl]
            health_states = health_states[idx_hl]
            
        if lam is not None and len(lam) > 0:
            idx_lam = (lam[:, 0] - start_time) <= args.end_time
            lam = lam[idx_lam]

    # Spatial alignment
    print("[*] Aligning trajectories spatially...")
    x_vio, y_vio, z_vio, roll_vio, pitch_vio, yaw_vio = align_trajectory(
        x_vio, y_vio, z_vio, roll_vio, pitch_vio, yaw_vio,
        x_gt_interp, y_gt_interp, z_gt_interp, roll_gt_interp, pitch_gt_interp, yaw_gt_interp)

    if has_px4:
        x_px4_interp, y_px4_interp, z_px4_interp, roll_px4_interp, pitch_px4_interp, yaw_px4_interp = align_trajectory(
            x_px4_interp, y_px4_interp, z_px4_interp, roll_px4_interp, pitch_px4_interp, yaw_px4_interp,
            x_gt_interp, y_gt_interp, z_gt_interp, roll_gt_interp, pitch_gt_interp, yaw_gt_interp)

    if has_tactile:
        x_tactile_interp, y_tactile_interp, z_tactile_interp, roll_tactile_interp, pitch_tactile_interp, yaw_tactile_interp = align_trajectory(
            x_tactile_interp, y_tactile_interp, z_tactile_interp, roll_tactile_interp, pitch_tactile_interp, yaw_tactile_interp,
            x_gt_interp, y_gt_interp, z_gt_interp, roll_gt_interp, pitch_gt_interp, yaw_gt_interp)

    if has_opti:
        x_opti_interp, y_opti_interp, z_opti_interp, roll_opti_interp, pitch_opti_interp, yaw_opti_interp = align_trajectory(
            x_opti_interp, y_opti_interp, z_opti_interp, roll_opti_interp, pitch_opti_interp, yaw_opti_interp,
            x_gt_interp, y_gt_interp, z_gt_interp, roll_gt_interp, pitch_gt_interp, yaw_gt_interp)


    # Compute errors (VIO vs Ground Truth)
    err_x      = np.abs(x_vio - x_gt_interp)
    err_y      = np.abs(y_vio - y_gt_interp)
    err_z      = np.abs(z_vio - z_gt_interp)
    err_pos_3d = np.sqrt(err_x**2 + err_y**2 + err_z**2)
    rmse_3d    = np.sqrt(np.mean(err_pos_3d**2))

    err_xy = np.sqrt(err_x**2 + err_y**2)
    rmse_xy = np.sqrt(np.mean(err_xy**2))
    rmse_z  = np.sqrt(np.mean(err_z**2))

    print(f"\n[*] --- VIO Tracking Errors (Global) ---")
    print(f"[*] Z-Axis Error:   RMSE = {rmse_z:.4f} m, Max = {np.max(err_z):.4f} m, Final = {err_z[-1]:.4f} m")
    print(f"[*] XY-Plane Error: RMSE = {rmse_xy:.4f} m, Max = {np.max(err_xy):.4f} m, Final = {err_xy[-1]:.4f} m")
    print(f"[*] 3D Error:       RMSE = {rmse_3d:.4f} m, Max = {np.max(err_pos_3d):.4f} m, Final = {err_pos_3d[-1]:.4f} m")

    if args.drop_time is not None:
        idx_drop = np.searchsorted(t_vio, args.drop_time)
        idx_start = 0
        if args.start_time is not None:
            idx_start = np.searchsorted(t_vio, args.start_time)
            
        idx_end = len(t_vio)
        if args.end_time is not None:
            idx_end = np.searchsorted(t_vio, args.end_time)
            
        if 0 < idx_drop < len(t_vio):
            rmse_z_before = np.sqrt(np.mean(err_z[idx_start:idx_drop]**2))
            rmse_z_after  = np.sqrt(np.mean(err_z[idx_drop:idx_end]**2))
            mean_z_before = np.mean(err_z[idx_start:idx_drop])
            mean_z_after  = np.mean(err_z[idx_drop:idx_end])
            improvement = ((rmse_z_before - rmse_z_after) / rmse_z_before) * 100.0
            
            start_str = f"t={args.start_time}s" if args.start_time is not None else "inizio"
            end_str = f"t={args.end_time}s" if args.end_time is not None else "fine"
            print(f"\n--- DROP ANALYSIS (OpenVINS Z-Axis) @ drop={args.drop_time}s, start={start_str}, end={end_str} ---")
            print(f"[*] RMSE Z prima del drop: {rmse_z_before:.4f} m  (Mean: {mean_z_before:.4f} m)")
            print(f"[*] RMSE Z dopo il drop:   {rmse_z_after:.4f} m  (Mean: {mean_z_after:.4f} m)")
            if improvement > 0:
                print(f"[*] L'errore su Z è MIGLIORATO del {improvement:.1f}%")
            else:
                print(f"[*] L'errore su Z è PEGGIORATO del {-improvement:.1f}%")
            print("--------------------------------------------------------------------------------------\n")
        else:
            print(f"\n[!] Attenzione: t={args.drop_time}s è fuori dal range di tempo disponibile (0 - {t_vio[-1]:.1f}s)")
    # Compute PX4/Tactile errors for plotting (if available)
    if has_px4:
        err_px4_3d = np.sqrt(
            (x_px4_interp - x_gt_interp)**2 +
            (y_px4_interp - y_gt_interp)**2 +
            (z_px4_interp - z_gt_interp)**2)

    if has_tactile:
        err_tactile_x = np.abs(x_tactile_interp - x_gt_interp)
        err_tactile_y = np.abs(y_tactile_interp - y_gt_interp)
        err_tactile_z = np.abs(z_tactile_interp - z_gt_interp)
        err_tactile_3d = np.sqrt(err_tactile_x**2 + err_tactile_y**2 + err_tactile_z**2)

    # Figure 5: VIO vs Tactile tracking errors (Default)
    fig_err_data = [
        {'vio': err_x, 'tactile': err_tactile_x if has_tactile else None},
        {'vio': err_y, 'tactile': err_tactile_y if has_tactile else None},
        {'vio': err_z, 'tactile': err_tactile_z if has_tactile else None},
        {'vio': err_pos_3d, 'tactile': err_tactile_3d if has_tactile else None},
    ]
    myPlot(t_vio, fig_err_data,
           ["Abs Error X [m]", "Abs Error Y [m]", "Abs Error Z [m]", "Total 3D Error [m]"],
           "Absolute Tracking Errors", ncols=2, use_tex=args.tex)

    # Figure 8: Teleop Commands and/or FSM States (Default)
    if has_teleop or has_fsm:
        fig_teleop_data = []
        labels = []
        
        if has_teleop:
            t_plot = t_teleop
            fig_teleop_data = [{'vio': vx_tel}, {'vio': vy_tel}, {'vio': wz_tel}]
            labels = ["Vx [m/s]", "Vy [m/s]", "Wz [rad/s]"]
        else:
            t_plot = t_fsm
            
        if has_fsm:
            # Interpolate FSM states onto t_plot per plottarli come linea
            fsm_states_interp = np.zeros_like(t_plot)
            for i, t in enumerate(t_plot):
                idx = np.searchsorted(t_fsm, t) - 1
                idx = max(0, min(idx, len(fsm_states)-1))
                fsm_states_interp[i] = fsm_states[idx]
            fig_teleop_data.append({'vio': fsm_states_interp})
            labels.append("FSM State (0=NAV, 1=STR, 2=SWP, 3=RET)")
            
        myPlot(t_plot, fig_teleop_data, labels,
               "Teleop Commanded Velocities & FSM State", ncols=2, use_tex=args.tex,
               t_fsm=t_fsm, fsm_states=fsm_states)

    # Figure 1: Position tracking
    fig_pos_data = [
        {'vio': x_vio, 'ref': x_gt_interp, 'px4': x_px4_interp if has_px4 else None, 'tactile': x_tactile_interp if has_tactile else None, 'opti': x_opti_interp if has_opti else None},
        {'vio': y_vio, 'ref': y_gt_interp, 'px4': y_px4_interp if has_px4 else None, 'tactile': y_tactile_interp if has_tactile else None, 'opti': y_opti_interp if has_opti else None},
        {'vio': z_vio, 'ref': z_gt_interp, 'px4': z_px4_interp if has_px4 else None, 'tactile': z_tactile_interp if has_tactile else None, 'opti': z_opti_interp if has_opti else None},
    ]
    myPlot(t_vio, fig_pos_data, ["X [m]", "Y [m]", "Z [m]"],
           "Position Tracking (ENU Frame)", ncols=3, use_tex=args.tex)

    if args.all_plots:

        # Figure 2: Orientation tracking
        fig_rpy_data = [
            {'vio': roll_vio,  'ref': roll_gt_interp,  'px4': roll_px4_interp  if has_px4 else None, 'tactile': roll_tactile_interp if has_tactile else None, 'opti': roll_opti_interp if has_opti else None},
            {'vio': pitch_vio, 'ref': pitch_gt_interp, 'px4': pitch_px4_interp if has_px4 else None, 'tactile': pitch_tactile_interp if has_tactile else None, 'opti': pitch_opti_interp if has_opti else None},
            {'vio': yaw_vio,   'ref': yaw_gt_interp,   'px4': yaw_px4_interp   if has_px4 else None, 'tactile': yaw_tactile_interp if has_tactile else None, 'opti': yaw_opti_interp if has_opti else None},
        ]
        myPlot(t_vio, fig_rpy_data, ["Roll [rad]", "Pitch [rad]", "Yaw [rad]"],
               "Orientation Tracking", ncols=3, use_tex=args.tex)

        # Figure 3: Eigenvalues
        if lam is not None and len(lam) > 0 and lam.shape[1] >= 4:
            t_lam = lam[:, 0] - start_time
            lx, ly, lz = lam[:, 1], lam[:, 2], lam[:, 3]
            fig_eig_data = [{'vio': lx}, {'vio': ly}, {'vio': lz}]
            myPlot(t_lam, fig_eig_data, ["Lambda X", "Lambda Y", "Lambda Z"],
                   "OpenVINS Degeneracy Eigenvalues", ncols=3, use_tex=args.tex,
                   t_health=t_health, health_states=health_states)

        # Figure 4: X-Y trajectory top view
        fig_xy = plt.figure(figsize=(10, 8))
        plt.plot(x_gt_interp, y_gt_interp, 'r--', label='Reference (OptiTrack/GT)', linewidth=2)
        plt.plot(x_vio,       y_vio,       'k-',  label='OpenVINS (VIO)', linewidth=2)
        if has_px4:
            plt.plot(x_px4_interp, y_px4_interp, 'b--', label='PX4 EKF2', linewidth=1.5)
        if has_tactile:
            plt.plot(x_tactile_interp, y_tactile_interp, 'g-.', label='PX4 VIO Echo', linewidth=1.5)
        if has_opti:
            plt.plot(x_opti_interp, y_opti_interp, 'm:', label='OptiTrack', linewidth=1.5)
            
        plt.plot(x_gt_interp[0], y_gt_interp[0], 'ro', label='Start Point', markersize=8)
        plt.title("X-Y Trajectory Comparison (Top View)")
        plt.xlabel("X [m]")
        plt.ylabel("Y [m]")
        plt.axis('equal')
        plt.grid(True)
        plt.legend()

        # Figure 6: PX4/Tactile vs VIO error comparison
        if has_px4 or has_tactile:
            fig_cmp, ax_cmp = plt.subplots(figsize=(10, 5))
            ax_cmp.plot(t_vio, err_pos_3d, 'k-',  label='OpenVINS 3D Error', linewidth=1.5)
            if has_px4:
                ax_cmp.plot(t_vio, err_px4_3d, 'b--', label='PX4 EKF2 3D Error', linewidth=1.5)
            if has_tactile:
                ax_cmp.plot(t_vio, err_tactile_3d, 'g-.', label='PX4 VIO Echo 3D Error', linewidth=1.5)
                
            ax_cmp.set_title("3D Tracking Error Comparison")
            ax_cmp.set_xlabel(r"$t$ [s]")
            ax_cmp.set_ylabel("3D Error [m]")
            ax_cmp.grid(True, linestyle='-', alpha=0.3)
            ax_cmp.legend(loc='best', fontsize='small')

        # Figure 7: Wrench Estimator
        if has_wrench:
            fig_force_data = [{'vio': fx}, {'vio': fy}, {'vio': fz}]
            myPlot(t_wrench, fig_force_data, ["Force X [N]", "Force Y [N]", "Force Z [N]"],
                   "External Force Estimator", ncols=3, use_tex=args.tex)
            
            fig_torque_data = [{'vio': tx}, {'vio': ty}, {'vio': tz}]
            myPlot(t_wrench, fig_torque_data, ["Torque X [Nm]", "Torque Y [Nm]", "Torque Z [Nm]"],
                   "External Torque Estimator", ncols=3, use_tex=args.tex)

    if args.save:
        for i in plt.get_fignums():
            plt.figure(i).savefig(f"plot_fig_{i}.png", dpi=300, bbox_inches='tight')
        print("[*] Plots saved as PNG files.")
    else:
        print("[*] Displaying plots...")
        plt.show()

if __name__ == "__main__":
    main()
