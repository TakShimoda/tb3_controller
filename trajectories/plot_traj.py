import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import math

import trajectories as tr

# --------- SETTINGS ---------
# List of files to plot, along with their starting coordinates
# +X = 0 degrees, ccw from here
traj_key = tr.file_keys[3]
files = tr.files[traj_key]

arrow_stride = 1      # Plot arrow every Nth point to avoid clutter
# Choose if we want to plot arrows for orientation
    # 0: None, 1: arrows, 2: quiver, 3: plot at waypoints
plot_arrows = 3
plot_waypoints = True
plot_time = True    # Plot time on waypoints?
linestyles = ['-', '--', '-.', ':']

# Sampling resolution for arcs (higher = smoother)
arc_resolution = 20
LINEAR_VELOCITY = 0.18  # m/s
ANGULAR_VELOCITY = 0.2  # rad/s

# --------- FUNCTIONS ---------

def load_and_generate_trajectory(filename, origin):
    df = pd.read_csv(filename, header=0).astype(float)

    trajectory = []
    cumulative_time = 0.0

    x, y, theta = origin
    #theta = 0.0  # 
    theta = np.deg2rad(theta)

    trajectory.append((x, y, theta, np.nan, cumulative_time))

    for _, row in df.iterrows():
        # type_code, linear_disp, angular_disp_deg = row
        type_code = float(row[0])
        linear_disp = float(row[1])
        angular_disp_deg = float(row[2])
        waypoint = row[3] if not pd.isna(row[3]) else np.nan
        angular_disp = np.deg2rad(angular_disp_deg)

        if type_code == 0:  # straight
            duration = abs(linear_disp) / LINEAR_VELOCITY
            cumulative_time += duration
            if linear_disp == 0:
                cumulative_time += 2.0
            x += linear_disp * np.cos(theta)
            y += linear_disp * np.sin(theta)
            #trajectory.append((x, y, theta))
            trajectory.append((x, y, theta, waypoint, cumulative_time))

        elif type_code == 1:  # in-place rotation
            duration = abs(angular_disp) / ANGULAR_VELOCITY
            cumulative_time += duration
            theta += angular_disp

        elif type_code == 2:  # circular arc
            radius = linear_disp / angular_disp if angular_disp != 0 else 0
            duration = abs(angular_disp) * radius / LINEAR_VELOCITY
            angle_samples = np.linspace(0, angular_disp, arc_resolution)
            for delta_angle in angle_samples[1:]:
                cx = x - radius * np.sin(theta)
                cy = y + radius * np.cos(theta)
                x_new = cx + radius * np.sin(theta + delta_angle)
                y_new = cy - radius * np.cos(theta + delta_angle)
                traj_time = cumulative_time + (abs(delta_angle) * radius / LINEAR_VELOCITY)
                #trajectory.append((x_new, y_new, theta + delta_angle, np.nan))
                trajectory.append((x_new, y_new, theta + delta_angle, np.nan, traj_time))
            cumulative_time += duration
            theta += angular_disp

    return np.array(trajectory, dtype=object)

# --------- MAIN ---------

plt.figure(figsize=(8, 8))
# colors = plt.cm.get_cmap('tab10', len(files))
colors = [
    'blue', 'red', 'green', 'orange', 'purple', 'brown', 'pink',
    'gray', 'olive', 'cyan', '#1f77b4', '#ff7f0e', '#2ca02c',
    "#d62728", '#9467bd'
]

for idx, (file, origin, label) in enumerate(files):
    traj = load_and_generate_trajectory(file, origin)
    #xs, ys, thetas = traj[:, 0], traj[:, 1], traj[:, 2]
    # xs, ys, thetas, waypoints = traj[:, 0].astype(float), traj[:, 1].astype(float), \
    #     traj[:, 2].astype(float), traj[:, 3]
    xs, ys, thetas, waypoints, times = traj[:, 0].astype(float), traj[:, 1].astype(float), \
        traj[:, 2].astype(float), traj[:, 3], traj[:, 4].astype(float)
    offset = 0.04 * idx #small offset for overlaps
    plt.plot(xs+offset, ys+offset, label=label, color=colors[idx % len(colors)], linewidth=2.5, 
             linestyle=linestyles[idx % len(linestyles)], alpha=0.7)
    plt.scatter(*origin, color=colors[idx % len(colors)], marker='o')  # mark start point

    # Plot waypoint markers if they exist
    if plot_waypoints:
        # Plot the starting point
        plt.scatter(origin[0], origin[1], color='black', 
                marker='o', s=40, zorder=5)
        plt.text(origin[0] + offset + 0.1, origin[1] + offset + 0.1, '0', 
                fontsize=12, color=colors[idx % len(colors)])
        if plot_arrows == 3:
            dx = 0.3 * np.cos(np.deg2rad(origin[2]))
            dy = 0.3 * np.sin(np.deg2rad(origin[2]))
            plt.arrow(origin[0] + offset, origin[1] + offset, dx, dy, head_width=0.15, 
                head_length=0.15, fc=colors[idx % len(colors)], ec='black', zorder=6)
        for i in range(len(xs)):
            if not pd.isna(waypoints[i]):
                time_label = f"{times[i]:.1f}s"
                wp_num = int(waypoints[i])
                if plot_time:
                    s_label = time_label 
                else:
                    s_label = str(wp_num)
                plt.scatter(xs[i] + offset, ys[i] + offset, color='black', 
                        marker='o', s=40, zorder=5)
                plt.text(xs[i] + offset + 0.1, ys[i] + offset + 0.1, s_label, 
                        fontsize=12, color=colors[idx % len(colors)])
                # Add arrow for orientation at this waypoint
                dx = 0.3 * np.cos(thetas[i])
                dy = 0.3 * np.sin(thetas[i])
                if plot_arrows == 3:
                    plt.arrow(xs[i] + offset, ys[i] + offset, dx, dy, 
                        head_width=0.15, head_length=0.15, fc=colors[idx % len(colors)], ec='black', zorder=6)

    if plot_arrows == 0:
        continue
    elif plot_arrows == 1:
        # Draw arrows every Nth point
        for i in range(0, len(xs), arrow_stride):
            dx = 0.2 * np.cos(thetas[i])  # arrow length scaled for display
            dy = 0.2 * np.sin(thetas[i])
            plt.arrow(xs[i], ys[i], dx, dy, head_width=0.1, head_length=0.1, 
                    fc=colors[idx], ec=colors[idx])
    elif plot_arrows == 2:
        # Subsample for quiver
        xs_sub = xs[::arrow_stride]
        ys_sub = ys[::arrow_stride]
        thetas_sub = thetas[::arrow_stride]

        dx = 0.3 * np.cos(thetas_sub)  # adjust arrow length
        dy = 0.3 * np.sin(thetas_sub)

        plt.quiver(xs_sub, ys_sub, dx, dy, angles='xy', scale_units='xy', 
                   scale=1.0, color=colors(idx), width=0.005)
    

plt.title(traj_key)
plt.xlabel("X [m]")
plt.ylabel("Y [m]")
plt.grid(True)
plt.legend()
plt.axis('equal')
plt.show()

