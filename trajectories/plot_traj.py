import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import math

# --------- SETTINGS ---------
# List of files to plot, along with their starting coordinates
# +X = 0 degrees, ccw from here
files = [
    # ("zigzag.csv", (0, 0, 0), "zigzag1"),
    # ("zigzag.csv", (4, 4, 180), "zigzag2"),
    ("lawnmower.csv", (0, 0, 0), "lawnmoer"),
    ("lawnmower.csv", (5, 4, 180), "lawnmoer2")
    # ("square.csv", (2, 2, 90), "square")
]
arrow_stride = 1      # Plot arrow every Nth point to avoid clutter
# Choose if we want to plot arrows for orientation
    # 0: None, 1: arrows, 2: quiver
plot_arrows = 1
linestyles = ['-', '--', '-.', ':']

# Sampling resolution for arcs (higher = smoother)
arc_resolution = 20

# --------- FUNCTIONS ---------

def load_and_generate_trajectory(filename, origin):
    df = pd.read_csv(filename, header=0).astype(float)

    trajectory = []

    x, y, theta = origin
    #theta = 0.0  # 
    theta = np.deg2rad(theta)

    trajectory.append((x, y, theta))

    for _, row in df.iterrows():
        type_code, linear_disp, angular_disp_deg = row
        angular_disp = np.deg2rad(angular_disp_deg)

        if type_code == 0:  # straight
            x += linear_disp * np.cos(theta)
            y += linear_disp * np.sin(theta)
            trajectory.append((x, y, theta))

        elif type_code == 1:  # in-place rotation
            theta += angular_disp

        elif type_code == 2:  # circular arc
            radius = linear_disp / angular_disp if angular_disp != 0 else 0
            angle_samples = np.linspace(0, angular_disp, arc_resolution)
            for delta_angle in angle_samples[1:]:
                cx = x - radius * np.sin(theta)
                cy = y + radius * np.cos(theta)
                x_new = cx + radius * np.sin(theta + delta_angle)
                y_new = cy - radius * np.cos(theta + delta_angle)
                trajectory.append((x_new, y_new, theta+delta_angle))
            theta += angular_disp

    return np.array(trajectory)

# --------- MAIN ---------

plt.figure(figsize=(8, 8))
colors = plt.cm.get_cmap('tab10', len(files))

for idx, (file, origin, label) in enumerate(files):
    traj = load_and_generate_trajectory(file, origin)
    xs, ys, thetas = traj[:, 0], traj[:, 1], traj[:, 2]
    offset = 0.05 * idx #small offset for overlaps
    plt.plot(xs+offset, ys+offset, label=label, color=colors(idx), linewidth=2.5, 
             linestyle=linestyles[idx % len(linestyles)], alpha=0.7)
    plt.scatter(*origin, color=colors(idx), marker='o')  # mark start point
    if plot_arrows == 0:
        continue
    elif plot_arrows == 1:
        # Draw arrows every Nth point
        for i in range(0, len(xs), arrow_stride):
            dx = 0.2 * np.cos(thetas[i])  # arrow length scaled for display
            dy = 0.2 * np.sin(thetas[i])
            plt.arrow(xs[i], ys[i], dx, dy, head_width=0.1, head_length=0.1, 
                    fc=colors(idx), ec=colors(idx))
    else:
        # Subsample for quiver
        xs_sub = xs[::arrow_stride]
        ys_sub = ys[::arrow_stride]
        thetas_sub = thetas[::arrow_stride]

        dx = 0.3 * np.cos(thetas_sub)  # adjust arrow length
        dy = 0.3 * np.sin(thetas_sub)

        plt.quiver(xs_sub, ys_sub, dx, dy, angles='xy', scale_units='xy', 
                   scale=1.0, color=colors(idx), width=0.005)

plt.title("2D Robot Trajectories")
plt.xlabel("X [m]")
plt.ylabel("Y [m]")
plt.grid(True)
plt.legend()
plt.axis('equal')
plt.show()

