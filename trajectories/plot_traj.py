import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import math

# --------- SETTINGS ---------
# List of files to plot, along with their starting coordinates
# +X = 0 degrees, ccw from here
files = {
    "I_shape.csv": (2, 0, 90),
    "circle_1_cw.csv": (0, 0, 0),
    "square.csv": (0, 0, 180),
    "triangle.csv": (3, 3, 0)
}

# Sampling resolution for arcs (higher = smoother)
arc_resolution = 20

# --------- FUNCTIONS ---------

def load_and_generate_trajectory(filename, origin):
    df = pd.read_csv(filename, header=0).astype(float)

    trajectory = []

    x, y, theta = origin
    #theta = 0.0  # 
    theta = np.deg2rad(theta)

    trajectory.append((x, y))

    for _, row in df.iterrows():
        type_code, linear_disp, angular_disp_deg = row
        angular_disp = np.deg2rad(angular_disp_deg)

        if type_code == 0:  # straight
            x += linear_disp * np.cos(theta)
            y += linear_disp * np.sin(theta)
            trajectory.append((x, y))

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
                trajectory.append((x_new, y_new))
            theta += angular_disp

    return np.array(trajectory)

# --------- MAIN ---------

plt.figure(figsize=(8, 8))
colors = plt.cm.get_cmap('tab10', len(files))

for idx, (file, origin) in enumerate(files.items()):
    traj = load_and_generate_trajectory(file, origin)
    plt.plot(traj[:, 0], traj[:, 1], label=file, color=colors(idx))
    plt.scatter(*origin, color=colors(idx), marker='o')  # mark start point

plt.title("2D Robot Trajectories")
plt.xlabel("X [m]")
plt.ylabel("Y [m]")
plt.grid(True)
plt.legend()
plt.axis('equal')
plt.show()

