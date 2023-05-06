import cv2
import time
import hybo
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import griddata
import matplotlib.tri as tri

SERIAL_DEV = '/dev/ttyUSB0'

# simple xy-plot
def visualize_points1(points): 
    plt.clf()
    x_values = points[:, 0]
    y_values = points[:, 1]
    z_values = points[:, 2]

    limit = 7000
    #plt.scatter(points[:, 0], points[:, 1], c='b', marker='o', s=10)
    plt.scatter(x_values, y_values, c='b', marker='o', s=10)
    plt.xlim(-limit, limit)
    plt.ylim(-limit, limit)
    plt.gca().set_aspect('equal', adjustable='box')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('LIDAR Points Visualization')
    plt.grid(True)
    plt.pause(0.001)

# colored xy-plot with z as depth
def visualize_points2(points): 
    plt.clf()
    x_values = points[:, 0]
    y_values = points[:, 1]
    z_values = points[:, 2] / 100. # mm --> cm
    
    limit = 7000
    plt.scatter(x_values, y_values, c=z_values, marker='o', s=10, cmap='viridis')
    plt.xlim(-limit, limit)
    plt.ylim(-limit, limit)
    plt.gca().set_aspect('equal', adjustable='box')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('LIDAR Points Visualization')
    plt.grid(True)
    plt.colorbar(label='Z Value')
    plt.pause(0.001)

# contour plot
def visualize_points3(points):
    plt.clf()
    
    x_values = points[:, 0]
    y_values = points[:, 1]
    z_values = points[:, 2]

    triang = tri.Triangulation(x_values, y_values)
    plt.tricontourf(triang, z_values, cmap=plt.get_cmap("viridis"))
    # triang = tri.Triangulation(y_values, z_values)
    # plt.tricontourf(triang, x_values, cmap=plt.get_cmap("viridis"))
    limit = 7000
    plt.xlim(-limit, limit)
    plt.ylim(-limit, limit)
    plt.gca().set_aspect('equal', adjustable='box')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('LIDAR Points Visualization')
    plt.colorbar(label='Z')
    plt.grid(True)
    plt.pause(0.001)

# x,y,z line plot
def visualize_points4(points):
    plt.clf()
    
    limit=12000
    x_values = points[:, 0]
    y_values = points[:, 1]
    z_values = points[:, 2]
    index = np.arange(points.shape[0])

    plt.plot(index, x_values, label='X')
    plt.plot(index, y_values, label='Y')
    plt.plot(index, z_values, label='Z')

    plt.xlabel('Index')
    plt.ylabel('Value')
    plt.title('LIDAR Points Visualization')
    plt.legend()
    plt.grid(True)
    plt.xlim(0, points.shape[0])
    plt.ylim(-limit, limit)
    plt.pause(0.001)

# distance & angle on radial plot
def visualize_points5(points):
    plt.clf()

    x_values = points[:, 0]
    y_values = points[:, 1]
    z_values = points[:, 2]

    # Calculate distance and angle
    distance = np.sqrt(x_values**2 + y_values**2) # + z_values**2)
    angle = np.arctan2(y_values, x_values)
    # angle = np.arctan2(y_values, x_values)

    ax = plt.subplot(1, 1, 1, polar=True)
    ax.scatter(angle, distance, marker='o', s=10)
    ax.set_title('LIDAR Points Visualization')
    ax.set_rlabel_position(-22.5)
    ax.grid(True)

    plt.pause(0.05)

# clipped distance & angle on radial plot
def visualize_points5a(points):
    lim_min = 0.2 #-70000
    lim_max = 7.0
    plt.clf()

    x_values = points[:, 0]/1000.
    y_values = points[:, 1]/1000.
    z_values = points[:, 2]/1000.

    # Calculate distance and angle
    distance = np.sqrt(x_values**2 + y_values**2) # + z_values**2)
    angle = np.arctan2(y_values, x_values)

    # Create a boolean mask for distances within the specified limits
    mask = (distance >= lim_min) & (distance <= lim_max)

    # Apply the mask to angle and distance arrays
    filtered_angle = angle[mask]
    filtered_distance = distance[mask]

    ax = plt.subplot(1, 1, 1, polar=True)
    ax.scatter(filtered_angle, filtered_distance, marker='o', s=10)
    ax.set_title('LIDAR Points Visualization')
    ax.set_rlabel_position(-22.5)
    ax.grid(True)
    ax.set_ylim(0, lim_max)
    plt.pause(0.001)

# distance over index
def visualize_points6(points):
    plt.clf()

    x_values = points[:, 0]/1000.
    y_values = points[:, 1]/1000.
    z_values = points[:, 2]/1000.

    # Calculate distance
    distance = np.sqrt(z_values**2 + x_values**2) # + z_values**2)

    plt.plot(distance, marker='o', markersize=5)
    plt.xlabel('Index')
    plt.ylabel('Distance')
    plt.title('Distance vs. Index')
    plt.grid(True)
    plt.ylim(0.0, 7.0)

    plt.pause(0.05)

hybo = hybo.Lidar(SERIAL_DEV)
hybo.start()

# waiting for first frame
time.sleep(1)

plt.figure(figsize=(10, 10))

try:
    while True:
        raw_scan = hybo.get_latest_frame()
        time.sleep(0.01)

        if raw_scan is not None:
            sequence  = raw_scan["sequence"]
            time_peak = raw_scan["time_peak"]
            new_scan  = raw_scan["points"]

            visualize_points5a(np.array(new_scan))
            time.sleep(0.05)

except KeyboardInterrupt:
    print("Visualization stopped.")
finally:
    hybo.close()

