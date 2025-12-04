import matplotlib.pyplot as plt
import numpy as np
from rotation_converter import RotationConverter

def plot_rotation(roll, pitch, yaw):
    converter = RotationConverter()
    q = converter.euler_to_quaternion(roll, pitch, yaw)
    w, x, y, z = q

    # Rotation Matrix from Quaternion (Standard Math)
    # This transforms a vector by the quaternion rotation
    R = np.array([
        [1 - 2*y**2 - 2*z**2,  2*x*y - 2*z*w,      2*x*z + 2*y*w],
        [2*x*y + 2*z*w,        1 - 2*x**2 - 2*z**2, 2*y*z - 2*x*w],
        [2*x*z - 2*y*w,        2*y*z + 2*x*w,      1 - 2*x**2 - 2*y**2]
    ])

    # Original Axes (Identity)
    origin = np.array([0, 0, 0])
    axes = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]]) # X, Y, Z

    # Rotate the axes
    new_axes = R @ axes.T  # Matrix multiplication

    # Plotting
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlim([-1, 1]); ax.set_ylim([-1, 1]); ax.set_zlim([-1, 1])
    ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')

    colors = ['r', 'g', 'b'] # X=Red, Y=Green, Z=Blue
    labels = ['X_new', 'Y_new', 'Z_new']

    for i in range(3):
        ax.quiver(0, 0, 0, new_axes[0, i], new_axes[1, i], new_axes[2, i], color=colors[i], label=labels[i])

    plt.title(f"Rotation: R={np.degrees(roll):.1f}, P={np.degrees(pitch):.1f}, Y={np.degrees(yaw):.1f}")
    plt.legend()
    plt.show()

if __name__ == "__main__":
    # Change these values to see different rotations
    plot_rotation(np.radians(45), np.radians(0), np.radians(45))