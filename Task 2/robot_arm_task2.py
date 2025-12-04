import numpy as np
import matplotlib.pyplot as plt

class PerpendicularRobotArm:
    """
    A specific implementation for the 'Task 2' Robot.
    Constraint: 4 Links, each joint axis is perpendicular to the previous one.
    """

    def __init__(self, link_length=1.0):
        self.L = link_length
    
    def dh_matrix(self, theta, d, a, alpha):
        """
        Creates the Homogeneous Transformation Matrix for a single link
        using standard DH parameters.
        """
        ct, st = np.cos(theta), np.sin(theta)
        ca, sa = np.cos(alpha), np.sin(alpha)

        return np.array([
            [ct, -st*ca,  st*sa, a*ct],
            [st,  ct*ca, -ct*sa, a*st],
            [0,   sa,       ca,     d],
            [0,   0,        0,      1]
        ])

    def forward_kinematics(self, j1, j2, j3, j4):
        """
        Calculates the position of all joints and the end-effector.
        
        Args:
            j1, j2, j3, j4 (float): Joint angles in radians.
            
        Returns:
            frames (list): List of 4x4 matrices representing the pose of each joint.
            points (np.array): Nx3 array of joint positions (x,y,z) for plotting.
        """
        # --- DERIVING THE DH TABLE ---
        # The prompt states: "Axis of each joint is perpendicular to the previous joint."
        # This implies a 90 degree twist (alpha) between every joint axis.
        
        # Row Format: [theta, d, a, alpha]
        # Link 1: Rotate J1, Move L along X, Twist 90 deg (to make J2 axis perp to J1)
        # Link 2: Rotate J2, Move L along X, Twist 90 deg (to make J3 axis perp to J2)
        # Link 3: Rotate J3, Move L along X, Twist 90 deg (to make J4 axis perp to J3)
        # Link 4: Rotate J4, Move L along X, No Twist (End effector has no axis)
        
        dh_table = [
            [j1, 0, self.L, np.radians(90)],
            [j2, 0, self.L, np.radians(90)],
            [j3, 0, self.L, np.radians(90)],
            [j4, 0, self.L, 0] 
        ]

        # Initialize base frame (Identity)
        T_current = np.eye(4)
        
        # Store matrices and positions for visualization
        frames = [T_current]
        positions = [T_current[:3, 3]] # Start at 0,0,0

        print(f"--- Computing FK for Angles: {np.degrees([j1, j2, j3, j4]).round(1)} deg ---")

        for i, params in enumerate(dh_table):
            theta, d, a, alpha = params
            
            # Compute transform for this link
            T_link = self.dh_matrix(theta, d, a, alpha)
            
            # Update global transform
            T_current = T_current @ T_link
            
            frames.append(T_current)
            positions.append(T_current[:3, 3])
            
            print(f"Joint {i+1} -> {i+2} Position: {np.round(T_current[:3, 3], 3)}")

        return frames, np.array(positions)

    def visualize(self, j1, j2, j3, j4):
        """
        Plots the robot in 3D with Coordinate Frames to prove perpendicular axes.
        """
        frames, positions = self.forward_kinematics(j1, j2, j3, j4)
        
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')
        
        # Plot Links (Black Lines)
        ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], 
                '-o', color='black', linewidth=3, markersize=8, label='Robot Links')

        # Plot Coordinate Frames (RGB axes) at each joint
        # This proves the "Perpendicular" constraint visually
        scale = self.L * 0.4 # Scale arrows based on link length
        for i, T in enumerate(frames):
            origin = T[:3, 3]
            # Extract rotation vectors (columns of rotation matrix)
            x_axis = T[:3, 0]
            y_axis = T[:3, 1]
            z_axis = T[:3, 2] # This is the Axis of Rotation for the joint
            
            ax.quiver(*origin, *x_axis * scale, color='r') # X
            ax.quiver(*origin, *y_axis * scale, color='g') # Y
            ax.quiver(*origin, *z_axis * scale, color='b') # Z (Rotation Axis)

        # Labels and setting limits
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        ax.set_title('Task 2: 4-Link Perpendicular Robot\n(Blue Arrows = Joint Axes)')
        
        # Ensure aspect ratio is equal-ish
        limit = self.L * 4
        ax.set_xlim(-limit, limit)
        ax.set_ylim(-limit, limit)
        ax.set_zlim(0, limit)
        
        # Add legend explanations
        import matplotlib.patches as mpatches
        red_patch = mpatches.Patch(color='red', label='X Axis (Common Normal)')
        blue_patch = mpatches.Patch(color='blue', label='Z Axis (Rotation Axis)')
        plt.legend(handles=[red_patch, blue_patch])
        
        plt.show()

# --- Execution ---
if __name__ == "__main__":
    # Create robot with Link Length L=1m
    robot = PerpendicularRobotArm(link_length=1.0)

    # Test Case:
    # J1=0   -> Link 1 along X
    # J2=90  -> Link 2 goes UP (Z)
    # J3=0   -> Link 3 goes along Y (due to twist)
    # J4=-90 -> Link 4 moves accordingly
    
    j_angles = np.radians([0, 90, 0, -90])
    
    robot.visualize(*j_angles)