import numpy as np

class ForwardKinematics:
    """
    Calculates the Forward Kinematics for a robot using Denavit-Hartenberg (DH) parameters.
    """

    def __init__(self):
        pass

    def dh_transform_matrix(self, theta, d, a, alpha):
        """
        Creates a transformation matrix for a single joint using standard DH parameters.
        
        Args:
            theta (float): Rotation around Z-axis (Joint Angle)
            d (float): Offset along Z-axis (Link Offset)
            a (float): Length along X-axis (Link Length)
            alpha (float): Twist angle around X-axis (Link Twist)
        
        Returns:
            np.array: 4x4 Homogeneous Transformation Matrix
        """
        # Precompute trig functions for efficiency
        ct = np.cos(theta)
        st = np.sin(theta)
        ca = np.cos(alpha)
        sa = np.sin(alpha)

        # Standard DH Transformation Matrix
        # This matrix represents the coordinate frame of joint i relative to joint i-1
        transform = np.array([
            [ct, -st * ca,  st * sa, a * ct],
            [st,  ct * ca, -ct * sa, a * st],
            [0,   sa,       ca,      d     ],
            [0,   0,        0,       1     ]
        ])
        
        return transform

    def compute_fk(self, dh_params):
        """
        Computes the final End-Effector pose by multiplying all link matrices.
        
        Args:
            dh_params (list of lists): A list where each row is [theta, d, a, alpha]
                                       for a specific joint.
        
        Returns:
            np.array: Final 4x4 Transformation Matrix (Position + Orientation)
        """
        # Start with an Identity Matrix (Base frame at 0,0,0)
        T_total = np.eye(4)
        
        print("--- Computing Forward Kinematics ---")
        
        for i, params in enumerate(dh_params):
            theta, d, a, alpha = params
            
            # Calculate matrix for this specific link
            T_i = self.dh_transform_matrix(theta, d, a, alpha)
            
            # Multiply total matrix by this link's matrix
            # Order matters: T_total = T_0 * T_1 * ... * T_n
            T_total = np.dot(T_total, T_i)
            
            # Optional: Print intermediate positions to show debug understanding
            position = T_total[:3, 3]
            print(f"Joint {i+1} Position: {np.round(position, 3)}")

        return T_total

# --- Demonstration with a 3-DOF Robot Arm ---
if __name__ == "__main__":
    fk_solver = ForwardKinematics()

    # Define a simple 3-DOF Planar Robot Arm
    # Format: [theta (angle), d (offset), a (length), alpha (twist)]
    # This robot has 3 links of length 1 meter, all moving in the flat plane.
    
    # Let's try to reach a point:
    # Joint 1: 90 degrees (points Y axis)
    # Joint 2: 0 degrees (straight out)
    # Joint 3: 0 degrees (straight out)
    
    dh_table = [
        [np.radians(90), 0, 1.0, 0], # Link 1
        [np.radians(0),  0, 1.0, 0], # Link 2
        [np.radians(0),  0, 1.0, 0]  # Link 3
    ]

    final_pose = fk_solver.compute_fk(dh_table)

    print("\nFinal End-Effector Pose Matrix:")
    print(np.round(final_pose, 3))
    
    print("\nFinal X, Y, Z Position:")
    print(np.round(final_pose[:3, 3], 3))