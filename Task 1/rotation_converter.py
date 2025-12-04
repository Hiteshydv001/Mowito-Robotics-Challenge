import numpy as np
import math

class RotationConverter:
    """
    A class to convert between Euler Angles and Quaternions.
    Convention: Intrinsic Z-Y-X (Yaw, Pitch, Roll).
    """

    @staticmethod
    def euler_to_quaternion(roll, pitch, yaw):
        """
        Convert Euler angles to Quaternion.
        
        Args:
            roll (float): Rotation around X-axis (in radians)
            pitch (float): Rotation around Y-axis (in radians)
            yaw (float): Rotation around Z-axis (in radians)
            
        Returns:
            np.array: Quaternion [w, x, y, z]
        
        Math:
            q = cos(a/2) + sin(a/2) * axis
            We combine rotations: q = q_yaw * q_pitch * q_roll
        """
        # Half angles are used in quaternion formulas
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)

        # Standard conversion formula for Z-Y-X sequence
        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy

        return np.array([w, x, y, z])

    @staticmethod
    def quaternion_to_euler(w, x, y, z):
        """
        Convert Quaternion to Euler angles (Roll, Pitch, Yaw).
        
        Args:
            w, x, y, z (float): The quaternion components.
            
        Returns:
            tuple: (roll, pitch, yaw) in radians.
        """
        # Normalize the quaternion first to avoid math errors
        norm = math.sqrt(w*w + x*x + y*y + z*z)
        w, x, y, z = w/norm, x/norm, y/norm, z/norm

        # Roll (X-axis rotation)
        # atan2(2(wx + yz), 1 - 2(x^2 + y^2))
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        # Pitch (Y-axis rotation)
        # This is where the EDGE CASE lives (Gimbal Lock)
        # We calculate sin(pitch). Ideally, this is between -1 and 1.
        sinp = 2 * (w * y - z * x)

        # EDGE CASE HANDLING:
        # If we are at +90 or -90 degrees (North/South pole), 
        # sinp approaches 1 or -1. Numerical errors can make it > 1.
        if abs(sinp) >= 1:
            # Clamp the value to range [-1, 1] to avoid nan in arcsin
            # If pitch is 90 degrees, we are in Gimbal Lock.
            pitch = np.copysign(np.pi / 2, sinp)
            print("Warning: Gimbal Lock detected!")
        else:
            pitch = np.arcsin(sinp)

        # Yaw (Z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

# --- Demonstration Block ---
if __name__ == "__main__":
    converter = RotationConverter()

    # Test 1: Standard Rotation (45 degrees on all axes)
    r, p, y = np.radians(45), np.radians(45), np.radians(45)
    print(f"Original Input (deg): 45, 45, 45")
    
    q = converter.euler_to_quaternion(r, p, y)
    print(f"Quaternion: {np.round(q, 4)}")
    
    new_r, new_p, new_y = converter.quaternion_to_euler(q[0], q[1], q[2], q[3])
    print(f"Recovered Euler (deg): {np.degrees([new_r, new_p, new_y])}\n")

    # Test 2: Edge Case (Gimbal Lock - Pitch is 90 degrees)
    print("--- Testing Gimbal Lock (Pitch = 90 deg) ---")
    r, p, y = 0, np.radians(90), 0
    q = converter.euler_to_quaternion(r, p, y)
    new_r, new_p, new_y = converter.quaternion_to_euler(q[0], q[1], q[2], q[3])
    print(f"Recovered Pitch (deg): {np.degrees(new_p)}")