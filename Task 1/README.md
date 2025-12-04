# Robotics Maths Task 1: Euler-Quaternion Converter

This repository contains a Python implementation for converting between Euler Angles and Quaternions, specifically designed to handle robotic edge cases like Gimbal Lock.

## Files
- `rotation_converter.py`: The core class containing mathematical logic.
- `visualize.py`: A 3D visualization tool to verify the rotations.

## Math Explanation
The converter uses the **Z-Y-X Intrinsic Tait-Bryan angles**, which is the standard convention in Robotics (e.g., ROS).

### The Singularity (Gimbal Lock)
When converting a Quaternion back to Euler angles, if the Pitch is +/- 90 degrees, the Roll and Yaw axes align. This causes a mathematical singularity where `atan2` becomes unstable.
- **My Solution:** I check the value of `sin(pitch)`. If it is close to 1 or -1, I clamp the value to avoid domain errors in `arcsin` and output a warning.

## How to Run
1. Install dependencies:
   ```bash
   pip install -r requirements.txt