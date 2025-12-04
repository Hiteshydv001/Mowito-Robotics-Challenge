# Mowito Robotics Challenge

This repository contains solutions to two robotics mathematics challenges focusing on rotation representations and forward kinematics.

## Project Structure

```
Mowito/
├── Task 1/          # Euler-Quaternion Converter
├── Task 2/          # 4-Link Robot Forward Kinematics
└── README.md        # This file
```

---

## Task 1: Euler-Quaternion Converter

A Python implementation for converting between Euler Angles and Quaternions, with special handling for robotic edge cases like Gimbal Lock.

### Features
- Bidirectional conversion between Euler angles (roll, pitch, yaw) and quaternions
- Gimbal lock detection and handling
- 3D visualization for verification
- Uses Z-Y-X Intrinsic Tait-Bryan angles (standard in robotics/ROS)

### Files
- `rotation_converter.py`: Core conversion logic and mathematical implementation
- `visualize.py`: 3D visualization tool to verify rotations
- `forward_kinematics.py`: Forward kinematics implementation
- `requirements.txt`: Python dependencies

### Mathematical Approach

**Gimbal Lock Singularity:**
When converting quaternions back to Euler angles, if pitch reaches ±90°, the roll and yaw axes align, causing mathematical instability in `atan2`.

**Solution:**
- Check `sin(pitch)` value
- Clamp values near ±1 to prevent domain errors in `arcsin`
- Output warning when singularity is detected

### How to Run

```bash
cd "Task 1"
pip install -r requirements.txt
python rotation_converter.py
python visualize.py
```

---

## Task 2: 4-Link Perpendicular Robot Forward Kinematics

Computes forward kinematics for a 4-link robot where each joint axis is perpendicular to the previous one.

### Specifications
- **Link Length:** L = 1m (constant)
- **Constraint:** Each joint axis perpendicular to previous joint axis
- **Method:** Denavit-Hartenberg (DH) convention

### Mathematical Derivation

**DH Parameters Table:**

| Link | θ (Angle) | d (Offset) | a (Length) | α (Twist) | Description |
|------|-----------|------------|------------|-----------|-------------|
| 1    | j₁        | 0          | L          | 90°       | Yaw → Pitch twist |
| 2    | j₂        | 0          | L          | 90°       | Pitch → Yaw twist |
| 3    | j₃        | 0          | L          | 90°       | Yaw → Pitch twist |
| 4    | j₄        | 0          | L          | 0°        | Final link to end effector |

**Key Concept:**
- Frame assignment: Z_i is the rotation axis for Joint i+1
- Perpendicularity enforced by consistent twist angle α = 90°
- X_i aligned along link of length L

### Visualization
The implementation includes 3D plotting using Matplotlib:
- **Black lines:** Robot links
- **Blue arrows:** Z-axis (rotation axis) of each joint
- Visual verification that each blue arrow is perpendicular to the previous one

### Files
- `robot_arm_task2.py`: Main implementation with FK computation and visualization
- `requirements.txt`: Python dependencies

### How to Run

```bash
cd "Task 2"
pip install -r requirements.txt
python robot_arm_task2.py
```

---

## Dependencies

Both tasks require:
- Python 3.x
- NumPy
- Matplotlib

Install dependencies for each task using:
```bash
pip install -r requirements.txt
```

---

## Results

Both tasks include:
- ✅ Mathematical derivations
- ✅ Working Python implementations
- ✅ 3D visualizations
- ✅ Edge case handling

---

## Author

Developed as part of the Mowito Robotics Challenge

## License

This project is part of a technical assessment.
