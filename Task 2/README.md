## Task 2: 4-Link "Perpendicular" Robot

**Objective:** Compute Forward Kinematics for a 4-link robot where each joint axis is perpendicular to the previous one. Link length $L = 1m$.

**Mathematical Derivation (DH Parameters):**
To satisfy the constraint that "the axis of each joint is perpendicular to the previous joint," I utilized the **Denavit-Hartenberg (DH)** convention with a consistent twist angle $\alpha = 90^\circ$.

1.  **Frame Assignment:**
    *   $Z_i$ is the axis of rotation for Joint $i+1$.
    *   Since $Z_{i}$ must be perpendicular to $Z_{i-1}$, the twist angle $\alpha$ must be $\pm 90^\circ$.
    *   $X_i$ is along the link of length $L$.

2.  **The DH Table:**
    | Link | $\theta$ (Angle) | $d$ (Offset) | $a$ (Length) | $\alpha$ (Twist) | Explanation |
    |---|---|---|---|---|---|
    | 1 | $j_1$ | 0 | $L$ | $90^\circ$ | Rotates Yaw -> Twist to Pitch |
    | 2 | $j_2$ | 0 | $L$ | $90^\circ$ | Rotates Pitch -> Twist to Yaw (Orthogonal) |
    | 3 | $j_3$ | 0 | $L$ | $90^\circ$ | Rotates Yaw -> Twist to Pitch |
    | 4 | $j_4$ | 0 | $L$ | $0^\circ$ | Final link to End Effector |

**Visualization:**
The script `robot_arm_task2.py` includes a 3D plot using Matplotlib.
- **Black Lines:** The robot links.
- **Blue Arrows:** The $Z$-axis (Rotation Axis) of each joint. You can visually verify that every Blue arrow is perpendicular to the previous Blue arrow.