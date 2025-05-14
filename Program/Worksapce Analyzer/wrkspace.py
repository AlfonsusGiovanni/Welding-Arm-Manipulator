import numpy as np
import matplotlib.pyplot as plt

# === Robot parameters ===
base_height = 191.0      # mm (offset from ground to joint‐2 axis)
L1 = 23.5               # mm (offset on X axis from J1 to J2)
L2 = 650.0              # mm (link from J2 to J3)
L3 = 640.0              # mm (link from J3 to tool)

# === Joint limits (degrees) ===
theta2_min, theta2_max = -60,  90   # Joint 2 range
theta3_min, theta3_max = -90,  60   # Joint 3 range

# === Sampling resolution ===
n = 300  # number of steps per joint

# Convert to radians and build sweep arrays
theta2 = np.linspace(np.deg2rad(theta2_min),
                     np.deg2rad(theta2_max), n)
theta3 = np.linspace(np.deg2rad(theta3_min),
                     np.deg2rad(theta3_max), n)

# === Compute reachable points ===
X, Z = [], []
for t2 in theta2:
    for t3 in theta3:
        # FK in X–Z plane:
        x = L2 * np.cos(t2) + L3 * np.cos(t2 + t3)
        z = base_height + L2 * np.sin(t2) + L3 * np.sin(t2 + t3)
        X.append(x)
        Z.append(z)

# Convert lists to arrays
X = np.array(X)
Z = np.array(Z)

# === Plot the workspace ===
plt.figure(figsize=(8, 6))
plt.scatter(X, Z, s=1, alpha=0.8)
plt.xlabel('X (mm)')
plt.ylabel('Z (mm)')
plt.title('2D Side-View Workspace of the Manipulator')
plt.axis('equal')
plt.grid(True)
plt.show()
