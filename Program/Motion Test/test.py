import math
import matplotlib.pyplot as plt

class Spline3D:
    def __init__(self):
        self.output_x = []
        self.output_y = []
        self.output_z = []

    def compute_parametric_t(self, points):
        t = [0]
        for i in range(1, len(points)):
            dx = points[i][0] - points[i - 1][0]
            dy = points[i][1] - points[i - 1][1]
            dz = points[i][2] - points[i - 1][2]
            dist = math.sqrt(dx**2 + dy**2 + dz**2)
            t.append(t[-1] + dist)
        return t

    def cubic_spline(self, x_points, y_points, x):
        n = len(x_points) - 1
        h = [x_points[i + 1] - x_points[i] for i in range(n)]

        A = [0] + [2 * (h[i - 1] + h[i]) for i in range(1, n)] + [0]
        B = [0] * (n + 1)
        for i in range(1, n):
            B[i] = (6 / h[i]) * (y_points[i + 1] - y_points[i]) - (6 / h[i - 1]) * (y_points[i] - y_points[i - 1])

        M = [0] * (n + 1)
        for i in range(1, n):
            temp = h[i - 1] / A[i]
            A[i + 1] -= temp * h[i - 1]
            B[i + 1] -= temp * B[i]
        for i in range(n - 1, 0, -1):
            M[i] = (B[i] - h[i] * M[i + 1]) / A[i]

        for i in range(n):
            if x_points[i] <= x <= x_points[i + 1]:
                a = (M[i + 1] - M[i]) / (6 * h[i])
                b = M[i] / 2
                c = (y_points[i + 1] - y_points[i]) / h[i] - (M[i + 1] + 2 * M[i]) * h[i] / 6
                d = y_points[i]
                dx = x - x_points[i]
                return a * dx**3 + b * dx**2 + c * dx + d
        return None

    def spline_interpolation(self, points, steps):
        t = self.compute_parametric_t(points)
        xs = [p[0] for p in points]
        ys = [p[1] for p in points]
        zs = [p[2] for p in points]

        t_min = t[0]
        t_max = t[-1]
        total_steps = steps * (len(points) - 1)

        self.output_x = [0] * (total_steps + 1)
        self.output_y = [0] * (total_steps + 1)
        self.output_z = [0] * (total_steps + 1)

        for i in range(total_steps + 1):
            ti = t_min + i * (t_max - t_min) / total_steps
            self.output_x[i] = self.cubic_spline(t, xs, ti)
            self.output_y[i] = self.cubic_spline(t, ys, ti)
            self.output_z[i] = self.cubic_spline(t, zs, ti)

# Example usage and 3D plotting
start_point = [(0, 50, 0), (0, 75, 10), (0, 100, 0)]

spline = Spline3D()
spline.spline_interpolation(start_point, steps=10)

# Plot using matplotlib 3D
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot original control points
px, py, pz = zip(*start_point)
ax.plot(px, py, pz, 'ro-', label='Control Points')

# Plot spline path
ax.plot(spline.output_x, spline.output_y, spline.output_z, 'b-', label='Spline Path')

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('3D Cubic Spline Interpolation')
ax.legend()
plt.tight_layout()
plt.show()
