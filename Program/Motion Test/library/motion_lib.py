# Welding Arm Manipulator 6-DOF - Alfonsus Giovanni
# Motion Mathematical Formula Library
# Last Update: April 2025

import math

MAX_STEP = 100
DOT_PATTERN = 0x01
LINEAR_PATTERN = 0x02
ZIGZAG_PATTERN = 0x03
WAVE_PATTERN = 0x04

class Motion:
    def __init__(self, input_step):
        self.step = input_step
        self.iteration = 1/input_step

        self.output_x = [0] * MAX_STEP
        self.output_y = [0] * MAX_STEP
        self.output_z = [0] * MAX_STEP

    def linear_interpolation(self, start_point, end_point, steps):
        x0, y0, z0 = start_point
        xf, yf, zf = end_point

        dx = (xf - x0) / steps
        dy = (yf - y0) / steps
        dz = (zf - z0) / steps

        for i in range(steps+1):
            x = x0 + i * dx
            y = y0 + i * dy
            z = z0 + i * dz

            self.output_x[i] = x
            self.output_y[i] = y
            self.output_z[i] = z

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

    def generate_dot_pattern(self):
        pass

    def generate_linear_pattern(self):
        pass

    def generate_zigzag_patter(self):
        pass

    def generate_wave_pattern(self):
        pass

linear = Motion(MAX_STEP)
spline = Motion(MAX_STEP)