import turtle
import time
import math

# Turtle setup
turtle.speed(0)
turtle.pensize(2)
turtle.hideturtle()

SCALE = 2

def draw_point(point, color="red"):
    x, y = point
    turtle.penup()
    turtle.goto(x * SCALE, y * SCALE)
    turtle.pendown()
    turtle.dot(5, color)

def move_to(point):
    x, y = point
    turtle.penup()
    turtle.goto(x * SCALE, y * SCALE)
    turtle.pendown()

def linear_interpolation(P0, Pf, duration, steps, color="blue"):
    x0, y0 = P0
    xf, yf = Pf
    dx = (xf - x0) / steps
    dy = (yf - y0) / steps

    move_to(P0)
    turtle.pencolor(color)
    for i in range(steps + 1):
        x = x0 + i * dx
        y = y0 + i * dy
        turtle.goto(x * SCALE, y * SCALE)
        time.sleep(duration / steps)

def compute_parametric_t(points):
    t = [0]
    for i in range(1, len(points)):
        dx = points[i][0] - points[i-1][0]
        dy = points[i][1] - points[i-1][1]
        dist = math.sqrt(dx**2 + dy**2)
        t.append(t[-1] + dist)
    return t

def cubic_spline(x_points, y_points, x):
    n = len(x_points) - 1  # Number of intervals
    h = [x_points[i + 1] - x_points[i] for i in range(n)]
    
    # Step 1: Solve for the second derivatives (M)
    A = [0] + [2 * (h[i - 1] + h[i]) for i in range(1, n)] + [0]
    B = [0] * (n + 1)
    for i in range(1, n):
        B[i] = (6 / h[i]) * (y_points[i + 1] - y_points[i]) - (6 / h[i - 1]) * (y_points[i] - y_points[i - 1])
    
    # Tridiagonal matrix algorithm (Thomas algorithm)
    M = [0] * (n + 1)  # Second derivatives
    for i in range(1, n):
        A[i + 1] -= (h[i] / A[i]) * h[i]
        B[i + 1] -= (h[i] / A[i]) * B[i]
        print(A[i], B[i])
    for i in range(n - 1, 0, -1):
        M[i] = (B[i] - h[i] * M[i + 1]) / A[i]
    
    # Step 2: Find the spline interval and calculate the interpolated value
    for i in range(n):
        if x_points[i] <= x <= x_points[i + 1]:
            a = (M[i + 1] - M[i]) / (6 * h[i])
            b = M[i] / 2
            c = (y_points[i + 1] - y_points[i]) / h[i] - (M[i + 1] + 2 * M[i]) * h[i] / 6
            d = y_points[i]
            dx = x - x_points[i]
            return a * dx**3 + b * dx**2 + c * dx + d

def parametric_cubic_spline(points, duration, steps, color="green"):
    t = compute_parametric_t(points)
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]

    move_to((xs[0], ys[0]))
    turtle.pencolor(color)

    t_min = t[0]
    t_max = t[-1]
    total_steps = steps * (len(points) - 1)

    for i in range(total_steps + 1):
        ti = t_min + i * (t_max - t_min) / total_steps
        print("t1:", ti)
        print(" ")
        x = cubic_spline(t, xs, ti)
        y = cubic_spline(t, ys, ti)
        turtle.goto(x * SCALE, y * SCALE)
        time.sleep(duration / total_steps)

# Auto-generate control points
def generate_control_points(P0, P3, y):
    x1 = 0.75 * P0[0] + 0.25 * P3[0]
    x2 = 0.25 * P0[0] + 0.75 * P3[0]
    P1 = (x1, y)
    P2 = (x2, y)
    return P1, P2

# Define key positions
home = (-100, 0)
start_point = [(-100, 50), (0, 50), (100, 50)]
end_point = [(-75, 50), (25, 50), (125, 50)]
total_point = len(start_point)

# Draw key points
for pt in start_point:
    draw_point(pt, "red")

for pt in end_point:
    draw_point(pt, "blue")

# Simulate
linear_interpolation(home, start_point[0], 2, 25, "blue") # Home -> First Welding Point

for i in range (total_point-1):
    linear_interpolation(start_point[i], end_point[i], 2, 50, "black")
    P0 = end_point[i]
    P3 = start_point[i+1]
    P1, P2 = generate_control_points(P0, P3, 40)
    parametric_cubic_spline([P0, P1, P2, P3], 1, 25, color="green") # To The Next Point
        
linear_interpolation(start_point[2], end_point[2], 2, 50, "black") # Start Point - End Point 3

turtle.done()
