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

def compute_cubic_spline(t, s):
    n = len(t)
    h = [t[i+1] - t[i] for i in range(n - 1)]
    a = s[:]
    alpha = [0] * n
    for i in range(1, n - 1):
        alpha[i] = (3 / h[i]) * (a[i+1] - a[i]) - (3 / h[i-1]) * (a[i] - a[i-1])

    c = [0] * n
    lu = [1] * n
    mu = [0] * n
    z = [0] * n

    for i in range(1, n - 1):
        lu[i] = 2 * (t[i+1] - t[i-1]) - h[i-1] * mu[i-1]
        mu[i] = h[i] / lu[i]
        z[i] = (alpha[i] - h[i-1] * z[i-1]) / lu[i]

    b = [0] * (n - 1)
    d = [0] * (n - 1)
    c[n - 1] = 0

    for j in range(n - 2, -1, -1):
        c[j] = z[j] - mu[j] * c[j+1]
        b[j] = (a[j+1] - a[j]) / h[j] - h[j] * (c[j+1] + 2 * c[j]) / 3
        d[j] = (c[j+1] - c[j]) / (3 * h[j])

    return a[:-1], b, c[:-1], d, t[:-1], h

def parametric_cubic_spline(points, duration, steps, color="green"):
    t = compute_parametric_t(points)
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]

    ax, bx, cx, dx, tx, h = compute_cubic_spline(t, xs)
    ay, by, cy, dy, ty, _ = compute_cubic_spline(t, ys)

    move_to((xs[0], ys[0]))
    turtle.pencolor(color)

    for i in range(len(h)):
        for j in range(steps + 1):
            tau = h[i] * j / steps
            x = ax[i] + bx[i] * tau + cx[i] * tau**2 + dx[i] * tau**3
            y = ay[i] + by[i] * tau + cy[i] * tau**2 + dy[i] * tau**3
            turtle.goto(x * SCALE, y * SCALE)
            time.sleep(duration/steps)

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
