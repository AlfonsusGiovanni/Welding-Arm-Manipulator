import matplotlib.pyplot as plt

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

# Example data points
x_points = [0, 1, 2]  # x coordinates
y_points = [0, 1, 0]  # y coordinates

# Generate waypoints
points_num = len(x_points)
iteration = 0.1
point_accuracy = int(points_num / iteration)

x_values = [0] * (point_accuracy+1)

for i in range(point_accuracy+1):
    x_values[i] = i * iteration

waypoints = [(x, cubic_spline(x_points, y_points, x)) for x in x_values]

# Separate x and y for plotting
waypoint_x = [wp[0] for wp in waypoints]
waypoint_y = [wp[1] for wp in waypoints]

# Plot the original data points
plt.scatter(x_points, y_points, color='red', label='Original Points')

# Plot the interpolated waypoints
plt.plot(waypoint_x, waypoint_y, color='blue', label='Spline Waypoints', linewidth=2)

# Highlight waypoints
plt.scatter(waypoint_x, waypoint_y, color='green', s=10, label='Waypoints')

# Add labels, grid, and legend
plt.title("Cubic Spline Interpolation with Waypoints")
plt.xlabel("x")
plt.ylabel("y")
plt.legend()
plt.grid(True)

# Show the plot
plt.show()
