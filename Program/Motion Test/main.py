# Welding Arm Manipulator 6-DOF - Alfonsus Giovanni
# Motion Test Program
# Last Update: April 2025

from library import motion_lib as motion
import matplotlib.pyplot as plt

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Coordinate Test Position
start_point = [(0, 50, 0), (0, 75, 0), (0, 100, 0)] # Start Point 1, 2, 3
end_point = [(0, 65, 0), (0, 90, 0), (0, 115, 0)] # End Point 1, 2, 3
welding_point_data = []
point_pattern = [motion.LINEAR_PATTERN, motion.LINEAR_PATTERN, motion.LINEAR_PATTERN] # Point Pattern

ax.set_xlim(-10, 20)
ax.set_ylim(50, 115)
ax.set_zlim(-10, 20)

# Calculate Total Point
point_count = len(start_point)

for i in range(point_count):
    if point_pattern[i] == motion.DOT_PATTERN:
        pass
    elif point_pattern[i] == motion.LINEAR_PATTERN:
        pass
    elif point_pattern[i] == motion.ZIGZAG_PATTERN:
        pass
    elif point_pattern[i] == motion.WAVE_PATTERN:
        pass

# Calculate Linear Interpolation
motion.linear.linear_interpolation(start_point[0], end_point[0], 10) 
ax.plot(motion.linear.output_x, motion.linear.output_y, motion.linear.output_z, label='Welding Path 1')
ax.scatter(motion.linear.output_x, motion.linear.output_y, motion.linear.output_z, color='blue')

# Calculate Spline Interpolation For Position Change (1-2)
test_point = [end_point[0], (5, 70, 5), start_point[1]]
motion.spline.spline_interpolation(test_point, 10)
ax.plot(motion.spline.output_x, motion.spline.output_y, motion.spline.output_z, label='Transition Path')
ax.scatter(motion.spline.output_x, motion.spline.output_y, motion.spline.output_z, color='orange')

# Calculate Linear Interpolation For Position 2
motion.linear.linear_interpolation(start_point[1], end_point[1], 10) 
ax.plot(motion.linear.output_x, motion.linear.output_y, motion.linear.output_z, label='Welding Path 2')
ax.scatter(motion.linear.output_x, motion.linear.output_y, motion.linear.output_z, color='green')

# Calculate Spline Interpolation For Position Change (2-3)
test_point = [end_point[1], (5, 95, 5), start_point[2]]
motion.spline.spline_interpolation(test_point, 10)
ax.plot(motion.spline.output_x, motion.spline.output_y, motion.spline.output_z, label='Transition Path')
ax.scatter(motion.spline.output_x, motion.spline.output_y, motion.spline.output_z, color='red')

# Calculate Linear Interpolation For Position 3
motion.linear.linear_interpolation(start_point[2], end_point[2], 10) 
ax.plot(motion.linear.output_x, motion.linear.output_y, motion.linear.output_z, label='Welding Path 3')
ax.scatter(motion.linear.output_x, motion.linear.output_y, motion.linear.output_z, color='purple')

ax.set_xlabel('X Axis')
ax.set_ylabel('Y Axis')
ax.set_zlabel('Z Axis')
ax.legend()

plt.show()