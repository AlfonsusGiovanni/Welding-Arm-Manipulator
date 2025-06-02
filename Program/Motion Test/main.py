# Welding Arm Manipulator 6-DOF - Alfonsus Giovanni
# Motion Test Program
# Last Update: April 2025

from library import motion_lib as motion
import matplotlib.pyplot as plt

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Coordinate Test Position
start_point = [(0, 50, 0), (15, 75, 5), (0, 110, 15)] # Start Point 1, 2, 3
end_point = [(0, 65, 0), (15, 90, 5), (0, 110, 5)] # End Point 1, 2, 3

ax.set_xlim(-10, 20)
ax.set_ylim(50, 120)
ax.set_zlim(-10, 20)

# Calculate ZigZag Interpolation
motion.zigzag.generate_zigzag_pattern(start_point[0], end_point[0], 20, motion.AXIS_X_OFFSET, 0.5) 
ax.plot(motion.zigzag.output_x, motion.zigzag.output_y, motion.zigzag.output_z, color='blue', label='Welding Path 1')
ax.scatter(motion.zigzag.output_x, motion.zigzag.output_y, motion.zigzag.output_z, color='blue')
print(motion.zigzag.output_y)

# Calculate Spline Interpolation For Position Change (1-2)
mid_point = motion.spline.generate_spline_middle_point(end_point[0], start_point[1], motion.AXIS_Z_OFFSET, 10)
test_point = [end_point[0], mid_point, start_point[1]]
motion.spline.spline_interpolation(test_point, 10)
ax.plot(motion.spline.output_x, motion.spline.output_y, motion.spline.output_z, color='orange', label='Transition Path')
ax.scatter(motion.spline.output_x, motion.spline.output_y, motion.spline.output_z, color='orange')
print(motion.spline.output_y)

# Calculate ZigZag Interpolation For Position 2
motion.zigzag.generate_zigzag_pattern(start_point[1], end_point[1], 20, motion.AXIS_X_OFFSET, 0.5) 
ax.plot(motion.zigzag.output_x, motion.zigzag.output_y, motion.zigzag.output_z, color='green', label='Welding Path 2')
ax.scatter(motion.zigzag.output_x, motion.zigzag.output_y, motion.zigzag.output_z, color='green')

# Calculate Spline Interpolation For Position Change (2-3)
mid_point = motion.spline.generate_spline_middle_point(end_point[1], start_point[2], motion.AXIS_Y_OFFSET, 10)
test_point = [end_point[1], mid_point, start_point[2]]
motion.spline.spline_interpolation(test_point, 10)
ax.plot(motion.spline.output_x, motion.spline.output_y, motion.spline.output_z,  color='red', label='Transition Path')
ax.scatter(motion.spline.output_x, motion.spline.output_y, motion.spline.output_z, color='red')

# Calculate Linear Interpolation For Position 3
motion.linear.linear_interpolation(start_point[2], end_point[2], 10) 
ax.plot(motion.linear.output_x, motion.linear.output_y, motion.linear.output_z, color='purple', label='Welding Path 3')
ax.scatter(motion.linear.output_x, motion.linear.output_y, motion.linear.output_z, color='purple')

ax.set_xlabel('X Axis')
ax.set_ylabel('Y Axis')
ax.set_zlabel('Z Axis')
ax.legend()

plt.show()