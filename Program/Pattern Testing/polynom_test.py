import numpy as np
import matplotlib.pyplot as plt

def lagrange_interpolation(x_points, y_points, x_eval):
    """
    Perform polynomial interpolation using Lagrange Interpolation.
    
    Parameters:
    - x_points: Known x data points (list or numpy array).
    - y_points: Corresponding y data points (list or numpy array).
    - x_eval: Points to evaluate the interpolated polynomial.

    Returns:
    - Interpolated y values at x_eval.
    """
    def lagrange_basis(x, i, x_points):
        """
        Compute the i-th Lagrange basis polynomial at x.
        """
        basis = 1
        for j in range(len(x_points)):
            if j != i:
                basis *= (x - x_points[j]) / (x_points[i] - x_points[j])
        return basis
    
    y_eval = []
    for x in x_eval:
        y = 0
        for i in range(len(x_points)):
            y += y_points[i] * lagrange_basis(x, i, x_points)
        y_eval.append(y)
    return np.array(y_eval)

# Known data points
x_points = np.array([0, 1, 2, 3, 4])
y_points = np.array([1, 2, 0, 2, 1])

# Generate points to evaluate the interpolated polynomial
x_eval = np.linspace(min(x_points), max(x_points), 100)

# Perform interpolation
y_eval = lagrange_interpolation(x_points, y_points, x_eval)

# Plot the results
plt.scatter(x_points, y_points, color='red', label='Data Points')
plt.plot(x_eval, y_eval, label='Lagrange Polynomial', color='blue')
plt.title("Polynomial Interpolation (Lagrange)")
plt.xlabel("x")
plt.ylabel("y")
plt.grid(True)
plt.legend()
plt.show()
