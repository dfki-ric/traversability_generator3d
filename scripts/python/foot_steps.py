import numpy as np
import matplotlib.pyplot as plt

# Define a 2D Gaussian function
def gaussian_2d(X, Y, mu_x, mu_y, sigma_x, sigma_y):
    """Returns a 2D Gaussian distribution."""
    return np.exp(-(((X - mu_x) ** 2) / (2 * sigma_x ** 2) + ((Y - mu_y) ** 2) / (2 * sigma_y ** 2)))

# Define the grid
grid_size = 50  # 50x50 grid
x = np.linspace(-25, 25, grid_size)
y = np.linspace(-25, 25, grid_size)
X, Y = np.meshgrid(x, y)

# Define footsteps for a quadruped
footsteps = [
    {"mu_x": -15, "mu_y": -10, "sigma_x": 5, "sigma_y": 5},  # Front-left leg
    {"mu_x": -15, "mu_y": 10, "sigma_x": 5, "sigma_y": 5},   # Front-right leg
    {"mu_x": 10, "mu_y": -10, "sigma_x": 5, "sigma_y": 5},   # Back-left leg
    {"mu_x": 10, "mu_y": 10, "sigma_x": 5, "sigma_y": 5},    # Back-right leg
]

# Initialize the combined probability distribution
combined_distribution = np.zeros((grid_size, grid_size))

# Add each Gaussian contribution
for step in footsteps:
    combined_distribution += gaussian_2d(X, Y, **step)

# Normalize the distribution
combined_distribution /= combined_distribution.max()

# Plot the combined probability distribution
plt.figure(figsize=(8, 6))
plt.imshow(combined_distribution, extent=(-25, 25, -25, 25), origin='lower', cmap='viridis')
plt.colorbar(label='Normalized Probability')
plt.scatter(
    [step["mu_x"] for step in footsteps],
    [step["mu_y"] for step in footsteps],
    c='red', marker='x', label='Footstep Centers'
)
plt.title('2D Grid with Joint Probability Distribution')
plt.xlabel('X-axis')
plt.ylabel('Y-axis')
plt.legend()
plt.grid(True)
plt.show()

