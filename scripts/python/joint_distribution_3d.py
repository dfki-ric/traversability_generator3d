import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Define a 2D Gaussian function
def gaussian_2d(X, Y, mu_x, mu_y, sigma_x, sigma_y):
    """Returns a 2D Gaussian distribution."""
    return np.exp(-(((X - mu_x) ** 2) / (2 * sigma_x ** 2) + ((Y - mu_y) ** 2) / (2 * sigma_y ** 2)))

# Define the grid
grid_size = 50  # 50x50 grid
x = np.linspace(-25, 25, grid_size)
y = np.linspace(-25, 25, grid_size)
X, Y = np.meshgrid(x, y)

# Define footsteps for concrete and sand
footsteps_concrete = [
    {"mu_x": -15, "mu_y": -10, "sigma_x": 5, "sigma_y": 5},  # Front-left leg
    {"mu_x": -15, "mu_y": 10, "sigma_x": 5, "sigma_y": 5},   # Front-right leg
    {"mu_x": 10, "mu_y": -10, "sigma_x": 5, "sigma_y": 5},   # Back-left leg
    {"mu_x": 10, "mu_y": 10, "sigma_x": 5, "sigma_y": 5},    # Back-right leg
]

footsteps_sand = [
    {"mu_x": -5, "mu_y": 0, "sigma_x": 10, "sigma_y": 10},   # Midstep area with diffuse sand
    {"mu_x": 0, "mu_y": 10, "sigma_x": 12, "sigma_y": 12},   # Sand spread over a larger area
]

# Initialize likelihood distributions
likelihood_concrete = np.zeros((grid_size, grid_size))
likelihood_sand = np.zeros((grid_size, grid_size))

# Add Gaussians for concrete likelihood
for step in footsteps_concrete:
    likelihood_concrete += gaussian_2d(X, Y, **step)

# Add Gaussians for sand likelihood
for step in footsteps_sand:
    likelihood_sand += gaussian_2d(X, Y, **step)

# Normalize likelihoods
likelihood_concrete /= likelihood_concrete.max()
likelihood_sand /= likelihood_sand.max()

# Define prior probabilities (assume uniform)
prior_concrete = 0.5
prior_sand = 0.5

# Compute evidence P(Observation)
evidence = likelihood_concrete * prior_concrete + likelihood_sand * prior_sand

# Compute posterior probabilities using Bayes' Rule
posterior_concrete = (likelihood_concrete * prior_concrete) / evidence
posterior_sand = (likelihood_sand * prior_sand) / evidence

# Plotting: 3D surface plots of probabilities
fig_3d = plt.figure(figsize=(12, 10))

# 3D plot for Posterior Probability of Concrete
ax_concrete = fig_3d.add_subplot(121, projection='3d')
X_flat = X.flatten()
Y_flat = Y.flatten()
Z_concrete = posterior_concrete.flatten()  # Posterior for concrete at each cell

# 3D surface plot for concrete
ax_concrete.plot_trisurf(X_flat, Y_flat, Z_concrete, cmap='Blues', edgecolor='none')
ax_concrete.set_title('Posterior Probability: Concrete')
ax_concrete.set_xlabel('X-axis')
ax_concrete.set_ylabel('Y-axis')
ax_concrete.set_zlabel('Probability')

# 3D plot for Posterior Probability of Sand
ax_sand = fig_3d.add_subplot(122, projection='3d')
Z_sand = posterior_sand.flatten()  # Posterior for sand at each cell

# 3D surface plot for sand
ax_sand.plot_trisurf(X_flat, Y_flat, Z_sand, cmap='Oranges', edgecolor='none')
ax_sand.set_title('Posterior Probability: Sand')
ax_sand.set_xlabel('X-axis')
ax_sand.set_ylabel('Y-axis')
ax_sand.set_zlabel('Probability')

# Show the plots
plt.tight_layout()
plt.show()
