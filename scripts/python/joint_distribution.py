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

# Plot the posterior probabilities
fig, axes = plt.subplots(1, 3, figsize=(18, 6))

# Posterior for concrete
axes[0].imshow(posterior_concrete, extent=(-25, 25, -25, 25), origin='lower', cmap='Blues')
axes[0].set_title('Posterior Probability: Concrete')
axes[0].scatter(
    [step["mu_x"] for step in footsteps_concrete],
    [step["mu_y"] for step in footsteps_concrete],
    c='red', marker='x', label='Concrete Footsteps'
)
axes[0].legend()
axes[0].grid(True)

# Posterior for sand
axes[1].imshow(posterior_sand, extent=(-25, 25, -25, 25), origin='lower', cmap='Oranges')
axes[1].set_title('Posterior Probability: Sand')
axes[1].scatter(
    [step["mu_x"] for step in footsteps_sand],
    [step["mu_y"] for step in footsteps_sand],
    c='red', marker='x', label='Sand Footsteps'
)
axes[1].legend()
axes[1].grid(True)

# Combined terrain map
combined_map = np.where(posterior_concrete > posterior_sand, 1, 0)  # 1 for concrete, 0 for sand
axes[2].imshow(combined_map, extent=(-25, 25, -25, 25), origin='lower', cmap='coolwarm', alpha=0.8)
axes[2].set_title('Terrain Classification (1: Concrete, 0: Sand)')
axes[2].grid(True)

for ax in axes:
    ax.set_xlabel('X-axis')
    ax.set_ylabel('Y-axis')

plt.tight_layout()
plt.show()

