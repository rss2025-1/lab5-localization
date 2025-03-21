import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Sensor model component functions
def p_hit(z, d, sigma, z_max):
    if 0 <= z <= z_max:
        return (1 / np.sqrt(2 * np.pi * sigma**2)) * np.exp(-((z - d) ** 2) / (2 * sigma**2))
    return 0

def p_short(z, d):
    if 0 <= z <= d and d != 0:
        return (2 / d) * (1 - (z / d))
    return 0

def p_max(z, z_max, epsilon):
    if z_max - epsilon <= z <= z_max:
        return 1 / epsilon
    return 0

def p_rand(z, z_max):
    if 0 <= z <= z_max:
        return 1 / z_max
    return 0

def p_z_given_xm(z, d, sigma, z_max, epsilon,
                 alpha_hit, alpha_short, alpha_max, alpha_rand):
    return (alpha_hit * p_hit(z, d, sigma, z_max) +
            alpha_short * p_short(z, d) +
            alpha_max * p_max(z, z_max, epsilon) +
            alpha_rand * p_rand(z, z_max))


# 3D sensor model plot function
def plot_sensor_model_3d(
    sigma=0.2,
    z_max=10.0,
    epsilon=0.01,
    table_width=200,
    alpha_hit=0.7,
    alpha_short=0.1,
    alpha_max=0.1,
    alpha_rand=0.1
):
    # Validate alpha sum
    total_alpha = alpha_hit + alpha_short + alpha_max + alpha_rand
    if not np.isclose(total_alpha, 1.0):
        raise ValueError("Alpha values must sum to 1.")

    # Build grid
    zs = np.linspace(0, z_max, table_width)
    ds = np.linspace(0, z_max, table_width)
    Z, D = np.meshgrid(zs, ds, indexing='ij')

    # Compute likelihood table
    P = np.zeros_like(Z)
    for i in range(table_width):
        for j in range(table_width):
            z = zs[i]
            d = ds[j]
            P[i, j] = p_z_given_xm(z, d, sigma, z_max, epsilon,
                                   alpha_hit, alpha_short, alpha_max, alpha_rand)
            if P[i, j] > 1:
                print("Warning: Probability greater than 1 detected. Check parameters.")
                

    # Plot 3D surface
    fig = plt.figure(figsize=(10, 7))
    ax = fig.add_subplot(111, projection='3d')
    surf = ax.plot_surface(D, Z, P, cmap='viridis', edgecolor='none')
    print(plot_sensor_model_3d)

    ax.set_xlabel('Ground Truth Distance (d)')
    ax.set_ylabel('Measured Distance (z)')
    ax.set_zlabel('p(z | d)')
    ax.set_title('3D Sensor Model: Likelihood Surface')
    fig.colorbar(surf, shrink=0.5, aspect=1, label='Probability')
    plt.tight_layout()
    plt.show()

# Call it!
plot_sensor_model_3d()
