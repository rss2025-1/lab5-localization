import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Sensor model component functions for understanding (we avoid using these due to Python function overhead)
def p_hit(z, d, sigma, z_max):
    if 0 <= z <= z_max:
        return (1 / np.sqrt(2 * np.pi * sigma**2)) * np.exp(-((z - d) ** 2) / (2 * sigma**2))
    return 0

def p_short(z, d):
    if 0 <= z <= d and d != 0:
        return (2 / d) * (1 - (z / d))
    return 0

def p_max(z, z_max):   
    return 1 if z == z_max else 0

def p_rand(z, z_max):
    if 0 <= z <= z_max:
        return 1 / z_max
    return 0

# 3D sensor model plot function
def plot_sensor_model_3d(sigma=8.0,
    z_max=200.0,
    table_width=201,
    alpha_hit=0.74,
    alpha_short=0.07,
    alpha_max=0.07,
    alpha_rand=0.12
):
    # Validate alpha sum
    total_alpha = alpha_hit + alpha_short + alpha_max + alpha_rand
    if not np.isclose(total_alpha, 1.0):
        raise ValueError("Alpha values must sum to 1.")

    # Build grid
    zs = np.linspace(0, z_max, table_width)
    ds = np.linspace(0, z_max, table_width)
    Z, D = np.meshgrid(zs, ds, indexing='ij')

    # Compute P_hit likelihood table
    P_hit = (1.0 / (np.sqrt(2 * np.pi * sigma**2))) * np.exp(-((Z - D) ** 2) / (2 * sigma**2))
    # Normalize each column (ground truth slice) to sum to 1
    sumP_hit = np.sum(P_hit, axis=0, keepdims=True)
    sumP_hit[sumP_hit == 0] = 1  # avoid division by zero
    P_hit /= sumP_hit

    # Compute P_short likelihood table
    P_short = np.zeros_like(P_hit)
    mask_short = (Z <= D) & (D != 0) # p_short is defined only when 0 <= z <= d and d != 0.
    P_short[mask_short] = (2.0 / D[mask_short]) * (1 - (Z[mask_short] / D[mask_short]))

    # Compute P_max likelihood table
    # p_max is 1 if z == z_max and 0 otherwise.
    P_max = np.where(np.isclose(Z, z_max), 1.0, 0.0) # np.isclose to deal with any floating point precision issues.

    # Compute P_rand likelihood table
    P_rand = np.full_like(P_hit, 1.0 / z_max)

    # Combine all components into the final likelihood table
    P = (alpha_hit   * P_hit +
         alpha_short * P_short +
         alpha_max   * P_max +
         alpha_rand  * P_rand)
    # Normalize each column so that for each ground truth d, the probabilities sum to 1.
    col_sums = np.sum(P, axis=0, keepdims=True)
    col_sums[col_sums == 0] = 1  # avoid division by zero
    P /= col_sums

    # Old implementation for understanding
    # P_hit = np.empty((table_width, table_width))

    # # Compute likelihood table
    # for z_k in range(0, table_width):
    #     for d in range(0, table_width):
    #         P_hit[z_k, d] = p_hit(float(z_k), float(d), sigma, z_max)
    # sumP_hit = np.sum(P_hit, axis=0)
    # sumP_hit[sumP_hit == 0] = 1  # Avoid division by zero
    
    # P_hit /= sumP_hit
    # P= np.empty((table_width, table_width))

    # for z_k in range(table_width):
    #     for d in range(table_width):
    #         P[z_k, d] = (
    #             alpha_hit * P_hit[z_k, d] +
    #             alpha_short * p_short(z_k, d) +
    #             alpha_max * p_max(z_k, z_max) +
    #             alpha_rand * p_rand(z_k, z_max)
    #         )
           
    # col_sums = np.sum(P, axis=0, keepdims = True)
    # col_sums[col_sums == 0] = 1  # Avoid division by zero
    # P /= col_sums
    
    # Plot 3D surface
    fig = plt.figure(figsize=(10, 7))
    ax = fig.add_subplot(111, projection='3d')
    surf = ax.plot_surface(D, Z, P, cmap='viridis', edgecolor='none')

    ax.set_xlabel('Ground Truth Distance (in px)')
    ax.set_ylabel('Measured Distance (in px)')
    ax.set_zlabel('p(z | d)')
    ax.set_title('3D Sensor Model: Likelihood Surface')
    fig.colorbar(surf, shrink=0.5, aspect=1, label='Probability')
    plt.tight_layout()
    plt.show()

# Call it!
plot_sensor_model_3d()
