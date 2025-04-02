import numpy as np

class MotionModel:

    def __init__(self, node):
        ####################################
        # TODO
        # Do any precomputation for the motion
        # model here.

        node.declare_parameter("deterministic", True)
        self.deterministic = node.get_parameter("deterministic").value

        node.get_logger().info(f"Deterministic Mode: {self.deterministic}")
        ####################################

    def evaluate(self, particles, odometry):
        """
        Update the particles to reflect probable
        future states given the odometry data.

        args:
            particles: An Nx3 matrix of the form:

                [x0 y0 theta0]
                [x1 y0 theta1]
                [    ...     ]

            odometry: A 3-vector [dx dy dtheta]

        returns:
            particles: An updated matrix of the
                same size
        """
        particles = np.asarray(particles) # Ensure particles is a numpy array

        dx, dy, dtheta = odometry

        sigma_x = 0.1  # TODO
        sigma_y = 0.02  # TODO
        sigma_theta = np.pi / 4  # TODO

        x = particles[:, 0]
        y = particles[:, 1]
        theta = particles[:, 2]

        # Rotate odometry displacement to the world frame
        dx_world = dx * np.cos(theta) - dy * np.sin(theta)
        dy_world = dx * np.sin(theta) + dy * np.cos(theta)

        # Determine noise: if deterministic, use zeros; otherwise, sample from normal distributions
        if not self.deterministic:
            x_noise = np.random.normal(0, sigma_x, size=x.shape)
            y_noise = np.random.normal(0, sigma_y, size=y.shape)
            theta_noise = np.random.normal(0, sigma_theta, size=theta.shape)

        # Update particles with the rotated odometry and noise
        new_x = x + dx_world + x_noise if not self.deterministic else x + dx_world
        new_y = y + dy_world + y_noise if not self.deterministic else y + dy_world
        new_theta = theta + dtheta + theta_noise if not self.deterministic else theta + dtheta

        # Combine into the updated particles array
        updated_particles = np.stack((new_x, new_y, new_theta), axis=-1)
        return updated_particles

