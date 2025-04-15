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
        
        sigma_x = 0.05 # TODO
        sigma_y = 0.05  # TODO
        sigma_theta = np.deg2rad(20)  # TODO

        x = particles[:, 0]
        y = particles[:, 1]
        theta = particles[:, 2]

        # Sample or use zero noise
        if self.deterministic:
            noise = np.zeros((len(particles), 3))
        else:
            noise = np.stack([
                np.random.normal(0, sigma_x, size=len(particles)),
                np.random.normal(0, sigma_y, size=len(particles)),
                np.random.normal(0, sigma_theta, size=len(particles))
            ], axis=1)

        # Broadcast scalar odometry across all particles and add noise
        dx_all = dx + noise[:, 0]
        dy_all = dy + noise[:, 1]
        dtheta_all = dtheta + noise[:, 2]

        # Transform to world frame
        dx_world = dx_all * np.cos(theta) - dy_all * np.sin(theta)
        dy_world = dx_all * np.sin(theta) + dy_all * np.cos(theta)

        # Update particles
        new_x = x + dx_world
        new_y = y + dy_world
        new_theta = theta + dtheta_all

        return np.stack((new_x, new_y, new_theta), axis=-1)

