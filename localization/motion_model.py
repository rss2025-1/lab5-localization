import numpy as np

class MotionModel:

    def __init__(self, node):
        ####################################
        # TODO
        # Do any precomputation for the motion
        # model here.

        self.node = node

        node.declare_parameter("deterministic", "default")
        self.deterministic = node.get_parameter("deterministic").get_parameter_value().bool_value

        self.get_logger().info(f"Deterministic Mode: {self.deterministic}")
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



        dx, dy, dtheta = odometry

        

        sigma_x = 0 # TODO
        sigma_y = 0 # TODO
        sigma_theta = 0 # TODO

        updated_particles = []

        for particle in particles:
            if not self.deterministic:
                x_noise = np.random.normal(0, sigma_x)
                y_noise = np.random.normal(0 ,sigma_y)
                theta_noise = np.random.normal(0 ,sigma_theta)
            else:
                x_noise = 0
                y_noise = 0
                theta_noise = 0
            x, y, theta = particle
            dx_world = x * np.cos(particle[-1]) - y * np.sin(particle[-1])
            dy_world = x * np.sin(particle[-1]) + y * np.cos(particle[-1])
            
            updated_particle = [
                x + dx_world + x_noise,
                y + dy_world + y_noise,
                theta + dtheta + theta_noise
            ]

            updated_particles.append(updated_particle)

        return np.array(updated_particles)

