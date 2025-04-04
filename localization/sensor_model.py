import numpy as np
from scan_simulator_2d import PyScanSimulator2D
# Try to change to just `from scan_simulator_2d import PyScanSimulator2D` 
# if any error re: scan_simulator_2d occurs

from tf_transformations import euler_from_quaternion

from nav_msgs.msg import OccupancyGrid

import sys

np.set_printoptions(threshold=sys.maxsize)


class SensorModel:

    def __init__(self, node):
        node.declare_parameter('map_topic', "default")
        node.declare_parameter('num_beams_per_particle', 1)
        node.declare_parameter('scan_theta_discretization', 1.0)
        node.declare_parameter('scan_field_of_view', 1.0)
        node.declare_parameter('lidar_scale_to_map_scale', 1.0)

        self.map_topic = node.get_parameter('map_topic').get_parameter_value().string_value
        self.num_beams_per_particle = node.get_parameter('num_beams_per_particle').get_parameter_value().integer_value
        self.scan_theta_discretization = node.get_parameter(
            'scan_theta_discretization').get_parameter_value().double_value
        self.scan_field_of_view = node.get_parameter('scan_field_of_view').get_parameter_value().double_value
        self.lidar_scale_to_map_scale = node.get_parameter(
            'lidar_scale_to_map_scale').get_parameter_value().double_value

        ####################################
        # Adjust these parameters
        self.alpha_hit = 0.74
        self.alpha_short = 0.07
        self.alpha_max = 0.07
        self.alpha_rand = 0.12
        self.sigma_hit = 8.0
        

        # Your sensor table will be a `table_width` x `table_width` np array:
        self.table_width = 201
        ####################################

        node.get_logger().info("%s" % self.map_topic)
        node.get_logger().info("%s" % self.num_beams_per_particle)
        node.get_logger().info("%s" % self.scan_theta_discretization)
        node.get_logger().info("%s" % self.scan_field_of_view)

        # Precompute the sensor model table
        self.sensor_model_table = np.empty((self.table_width, self.table_width))
        self.precompute_sensor_model()

        # Create a simulated laser scan
        self.scan_sim = PyScanSimulator2D(
            self.num_beams_per_particle,
            self.scan_field_of_view,
            0,  # This is not the simulator, don't add noise
            0.01,  # This is used as an epsilon
            self.scan_theta_discretization)

        # Subscribe to the map
        self.map = None
        self.map_set = False
        self.map_subscriber = node.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_callback,
            1)

    def precompute_sensor_model(self):
        """
        Generate and store a table which represents the sensor model.

        For each discrete computed range value, this provides the probability of 
        measuring any (discrete) range. This table is indexed by the sensor model
        at runtime by discretizing the measurements and computed ranges from
        RangeLibc.
        This table must be implemented as a numpy 2D array.

        Compute the table based on class parameters alpha_hit, alpha_short,
        alpha_max, alpha_rand, sigma_hit, and table_width.

        args:
            N/A

        returns:
            No return type. Directly modify `self.sensor_model_table`.
        """
        # Format: self.sensor_model_table[measured_distance, ground_truth]

        # Check that the alpha values sum to 1
        total = self.alpha_hit + self.alpha_short + self.alpha_max + self.alpha_rand
        if not np.isclose(total, 1.0):
            raise ValueError("Alpha values must sum to 1.")
        
        self.z_max = self.table_width - 1  # Maximum range of the sensor

        # Build grid
        zs = np.linspace(0, self.z_max, self.table_width)
        ds = np.linspace(0, self.z_max, self.table_width)
        Z, D = np.meshgrid(zs, ds, indexing='ij')

        # Compute P_hit likelihood table
        """Case 1: Gaussian centered on expected distance d."""
        P_hit = (1.0 / (np.sqrt(2 * np.pi * self.sigma_hit**2))) * np.exp(-((Z - D) ** 2) / (2 * self.sigma_hit**2))
        # Normalize each column (ground truth slice) to sum to 1
        sumP_hit = np.sum(P_hit, axis=0, keepdims=True)
        sumP_hit[sumP_hit == 0] = 1  # avoid division by zero
        P_hit /= sumP_hit

        # Compute P_short likelihood table
        """Case 2: Short measurement (unexpected obstacle)."""
        P_short = np.zeros_like(P_hit)
        mask_short = (Z <= D) & (D != 0) # p_short is defined only when 0 <= z <= d and d != 0.
        P_short[mask_short] = (2.0 / D[mask_short]) * (1 - (Z[mask_short] / D[mask_short]))

        # Compute P_max likelihood table
        # p_max is 1 if z == z_max and 0 otherwise.
        """Case 3: Max range (missed detection)."""
        P_max = np.where(np.isclose(Z, self.z_max), 1.0, 0.0) # np.isclose to deal with any floating point precision issues.

        # Compute P_rand likelihood table
        """Case 4: Random measurement (noise, unknown event)."""
        P_rand = np.full_like(P_hit, 1.0 / self.z_max)

        # Combine all components into the final likelihood table
        self.sensor_model_table = (
            self.alpha_hit * P_hit +
            self.alpha_short * P_short +
            self.alpha_max   * P_max +
            self.alpha_rand  * P_rand
            )
        # Normalize each column so that for each ground truth d, the probabilities sum to 1.
        col_sums = np.sum(self.sensor_model_table, axis=0, keepdims=True)
        col_sums[col_sums == 0] = 1  # avoid division by zero
        self.sensor_model_table /= col_sums

    def meters_to_pixels(self, meters):
        return (meters / float(self.resolution * self.lidar_scale_to_map_scale))

    def evaluate(self, particles, observation):
        """
        Evaluate how likely each particle is given
        the observed scan.

        args:
            particles: An Nx3 matrix of the form:

                [x0 y0 theta0]
                [x1 y0 theta1]
                [    ...     ]

            observation: A vector of lidar data measured
                from the actual lidar. THIS IS Z_K. Each range in Z_K is Z_K^i

        returns:
           probabilities: A vector of length N representing
               the probability of each particle existing
               given the observation and the map.
        """

        if not self.map_set:
            return

        # Evaluate the sensor model here!
        #
        # You will probably want to use this function
        # to perform ray tracing from all the particles.
        # This produces a matrix of size N x num_beams_per_particle 

        scans = self.scan_sim.scan(particles)
        scans = self.meters_to_pixels(np.array(scans)) # N * num_beams
        scans = np.rint(np.clip(scans, 0, 200)).astype(int) # N * num_beams
        
        observation = self.meters_to_pixels(np.array(observation)) # 1 x num_beams
        observation = np.rint(np.clip(observation, 0, 200)).astype(int) # 1 x num_beams
        observation = observation[np.newaxis, :] # Reshaped for broadcasting
        
        likelihoods = self.sensor_model_table[scans, observation] # N x num_beams
        probabilities = np.exp(np.sum(np.log(likelihoods + 1e-12), axis=1)) # N x 1, added 1e-12 to avoid log(0)
        return probabilities

        # Geometric mean might be extra and unexpected from unit tests; we can try it after everything works.
        # N,m = scans.shape
        # for scan in scans:
        #     difference = np.abs(scan - observation)
        #     likelihood = 1
        #     for i in range(len(scan)):
        #         likelihood *= self.sensor_model_table[difference[i], scan[i]]

        # probs = self.sensor_model_table[difference, scan]

        # probs = np.exp(np.sum(np.log(probs), axis=1))
        # return np.array(np.power(probs, 1.0 / self.num_beams_per_particle))


    def map_callback(self, map_msg):
        # Convert the map to a numpy array
        self.map = np.array(map_msg.data, np.double) / 100.
        self.map = np.clip(self.map, 0, 1)

        self.resolution = map_msg.info.resolution

        # Convert the origin to a tuple
        origin_p = map_msg.info.origin.position
        origin_o = map_msg.info.origin.orientation
        origin_o = euler_from_quaternion((
            origin_o.x,
            origin_o.y,
            origin_o.z,
            origin_o.w))
        origin = (origin_p.x, origin_p.y, origin_o[2])

        # Initialize a map with the laser scan
        self.scan_sim.set_map(
            self.map,
            map_msg.info.height,
            map_msg.info.width,
            map_msg.info.resolution,
            origin,
            0.5)  # Consider anything < 0.5 to be free

        # Make the map set
        self.map_set = True

        print("Map initialized")


