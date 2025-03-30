from localization.sensor_model import SensorModel
from localization.motion_model import MotionModel
from sensor_msgs.msg import LaserScan

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Pose, TransformStamped
from tf_transformations import quaternion_from_euler, euler_from_quaternion

from rclpy.node import Node
import rclpy
import numpy as np
assert rclpy
import tf2_ros
import threading


class ParticleFilter(Node):

    def __init__(self):
        super().__init__("particle_filter")

        # MAKE SURE YOU INITIALIZED ALL THESE VARIABLES
        self.initialized = False
        self.lock = threading.Lock()
        self.prev_time = self.get_clock().now()
        self.num_particles = 200  # Default value or get from parameter
        self.particles = None
        self.weights = None

        self.declare_parameter('particle_filter_frame', "default")
        self.particle_filter_frame = self.get_parameter('particle_filter_frame').get_parameter_value().string_value

        #  *Important Note #1:* It is critical for your particle
        #     filter to obtain the following topic names from the
        #     parameters for the autograder to work correctly. Note
        #     that while the Odometry message contains both a pose and
        #     a twist component, you will only be provided with the
        #     twist component, so you should rely only on that
        #     information, and *not* use the pose component.
        
        self.declare_parameter('odom_topic', "/odom")
        self.declare_parameter('scan_topic', "/scan")

        scan_topic = self.get_parameter("scan_topic").get_parameter_value().string_value
        odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value

        self.laser_sub = self.create_subscription(LaserScan, scan_topic,
                                                  self.laser_callback,
                                                  1)

        self.odom_sub = self.create_subscription(Odometry, odom_topic,
                                                 self.odom_callback,
                                                 1)

        #  *Important Note #2:* You must respond to pose
        #     initialization requests sent to the /initialpose
        #     topic. You can test that this works properly using the
        #     "Pose Estimate" feature in RViz, which publishes to
        #     /initialpose.

        self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, "/initialpose",
                                                 self.pose_callback,
                                                 1)

        #  *Important Note #3:* You must publish your pose estimate to
        #     the following topic. In particular, you must use the
        #     pose field of the Odometry message. You do not need to
        #     provide the twist part of the Odometry message. The
        #     odometry you publish here should be with respect to the
        #     "/map" frame.

        self.odom_pub = self.create_publisher(Odometry, "/pf/pose/odom", 1)

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.particles_pub = self.create_publisher(PoseArray, "/particles", 1) # For debuggingvisualization

        # Initialize the models
        self.motion_model = MotionModel(self)
        self.sensor_model = SensorModel(self)
        self.num_beams_per_particle = self.get_parameter("num_beams_per_particle").get_parameter_value().integer_value

        self.get_logger().info("=============+READY+=============")

        # Implement the MCL algorithm
        # using the sensor model and the motion model
        #
        # Make sure you include some way to initialize
        # your particles, ideally with some sort
        # of interactive interface in rviz
        #
        # Publish a transformation frame between the map
        # and the particle_filter_frame.

    def pose_callback(self, pose_msg):
        """
        Callback for pose initialization requests from RViz
        
        Args:
            pose_msg: PoseWithCovarianceStamped message from /initialpose topic
        """
        self.lock.acquire()
        try:
            self.initialize_particles(pose_msg)
            self.weights = np.ones(len(self.particles)) / len(self.particles)
            self.initialized = True 
            self.prev_time = self.get_clock().now() 
            
            self.get_logger().info("Reinitialized particles based on RViz pose estimate")
        finally:
            self.lock.release()

    def odom_callback(self, odom_msg):
        """
        Update particle positions using motion model
        """
        if not self.initialized:
            return
            
        self.lock.acquire()
        try:
            # Calculate time difference
            now = self.get_clock().now()
            dt = (now - self.prev_time).nanoseconds / 1e9
            self.prev_time = now
            
            # Skip if dt is too small (avoid division by zero)
            if dt < 1e-6:
                return
                
            # Extract motion data (use only twist component as instructed)
            vx = odom_msg.twist.twist.linear.x
            vy = odom_msg.twist.twist.linear.y
            wz = odom_msg.twist.twist.angular.z
            
            # Scale by time
            dx = vx * dt
            dy = vy * dt
            dtheta = wz * dt
            
            # Update particles using motion model
            self.particles = self.motion_model.evaluate(self.particles, [dx, dy, dtheta])
            
            # Publish updated pose
            self.publish_pose_estimate()
            
        finally:
            self.lock.release()

    def laser_callback(self, scan_msg):
        """
        Update particle weights using sensor model and resample
        """
        if not self.sensor_model.map_set or not self.initialized:
            return
            
        self.lock.acquire()
        try:
            # Downsample scan for efficiency (as recommended)
            downsampled_indices = np.linspace(0, len(scan_msg.ranges) - 1, self.num_beams_per_particle).astype(int)
            ranges = np.array(scan_msg.ranges)[downsampled_indices]
            
            # Update weights using sensor model
            self.weights = self.sensor_model.evaluate(self.particles, ranges)
            
            # Normalize weights
            if np.sum(self.weights) > 0:
                self.weights /= np.sum(self.weights)
            else:
                # If all weights are zero, reinitialize with uniform weights
                self.weights = np.ones(self.num_particles) / self.num_particles
                self.get_logger().warn("All particle weights are zero - reinitializing weights")
                
            # Resample particles
            self.resample_particles()
            
            # Publish results
            self.publish_pose_estimate()
            self.publish_particles()
            
        finally:
            self.lock.release()

    def compute_average_pose(self):
        """Compute average pose from particles"""
        if self.particles is None:
            return None
        
        # This still doesn't handle multi modal cases, which we can just use histogram bins for. But I want to check if it's even necessary since it adds more computation.

        # Compute mean position
        mean_x = np.average(self.particles[:, 0], weights=self.weights)
        mean_y = np.average(self.particles[:, 1], weights=self.weights)
        
        # Compute mean orientation using CIRCULAR MEAN
        sin_sum = np.sum(np.sin(self.particles[:, 2]) * self.weights)
        cos_sum = np.sum(np.cos(self.particles[:, 2]) * self.weights)
        mean_theta = np.arctan2(sin_sum, cos_sum)
        
        return mean_x, mean_y, mean_theta


    def resample_particles(self):
        """
        Resample particles based on weights using numpy.random.choice
        """
        indices = np.random.choice(
            self.num_particles, 
            self.num_particles, 
            replace=True, 
            p=self.weights
        )
        self.particles = self.particles[indices]
        self.weights = np.ones(self.num_particles) / self.num_particles

    def initialize_particles(self, initial_pose, num_particles=200, spread_radius=1.0):
        """
        Initialize particles around an initial pose with some random spread
        
        Args:
            initial_pose: PoseWithCovarianceStamped message containing initial position
            num_particles: Number of particles to create
            spread_radius: Maximum distance particles can be from initial pose
        """
        # Extract initial position and orientation
        x = initial_pose.pose.pose.position.x  # x coordinate
        y = initial_pose.pose.pose.position.y  # y coordinate
        theta = euler_from_quaternion([        # Convert quaternion to yaw angle
            initial_pose.pose.pose.orientation.x,
            initial_pose.pose.pose.orientation.y,
            initial_pose.pose.pose.orientation.z,
            initial_pose.pose.pose.orientation.w
        ])[2]  

        # Create arrays to store particles
        self.particles = np.zeros((num_particles, 3))  # Each particle is [x, y, theta]
        self.weights = np.ones(num_particles) / num_particles  # Equal weights initially

        # Add noise to create spread
        # Position noise (x,y)
        self.particles[:, 0] = x + np.random.uniform(-spread_radius, spread_radius, num_particles)
        self.particles[:, 1] = y + np.random.uniform(-spread_radius, spread_radius, num_particles)
        
        # Orientation noise (theta)
        self.particles[:, 2] = theta + np.random.uniform(-np.pi/4, np.pi/4, num_particles)
        
        # Normalize angles to [-pi, pi]
        self.particles[:, 2] = np.arctan2(np.sin(self.particles[:, 2]), np.cos(self.particles[:, 2]))

        self.get_logger().info(f"Initialized {num_particles} particles around ({x:.2f}, {y:.2f}, {theta:.2f})")

    def publish_pose_estimate(self):
        """
        Publish pose estimate as transform and odometry message
        """
        # Compute average pose
        x, y, theta = self.compute_average_pose()
        
        # Create and publish transform
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "/map"
        transform.child_frame_id = self.particle_filter_frame
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = 0.0
        quat = quaternion_from_euler(0, 0, theta)
        transform.transform.rotation.x = quat[0]
        transform.transform.rotation.y = quat[1]
        transform.transform.rotation.z = quat[2]
        transform.transform.rotation.w = quat[3]
        self.tf_broadcaster.sendTransform(transform)
        
        # Create and publish odometry
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "/map"
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = quat[0]
        odom.pose.pose.orientation.y = quat[1]
        odom.pose.pose.orientation.z = quat[2]
        odom.pose.pose.orientation.w = quat[3]
        self.odom_pub.publish(odom)

    def publish_particles(self):
        """
        THIS IS FOR DEBUGGING! Publish particles for visualization
        """
        # Only publish if someone is subscribed (optimization)
        if self.particles_pub.get_subscription_count() == 0:
            return
            
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        
        # Convert particles to poses
        for i in range(min(self.num_particles, 100)):  # Limit to 100 for visualization
            pose = Pose()
            pose.position.x = self.particles[i, 0]
            pose.position.y = self.particles[i, 1]
            quat = quaternion_from_euler(0, 0, self.particles[i, 2])
            pose.orientation.x = quat[0]
            pose.orientation.y = quat[1]
            pose.orientation.z = quat[2]
            pose.orientation.w = quat[3]
            msg.poses.append(pose)
            
        self.particles_pub.publish(msg)
        self.get_logger.info(self.num_particles)


def main(args=None):
    rclpy.init(args=args)
    pf = ParticleFilter()
    rclpy.spin(pf)
    rclpy.shutdown()
