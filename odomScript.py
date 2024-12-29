import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import tf
import math

class OdometryNode:
    def _init_(self):
        rospy.init_node("odometry_node")
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.left_velocity = 0.0
        self.right_velocity = 0.0
        self.yaw_rate = 0.0

        self.last_time = rospy.Time.now()

        # ROS Publishers and Subscribers
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.tf_broadcaster = tf.TransformBroadcaster()

        rospy.Subscriber("/drive_system_left_motors_feedbacks", Float64MultiArray, self.get_left)
        rospy.Subscriber("/drive_system_right_motors_feedbacks", Float64MultiArray, self.get_right)
        rospy.Subscriber("/imu1/data", Imu, self.get_imu)

    def get_left(self, left):
        """Callback function for left motor velocity."""
        self.left_velocity = (left.data[0] + left.data[1]) / 2
        self.left_velocity = self.left_velocity * 2 * math.pi * 0.135 / 60  # Convert to m/s

    def get_right(self, right):
        """Callback function for right motor velocity."""
        self.right_velocity = (right.data[0] + right.data[1]) / 2
        self.right_velocity = self.right_velocity * 2 * math.pi * 0.135 / 60  # Convert to m/s

    def get_imu(self, imu_data):
        """Callback function for IMU data."""
        self.yaw_rate = imu_data.angular_velocity.z

    @staticmethod
    def euler_to_quaternion(roll, pitch, yaw):
        """Convert Euler angles to quaternion."""
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return qx, qy, qz, qw

    def update_odometry(self):
        """Calculate and publish odometry."""
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        self.last_time = current_time

        # Compute linear and angular velocities
        linear_velocity = (self.right_velocity + self.left_velocity) / 2
        angular_velocity =self.yaw_rate  # Combine IMU and odometry

        # Update pose
        delta_theta = angular_velocity * dt
        self.theta += delta_theta
        delta_x = linear_velocity * math.cos(self.theta) * dt
        delta_y = linear_velocity * math.sin(self.theta) * dt
        self.x += delta_x
        self.y += delta_y

        rospy.loginfo(f"Position -> x: {self.x:.2f}, y: {self.y:.2f}, theta: {self.theta:.2f}")

        # Publish odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.orientation = Quaternion(*self.euler_to_quaternion(0, 0, self.theta))
        odom_msg.twist.twist.linear.x = linear_velocity
        odom_msg.twist.twist.angular.z = angular_velocity
        self.odom_pub.publish(odom_msg)

        # Broadcast the transform
        self.tf_broadcaster.sendTransform(
            (self.x, self.y, 0),
            self.euler_to_quaternion(0, 0, self.theta),
            current_time,
            "base_link",
            "odom",
        )

    def spin(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            self.update_odometry()
            rate.sleep()

if _name_ == "_main_":
    try:
        node = OdometryNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass