import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy

class PIDController():
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.previous_error = 0.0
        self.integral = 0.0
        self.dt = 0.05

    def update(self, error, dt):
        P = self.Kp * error
        self.integral += error * dt
        I = self.Ki * self.integral
        derivative = (error - self.previous_error) / dt
        D = self.Kd * derivative
        PID = P + I + D
        self.previous_error = error
        return PID

class WallFollowingBot(Node):

    def __init__(self):
        super().__init__('wall_following_bot')

        # PID tuned with Ziegler-Nichols
        Kp = 0.4
        Kd = 0.08
        Ki = 0.02
        self.dt = 0.05

        self.pid = PIDController(Kp=Kp, Ki=Ki, Kd=Kd)
        self.desired_distance = 0.25 # Target distance from wall

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub_ = self.create_subscription(LaserScan, '/scan', self.distance_callback, qos)
        
        self.current_distance = None
        self.front_distance = None
        self.previous_time = self.get_clock().now()

        self.last_error_sign = None
        self.last_sign_change_time = None
        self.oscillation_periods = []

        self.pub_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Timer to update control logic periodically
        self.timer_ = self.create_timer(self.dt, self.update_control)

    def find_nearest(self, l):
        f_list = list(filter(lambda item: item > 0.0, l))
        if f_list:
            return min(f_list)
        else:
            return float('inf')

    def distance_callback(self, msg):
        # Get right-side distance from the scan data
        right_side_range = self.find_nearest(msg.ranges[265:275])
        self.current_distance = right_side_range

        front_ranges = self.find_nearest(msg.ranges[0:5]) + self.find_nearest(msg.ranges[355:360])
        self.front_distance = front_ranges
        # No need to call update_control here, it's already handled by the timer

    def update_control(self):
        if self.current_distance is None or self.current_distance == float('inf'):
            return  # Skip if no valid distance data

        # Calculate the time difference (dt)
        current_time = self.get_clock().now()
        dt = (current_time - self.previous_time).nanoseconds / 1e9  # Time in seconds
        self.previous_time = current_time

        # Calculate error in distance from the desired distance
        error = self.desired_distance - self.current_distance
        self.get_logger().info(f"Current Distance: {self.current_distance}, Error: {error}")

        # Detect obstacles in front (in the central range of the laser scan)
        obstacle_threshold = 0.5  # Distance threshold to detect obstacles (in meters)

        # If no obstacle is in front, calculate PID correction for steering
        correction = self.pid.update(error=error, dt=dt)

        if self.front_distance < obstacle_threshold:
            # If there's an obstacle in front, the bot needs to turn
            self.get_logger().info("Obstacle detected in front! Turning...")

            # Turn right if obstacle is detected in front
            twist = Twist()
            twist.linear.x = 0.05  # Move slowly forward
            twist.angular.z = correction  # Turn right (you can adjust the turning speed or direction)
            self.pub_.publish(twist)
            return


        # Log the correction value
        self.get_logger().info(f"Correction: {correction}")

        # Create Twist message for robot movement
        twist = Twist()
        twist.linear.x = 0.1  # Constant forward speed
        # twist.angular.z = correction  # Steering adjustment

        # Publish the velocity command to the robot
        self.pub_.publish(twist)

    def on_shutdown(self):
        self.get_logger().info('Shutting down wall follower...')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)    
    controller = WallFollowingBot()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.on_shutdown()
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()