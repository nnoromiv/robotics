import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy


class PIDController():

    def __init__(self, Kp, Ki, Kd):
        # PID control parameters
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.previous_error = 0.0
        self.integral = 0.0

        self.dt = 0.05

    
    def update(self, error, dt):
        # Proportional term
        P = self.Kp * error

        # Integral term
        self.integral += error * dt
        I = self.Ki * self.integral

        # Derivative term
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
            Kd = 0.02
            Ki = 0.02

            self.pid = PIDController(Kp=Kp, Ki=Ki, Kd=Kd)
            self.desired_distance = 0.5 # Target distance from wall

            # LaserScan subscriber for distance between sensor and wall
            qos = QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.BEST_EFFORT,
            )
            self.sub_ = self.create_subscription(LaserScan, '/scan', self.distance_callback, qos)
            
            # Distance and time variables
            self.current_distance = None
            self.previous_time = self.get_clock().now()

            # Variables for Oscillation period
            self.last_error_sign = None
            self.last_sign_change_time = None
            self.oscillation_periods = []

            # Publisher to /cmd_vel
            self.pub_ = self.create_publisher(Twist, '/cmd_vel', 10)
            
            # Timer to run wall following logic
            # self.timer_ = self.create_timer(self.dt, self.update)

        def find_nearest(self, list):
            f_list = filter(lambda item: item > 0.0, list)  # exclude zeros
            return min(min(f_list, default=10), 10)
        
        def distance_callback(self, msg):

            right_side_range = self.find_nearest(msg.ranges[265:275]),

            self.current_distance : int = min(right_side_range)
            self.update_control()

        def update_control(self):
            if self.current_distance is None:
                return # Wait until we get a valid distance
            
            # Calculate time step
            current_time = self.get_clock().now()
            dt = (current_time - self.previous_time).nanoseconds / 1e9 # Convert to seconds
            self.previous_time = current_time

            # Calculate distance error
            error = self.desired_distance - self.current_distance

            # Log distance
            self.get_logger().info(f"Currenct Distance: {self.current_distance}, Error: {error}")

            # Measuring Ziegler-Niclos Oscillation period Pu
            if self.last_error_sign is None:
                self.last_error_sign = error >= 0
                self.last_sign_change_time = current_time

            current_sign = error >= 0
            
            if current_sign != self.last_error_sign:
                # Calculate and log period
                if self.last_sign_change_time is not None:
                    oscillation_period = (current_time - self.last_sign_change_time).nanoseconds / 1e9
                    self.oscillation_periods.append(oscillation_period)
                    self.get_logger().info(f"Period: {oscillation_period} seconds")

                # Update for next period
                self.last_sign_change_time = current_time
                self.last_error_sign = current_sign

            # Get PID correction 
            correction = self.pid.update(error=error, dt=dt)

            # Log correction
            self.get_logger().info(f"Correction: {correction}")

            # Create twist for vel command
            twist = Twist()
            twist.linear.x = 0.2 # Forward speed
            twist.angular.z = correction # Steering adjustment

            # Publish Velocity
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