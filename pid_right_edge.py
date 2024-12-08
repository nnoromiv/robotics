import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy

# The PIDController class implements a Proportional-Integral-Derivative controller in Python.
class PIDController():
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.previous_error = 0.0
        self.integral = 0.0

    """
    The function calculates the PID control output based on the error, integral of error, derivative of
    error, and the respective coefficients Kp, Ki, and Kd.
    
    :param error: The `error` parameter in the `update` method represents the difference between the
    desired set point and the current value of the system being controlled. It is a crucial input to the
    PID controller as it determines the corrective action that needs to be taken to minimize this error
    and bring the system closer to the
    :param dt: The `dt` parameter represents the time step or time interval between each update in your
    control loop. It is typically the elapsed time since the last update and is used to calculate the
    integral and derivative terms in your PID controller
    :return: the calculated PID (Proportional-Integral-Derivative) control output based on the error and
    time difference (dt) provided as input parameters.
    """
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
        
        Kp = 0.4
        Kd = 0.1
        Ki = 0.0
        
        self.dt = 0.5

        self.pid = PIDController(
            Kp=Kp, 
            Ki=Ki, 
            Kd=Kd
        )
        self.desired_distance = 0.5 # Target distance from wall

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub_ = self.create_subscription(LaserScan, '/scan', self.distance_callback, qos)
        
        self.current_distance = None
        self.front_distance = None

        self.previous_time = self.get_clock().now()

        self.pub_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Timer to update control logic periodically
        self.timer_ = self.create_timer(self.dt, self.movement)

    def find_nearest(self, l):
        """
        The `find_nearest` function returns the nearest non-zero distance from a given list.
        
        :param l: The `find_nearest` method takes a list `l` as input. It filters out all elements in the
        list that are greater than 0.0 and then returns the nearest non-zero distance from the filtered list
        :return: The `find_nearest` method returns the nearest non-zero distance from a given list `l`. If
        there are non-zero distances in the list, it returns the minimum non-zero distance. If the list is
        empty or only contains zeros, it returns positive infinity (`float('inf')`).
        """
        """Return the nearest non-zero distance from a list"""
        f_list = list(filter(lambda item: item > 0.0, l))
        if f_list:
            return min(min(f_list, default=1), 1)
        else:
            return float('inf')

    def distance_callback(self, msg):
        """
        The `distance_callback` function calculates the front and right-side distances from scan data.
        
        :param msg: The `distance_callback` function is processing a message `msg` that
        contains scan data. The function extracts the front and right-side distances from the scan data and
        stores them in `self.front_distance` and `self.current_distance` attributes, respectively
        """
        # Get right-side distance from the scan data
        front_ranges = min(self.find_nearest(msg.ranges[0:5]), self.find_nearest(msg.ranges[355:360]))
        self.front_distance = front_ranges
        
        right_side_range = self.find_nearest(msg.ranges[265:320])
        self.current_distance = right_side_range

    def movement(self):
        """
        The `movement` function calculates error in distance, applies a PID correction for steering, and
        publishes velocity commands to steer the robot based on distance data.
        :return: The `movement` method returns when the current distance is invalid (`None` or infinity) to
        skip further processing if there is no valid distance data.
        """
        if self.current_distance is None or self.current_distance == float('inf'):
            self.get_logger().warn('Invalid right-side distance')
            return  # Skip if no valid distance data

        current_time = self.get_clock().now()
        self.previous_time = current_time

        # Calculate error in distance from the desired distance
        error = self.desired_distance - self.current_distance
        self.get_logger().info(f"Current Distance: {self.current_distance}, Error: {error}")

        # Calculate PID correction for steering
        correction = self.pid.update(error=error, dt=self.desired_distance)
        self.get_logger().info(f"Correction: {correction}")

        if self.current_distance < self.desired_distance or self.front_distance < self.desired_distance:
            self.get_logger().info("Steering away...")
            twist = Twist()
            twist.linear.x = 0.06
            twist.angular.z = correction
            self.pub_.publish(twist)
            return

        twist = Twist()
        twist.linear.x = 0.1  # Constant forward speed
        twist.angular.z = correction # Turn right (you can adjust the turning speed or direction)

        # Publish the velocity command to the robot
        self.pub_.publish(twist)

    def on_shutdown(self):
        """
        The `on_shutdown` function logs a message and shuts down the ROS 2 node.
        """
        self.get_logger().info('Shutting down wall follower...')
        rclpy.shutdown()

def main(args=None):
    """
    The main function initializes a ROS node, creates a WallFollowingBot controller, spins the
    controller, handles keyboard interrupts, and shuts down the node.
    
    :param args: The `args` parameter in the `main` function is used to pass command-line arguments to
    the program when it is executed. These arguments can be used to configure or customize the behavior
    of the program at runtime. In this case, the `args` parameter is being passed to the `rcl
    """
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