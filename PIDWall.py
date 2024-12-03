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
        
        # Use Ku and Pu values to compute PID parameters
        # Ku = 0.01  # Ultimate Gain
        # Pu = 2.0  # Oscillation Period in seconds
        

        # # Increase the proportional gain (Kp) using Ku above until 
        # # the system starts to oscillate continuously, i.e., when 
        # # the system reaches a stable oscillation, where the output 
        # # does not grow unbounded but instead oscillates with a 
        # # constant amplitude.
        # Kp = 0.6 * Ku
        # # Dampen the rate of error change, preventing overshooting 
        # # or oscillation in the robot's behavior. Pu/8
        # Kd = 0.08 * Kp * Pu
        # # Handles accumulated error over time Pu/2
        # Ki = 1.4 * Kp / Pu
        
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
        # self.left_distance = None
        # self.right_distance = None

        self.previous_time = self.get_clock().now()

        self.pub_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Timer to update control logic periodically
        self.timer_ = self.create_timer(self.dt, self.movement)

    def find_nearest(self, l):
        """Return the nearest non-zero distance from a list"""
        f_list = list(filter(lambda item: item > 0.0, l))
        if f_list:
            return min(f_list)
        else:
            return float('inf')

    def distance_callback(self, msg):
        # Get right-side distance from the scan data
        front_ranges = min(self.find_nearest(msg.ranges[0:5]), self.find_nearest(msg.ranges[355:360]))
        self.front_distance = front_ranges
        
        right_side_range = self.find_nearest(msg.ranges[265:320])
        self.current_distance = right_side_range

    def movement(self):
        if self.current_distance is None or self.current_distance == float('inf'):
            self.get_logger().warn('Invalid right-side distance')
            return  # Skip if no valid distance data
        
        # if self.left_distance == float('inf') or self.right_distance == float('inf'):
        #     self.get_logger().warn('Invalid left or right-side distance')
        #     return  # Skip if any of the side distances are invalid

        # Calculate the time difference (dt)
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