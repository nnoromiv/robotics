import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


MAX_RANGE = 0.15

class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('reading_laser')
        
        # Initialize regions and twist message
        self.regions = {
            'right': 0,
            'fright': 0,
            'front1': 0,
            'front2': 0,
            'fleft': 0,
            'left': 0,
        }
        self.twist_msg = None

        # Set up QoS profile (with 'best effort' reliability policy)
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        
        # Create publisher for velocity commands
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Create subscription to laser topic
        self.sub = self.create_subscription(LaserScan, '/scan', self.clbk_laser, qos)
        
        # Create a timer for periodic publishing
        timer_period = 0.2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        if self.twist_msg:
            self.pub.publish(self.twist_msg)

    def clbk_laser(self, msg):
        # Update regions based on LIDAR data
        self.regions = {
            'front1': self.find_nearest(msg.ranges[0:5]),
            'front2': self.find_nearest(msg.ranges[355:360]),
            'right': self.find_nearest(msg.ranges[265:275]),
            'fright': self.find_nearest(msg.ranges[310:320]),
            'fleft': self.find_nearest(msg.ranges[40:50]),
            'left': self.find_nearest(msg.ranges[85:95])
        }
        # Update twist message with movement decision
        self.twist_msg = self.movement()

    def find_nearest(self, lst):
        # Exclude zero distances and find minimum
        filtered_list = filter(lambda item: item > 0.0, lst)
        return min(min(filtered_list, default=1), 10)

    def movement(self):
        # Create a Twist message for movement
        msg = Twist()
        
        # Check front regions for obstacles and decide on motion
        print("Min distance in front region:", self.regions['front1'], self.regions['front2'])
        if self.regions['front1'] < 0.4 or self.regions['front2'] < 0.4:
            msg.linear.x = 0.05
            msg.angular.z = -1.0
            return msg
        else:
            msg.linear.x = 0.1
            msg.angular.z = 0.0
            return msg

    def stop(self):
        # Publish a stop command
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.pub.publish(msg)

def main():
    # Initialize rclpy and create the node
    rclpy.init()
    laser_navigator = LaserNavigator()

    # Run and handle keyboard interrupts
    try:
        rclpy.spin(laser_navigator)
    except KeyboardInterrupt:
        laser_navigator.stop()
    finally:
        # Clean up
        laser_navigator.destroy_timer(laser_navigator.timer)
        laser_navigator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()