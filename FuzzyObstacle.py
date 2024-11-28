import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy

class FuzzyObstacleAvoidance:
    
    def __init__(self) -> None:
        self.rule_base = [
            # Rules for avoiding obstacles
            ("Near", "Near", "Near", "Slow", "Right"),    # All near: turn right, slow down
            ("Near", "Near", "Medium", "Slow", "Right"),  # Front and front-right near, left medium: turn right, slow
            ("Near", "Medium", "Near", "Slow", "Right"),  # Front and left near: sharp right turn
            ("Near", "Far", "Near", "Slow", "Right"),     # Front near, left near, front-right far: slow and sharp right
            ("Near", "Far", "Medium", "Slow", "Left"),    # Front near, left medium, front-right far: avoid right
            
            ("Medium", "Near", "Near", "Medium", "Left"), # Front-right and left near, front medium: turn left
            ("Medium", "Medium", "Medium", "Medium", "Forward"), # Medium distances all around: move forward cautiously
            ("Medium", "Medium", "Far", "Medium", "Forward"),    # Medium front, far left, front-right: proceed cautiously
            ("Medium", "Far", "Medium", "Medium", "Right"),      # Medium front, medium left, far front-right: turn slightly right
            
            ("Far", "Near", "Near", "Medium", "Right"),          # Far front, near front-right, left near: avoid left
            ("Far", "Medium", "Near", "Medium", "Right"),        # Far front, medium front-right, near left: slight right turn
            ("Far", "Medium", "Medium", "Fast", "Forward"),      # Far front, medium everywhere else: proceed quickly forward
            ("Far", "Far", "Far", "Fast", "Forward"),            # All far: move forward quickly
        ]

    def remove_zero_memberships(self, membership):
        return {key: value for key, value in membership.items() if value != 0}

    def rising_edge(self, x, a, b):
        if x < a:
            return 0.0
        elif x > b:
            return 1.0
        return (x - a) / (b - a)

    def falling_edge(self, x, b, c):
        if x < b:
            return 1.0
        elif x > c:
            return 0.0
        return (c - x) / (c - b)

    def sensor_membership(self, distance):
        """
        Compute membership values for 'Near', 'Medium', and 'Far' fuzzy sets for a sensor.
        """
        if distance < 0:
            raise ValueError("Distance must be non-negative.")

        membership = {"Near": 0.0, "Medium": 0.0, "Far": 0.0}

        if 0 <= distance <= 0.5:
            membership["Near"] = 1.0
        elif 0.5 < distance <= 1.0:
            membership["Near"] = self.falling_edge(distance, 0.5, 1.0)
            membership["Medium"] = self.rising_edge(distance, 0.5, 1.0)
        elif 1.0 < distance <= 1.5:
            membership["Medium"] = self.falling_edge(distance, 1.0, 1.5)
            membership["Far"] = self.rising_edge(distance, 1.0, 1.5)
        elif distance > 1.5:
            membership["Far"] = 1.0

        return self.remove_zero_memberships(membership)

    def make_inference(self, front_membership, front_right_membership, left_membership):
        """
        Perform fuzzy inference using a rule base and sensor memberships.
        """
        output_membership = {"Speed": {}, "Direction": {}}
        firing_strength_sum = 0

        for rule in self.rule_base:
            front_condition, front_right_condition, left_condition, speed_output, direction_output = rule

            front_value = front_membership.get(front_condition, 0.0)
            front_right_value = front_right_membership.get(front_right_condition, 0.0)
            left_value = left_membership.get(left_condition, 0.0)

            firing_strength = min(front_value, front_right_value, left_value)
            firing_strength_sum += firing_strength

            if firing_strength > 0:
                if speed_output not in output_membership["Speed"]:
                    output_membership["Speed"][speed_output] = [firing_strength]
                else:
                    output_membership["Speed"][speed_output].append(firing_strength)

                if direction_output not in output_membership["Direction"]:
                    output_membership["Direction"][direction_output] = [firing_strength]
                else:
                    output_membership["Direction"][direction_output].append(firing_strength)

        return output_membership, firing_strength_sum

    def get_middle_of_range(self, key):
        ranges = {
            "Slow": (0.0, 0.3),
            "Medium": (0.3, 0.6),
            "Fast": (0.6, 1.0),
            "Left": (-2.0, -1.0),
            "Forward": (-1.0, 1.0),
            "Right": (1.0, 2.0),
        }

        if key in ranges:
            start, end = ranges[key]
            return (start + end) / 2
        else:
            raise ValueError("Invalid key.")

    def defuzzify(self, memberships, firing_strength_sum):
        weighted_sum = 0

        for key, value in memberships.items():
            middle = self.get_middle_of_range(key)
            for item in value:
                weighted_sum += middle * item

        return weighted_sum / firing_strength_sum


class ObstacleAvoidanceBot(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_bot')

        self.fuzzy = FuzzyObstacleAvoidance()

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub_ = self.create_subscription(LaserScan, '/scan', self.distance_callback, qos)

        self.front_distance = None
        self.front_right_distance = None
        self.left_distance = None

        self.pub_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer_ = self.create_timer(0.1, self.movement)

    def find_nearest(self, l):
        f_list = list(filter(lambda item: item > 0.0, l))
        return min(f_list) if f_list else None

    def distance_callback(self, msg):
        self.front_distance = self.find_nearest(msg.ranges[0:5] + msg.ranges[355:360])
        self.front_right_distance = self.find_nearest(msg.ranges[310:320])
        self.left_distance = self.find_nearest(msg.ranges[85:95])

    def movement(self):
        if self.front_distance is None or self.front_right_distance is None or self.left_distance is None:
            self.get_logger().warn("Sensor data not available yet.")
            return

        # Fuzzy logic inference
        front_membership = self.fuzzy.sensor_membership(self.front_distance)
        front_right_membership = self.fuzzy.sensor_membership(self.front_right_distance)
        left_membership = self.fuzzy.sensor_membership(self.left_distance)
        output, firing_strength_sum = self.fuzzy.make_inference(front_membership, front_right_membership, left_membership)

        if firing_strength_sum == 0:
            self.get_logger().warn("No firing strength from fuzzy rules.")
            return

        # Defuzzify
        speed = self.fuzzy.defuzzify(output.get("Speed", {}), firing_strength_sum)
        direction = self.fuzzy.defuzzify(output.get("Direction", {}), firing_strength_sum)

        # Publish Twist message
        twist = Twist()
        twist.linear.x = speed
        twist.angular.z = direction
        self.pub_.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    bot = ObstacleAvoidanceBot()

    try:
        rclpy.spin(bot)
    except KeyboardInterrupt:
        bot.get_logger().info("Shutting down...")
    finally:
        bot.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
