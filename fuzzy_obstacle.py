import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy

class FuzzyObstacleAvoidance:
    # R, F, L, S, D
    def __init__(self) -> None:
       self.rule_base = [
            # Near Proximity (All cases where something is Near)
            ("Near", "Near", "Near", "Slow", "Right"),  
            ("Near", "Near", "Medium", "Slow", "Right"),
            ("Near", "Near", "Far", "Slow", "Right"),
            ("Near", "Medium", "Near", "Slow", "Left"),  
            ("Near", "Medium", "Medium", "Slow", "Right"),
            ("Near", "Medium", "Far", "Slow", "Left"),
            ("Near", "Far", "Near", "Fast", "Forward"),  
            ("Near", "Far", "Medium", "Fast", "Left"),
            ("Near", "Far", "Far", "Fast", "Forward"),

            # Medium Proximity (All cases where something is Medium)
            ("Medium", "Near", "Near", "Slow", "Right"),
            ("Medium", "Near", "Medium", "Slow", "Right"),
            ("Medium", "Near", "Far", "Slow", "Left"),  
            ("Medium", "Medium", "Near", "Slow", "Right"),
            ("Medium", "Medium", "Medium", "Slow", "Forward"),
            ("Medium", "Medium", "Far", "Slow", "Left"),
            ("Medium", "Far", "Near", "Fast", "Right"),
            ("Medium", "Far", "Medium", "Fast", "Right"),
            ("Medium", "Far", "Far", "Fast", "Left"),

            # Far Proximity (All cases where something is Far)
            ("Far", "Near", "Near", "Slow", "Right"),
            ("Far", "Near", "Medium", "Slow", "Right"),
            ("Far", "Near", "Far", "Slow", "Right"),
            ("Far", "Medium", "Near", "Fast", "Right"),
            ("Far", "Medium", "Medium", "Fast", "Forward"),
            ("Far", "Medium", "Far", "Fast", "Left"),
            ("Far", "Far", "Near", "Fast", "Right"),
            ("Far", "Far", "Medium", "Fast", "Forward"),
            ("Far", "Far", "Far", "Fast", "Forward"),
        ]
       
    def remove_zero_memberships(self, membership):
        """
        Remove dictionary entries with zero values.
        Parameters: membership (dict): The membership dictionary.
        Returns:
            dict: A dictionary without zero-value entries.
        """
        return {key: value for key, value in membership.items() if value != 0}

    def rising_edge(self, x, a, b):
        """Calculate membership using a rising edge formula."""
        if x < a:
            return 0.0
        elif x > b:
            return 1.0
        return (x - a) / (b - a)

    def falling_edge(self, x, b, C):
        """Calculate membership using a falling edge formula."""
        if x < b:
            return 1.0
        elif x > C:
            return 0.0
        return (C - x) / (C - b)

    def sensor_membership(self, distance):
        """
        Compute the membership values for 'Near', 'Medium', and 'Far' fuzzy sets
        considering the desired distance. This method scales the membership values based on the desired distance

        membershipRange = {
            "Near":  [-0.4 0.2 0.45], 
            "Medium":  [0.2 0.45 0.7], 
            "Far":  [0.45 0.7 2]
            }
        """
        membership = {"Near": 0.0, "Medium": 0.0, "Far": 0.0}

        near_range = [-0.4, 0.2, 0.45]
        medium_range = [0.2, 0.45, 0.7]
        far_range = [0.45, 0.7, 2]

        if distance < 0:
            membership["Near"] = 1.0  
        
        # Calculate the updated thresholds based on the new membership ranges
        near_threshold = near_range[2]  # The upper value for "Near"
        medium_threshold = medium_range[2]  # The upper value for "Medium"

        # Membership assignment based on updated ranges
        if 0 <= distance <= near_threshold:
            membership["Near"] = 1.0
        elif near_threshold < distance <= near_range[1]:  # In this range, use the falling edge for "Near"
            membership["Near"] = self.falling_edge(distance, near_range[1], near_threshold)
            membership["Medium"] = self.rising_edge(distance, near_range[1], near_threshold)
        elif near_range[1] < distance <= medium_range[1]:  # In this range, use the falling edge for "Medium"
            membership["Medium"] = self.falling_edge(distance, near_range[1], medium_range[1])
            membership["Far"] = self.rising_edge(distance, near_range[1], medium_range[1])
        elif distance > medium_threshold:  # If distance is greater than the upper range for "Medium"
            membership["Far"] = 1.0

        return self.remove_zero_memberships(membership)

    def make_inference(self, right_membership, front_membership, left_membership):
        """
        Perform fuzzy inference using a rule base and sensor memberships.
        """
        output_membership = {"Speed": {}, "Direction": {}}
        firing_strength_sum = 0

        for rule in self.rule_base:
            right_condition, front_condition, left_condition, speed_output, direction_output = rule

            right_value = right_membership.get(right_condition, 0.0)
            front_value = front_membership.get(front_condition, 0.0)
            left_value = left_membership.get(left_condition, 0.0)

            firing_strength = min(front_value, right_value, left_value)
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
        """
        Return the middle point of the range corresponding to the given key.
        Parameters: key (str): The fuzzy set key ('Slow', 'Medium', 'Fast', ...).
        Returns: float: The middle of the range for the given fuzzy set.
        """
        ranges = {
            "Slow": (0.0, 0.04),
            "Medium": (0.04, 0.06),
            "Fast": (0.06, 0.1),
            "Right": (-1.0, -0.1),
            "Forward": (-0.1, 0.1),
            "Left": (0.1, 1),
        }

        if key in ranges:
            start, end = ranges[key]
            return (start + end) / 2
        else:
            raise ValueError("Invalid key.")

    def defuzzify(self, memberships, firing_strength_sum):
        """
        The `defuzzify` function calculates the weighted sum of the middle values of fuzzy sets based on
        their memberships and returns the result divided by the sum of firing strengths.
        
        :param memberships: The `memberships` parameter seems to be a dictionary where the keys represent
        fuzzy sets or linguistic terms, and the values are lists of membership values for each linguistic
        term
        :param firing_strength_sum: The `firing_strength_sum` parameter represents the total sum of firing
        strengths of the fuzzy rules that have fired. This value is used in defuzzification to calculate the
        final crisp output value by dividing the weighted sum by the `firing_strength_sum`
        :return: the defuzzified output value, which is calculated by summing up the weighted values of the
        middle of each range multiplied by their corresponding membership values, and then dividing this sum
        by the total firing strength sum.
        """
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
        self.right_distance = None
        self.left_distance = None

        self.desired_distance = 0.5  # Target distance from the wall

        self.pub_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer_ = self.create_timer(0.1, self.movement)

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
        filtered_list = filter(lambda item: item > 0.0, l)
        return min(min(filtered_list, default=1), 1)
    
    def distance_callback(self, msg):
        """
        The `distance_callback` function calculates the nearest distances from specific ranges in a message.
        
        :param msg: The `msg` parameter in the `distance_callback` function seems to be a message object
        that contains information about distances from sensors. It appears to have a `ranges` attribute
        which is likely an array or list of distance values from different angles
        """
        self.front_distance = self.find_nearest(msg.ranges[355:360])
        self.right_distance = self.find_nearest(msg.ranges[265:285])
        self.left_distance = self.find_nearest(msg.ranges[85:105])

    def movement(self):
        """
        This Python function uses fuzzy logic inference to determine movement speed and direction based on
        sensor data and obstacle detection.
        :return: If any of the sensor data (front_distance, right_distance, left_distance) is None, a
        warning message "Sensor data not available yet." is logged and the function returns without
        performing any further calculations. If there is no firing strength from the fuzzy rules
        (firing_strength_sum is 0), a warning message "No firing strength from fuzzy rules." is logged and
        the function returns. Otherwise,
        """
        if self.front_distance is None or self.right_distance is None or self.left_distance is None:
            self.get_logger().warn("Sensor data not available yet.")
            return

        # Fuzzy logic inference
        front_membership = self.fuzzy.sensor_membership(self.front_distance)
        right_membership = self.fuzzy.sensor_membership(self.right_distance)
        left_membership = self.fuzzy.sensor_membership(self.left_distance)
        output, firing_strength_sum = self.fuzzy.make_inference(right_membership, front_membership, left_membership)

        if firing_strength_sum == 0:
            self.get_logger().warn("No firing strength from fuzzy rules.")
            return

        # Defuzzify
        speed = self.fuzzy.defuzzify(output.get("Speed", {}), firing_strength_sum)
        direction = self.fuzzy.defuzzify(output.get("Direction", {}), firing_strength_sum)

        self.get_logger().info(f"Right Distance: {self.right_distance}, Front Distance: {self.front_distance}, Left Distance: {self.left_distance}")
        self.get_logger().info(f"Right: {right_membership}, Front: {front_membership}, Left: {left_membership}")
        self.get_logger().info(f"Speed: {speed}, Direction: {direction}")

        d1 = min(self.front_distance, self.right_distance, self.left_distance) 

        if d1 < self.desired_distance:
            self.get_logger().info("Obstacle detected in front...")
            twist = Twist()
            twist.linear.x = speed
            twist.angular.z = direction
        else:
            twist = Twist()
            twist.linear.x = 0.1
            twist.angular.z = 0.0

        
        self.pub_.publish(twist)


def main(args=None):
    """
    The main function initializes a ROS node, creates an ObstacleAvoidanceBot instance, spins the node,
    and handles shutdown procedures.
    
    :param args: In the `main` function you provided, the `args` parameter is used to pass command-line
    arguments to the `rclpy.init()` function. These arguments can be used to configure the ROS 2 node
    when it is initialized. By passing `args=args` to `rclpy.init
    """
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
