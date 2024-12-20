import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy

class GeneralFunctions:
    def __init__(self) -> None:
        pass

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
    
    def get_middle_of_range(self, key, ranges):
        """
        This Python function calculates the middle value of a given range based on a specified key.
        
        :param key: The `key` parameter is the key for which you want to find the middle of the range in the
        `ranges` dictionary
        :param ranges: The `ranges` parameter in the `get_middle_of_range` function is expected to be a
        dictionary where the keys are identifiers and the values are tuples representing a range. Each tuple
        should contain two elements - the start and end of the range
        :return: The function `get_middle_of_range` returns the middle value of the range specified by the
        key in the `ranges` dictionary. If the key is found in the `ranges` dictionary, it calculates the
        middle value of the range and returns it. If the key is not found in the `ranges` dictionary, it
        raises a `ValueError` with the message "Invalid key."
        """

        if key in ranges:
            start, end = ranges[key]
            return (start + end) / 2
        else:
            raise ValueError("Invalid key.")

    def defuzzify(self, memberships, firing_strength_sum, ranges):
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
        weighted_sum = 0.0

        for key, value in memberships.items():
            middle = self.get_middle_of_range(key, ranges)
            for item in value:
                weighted_sum += middle * item

        return weighted_sum / firing_strength_sum if firing_strength_sum > 0 else 1.0
    

class FuzzyRightWall:
    def __init__(self) -> None:
        self.rule_base = [
            ("Near", "Near", "Slow", "Left"),            
            ("Near", "Medium", "Slow", "Left"),
            ("Near", "Far", "Slow", "Left"),            
            ("Medium", "Near", "Medium", "Forward"), 
            ("Medium", "Medium", "Medium", "Forward"),            
            ("Medium", "Far", "Medium", "Forward"),            
            ("Far", "Near", "Fast", "Right"), 
            ("Far", "Medium", "Fast", "Right"),             
            ("Far", "Far", "Fast", "Right"),
        ]

    def make_inference(self, forward_membership, backward_membership):
        """
        Perform fuzzy inference using a rule base and sensor memberships.
        
        Parameters:
            rule_base (list of tuples): A list of rules in the format (Forward Condition, Backward Condition, Speed, Direction).
            forward_membership (dict): Membership values for the forward sensor (Near, Medium, Far).
            backward_membership (dict): Membership values for the backward sensor (Near, Medium, Far).
            
        Returns:
            dict: Aggregated memberships for 'Speed' and 'Direction'.
        """
        # Initialize the output memberships for Speed and Direction
        output_membership = {"Speed": {}, "Direction": {}}
        firing_strength_sum = 0

        # Iterate through the rule base
        for rule in self.rule_base:
            front_right_condition, right_back_condition, speed_output, direction_output = rule

            # Get the membership values for the current conditions
            forward_value = forward_membership.get(front_right_condition, 0.0)
            backward_value = backward_membership.get(right_back_condition, 0.0)

            # The rule's firing strength is the minimum of the two memberships
            firing_strength = min(forward_value, backward_value)
            firing_strength_sum += firing_strength

            # Update the output memberships based on the firing strength
            if firing_strength > 0:
                # Update Speed output
                if speed_output not in output_membership["Speed"]:
                    output_membership["Speed"][speed_output] = [firing_strength]
                else:
                    output_membership["Speed"][speed_output].append(firing_strength)

                # Update Direction output
                if direction_output not in output_membership["Direction"]:
                    output_membership["Direction"][direction_output] = [firing_strength]
                else:
                    output_membership["Direction"][direction_output].append(firing_strength)

        return output_membership, firing_strength_sum

class FuzzyObstacleAvoidance:
    
    def __init__(self) -> None:
        # R, F, L, S, D
       self.rule_base = [
            # Near Proximity (All cases where something is Near)
            ("Near", "Near", "Near", "Slow", "Left"),  
            ("Near", "Near", "Medium", "Slow", "Left"),
            ("Near", "Near", "Far", "Slow", "Left"),
            ("Near", "Medium", "Near", "Slow", "Left"),  
            ("Near", "Medium", "Medium", "Slow", "Left"),
            ("Near", "Medium", "Far", "Slow", "Left"),
            ("Near", "Far", "Near", "Slow", "Left"),  
            ("Near", "Far", "Medium", "Slow", "Left"),
            ("Near", "Far", "Far", "Slow", "Left"),

            # Medium Proximity (All cases where something is Medium)
            ("Medium", "Near", "Near", "Medium", "Forward"),
            ("Medium", "Near", "Medium", "Medium", "Forward"),
            ("Medium", "Near", "Far", "Medium", "Forward"),  
            ("Medium", "Medium", "Near", "Medium", "Forward"),
            ("Medium", "Medium", "Medium", "Medium", "Forward"),
            ("Medium", "Medium", "Far", "Medium", "Forward"),
            ("Medium", "Far", "Near", "Medium", "Forward"),
            ("Medium", "Far", "Medium", "Medium", "Forward"),
            ("Medium", "Far", "Far", "Medium", "Forward"),

            # Far Proximity (All cases where something is Far)
            ("Far", "Near", "Near", "Fast", "Right"),
            ("Far", "Near", "Medium", "Fast", "Right"),
            ("Far", "Near", "Far", "Fast", "Right"),
            ("Far", "Medium", "Near", "Fast", "Right"),
            ("Far", "Medium", "Medium", "Fast", "Right"),
            ("Far", "Medium", "Far", "Fast", "Right"),
            ("Far", "Far", "Near", "Fast", "Right"),
            ("Far", "Far", "Medium", "Fast", "Right"),
            ("Far", "Far", "Far", "Fast", "Right"),
        ]
     
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

class FuzzyCombination(Node):
    def __init__(self):
        super().__init__('master_bot')

        self.fuzzy = GeneralFunctions()
        self.fuzzy_avoidance = FuzzyObstacleAvoidance()
        self.fuzzy_right_wall = FuzzyRightWall()

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub_ = self.create_subscription(LaserScan, '/scan', self.distance_callback, qos)

        self.front_right_distance = None
        self.right_back_distance = None
        self.front_distance = None
        self.right_distance = None
        self.left_distance = None
        self.current_speed = 0.0

        self.desired_distance = 0.5  # Target distance from the wall

        self.ranges = {
            "Slow": (0.0, 0.04),
            "Medium": (0.04, 0.06),
            "Fast": (0.06, 0.1),
            "Right": (-1.0, -0.1),
            "Forward": (-0.1, 0.1),
            "Left": (0.1, 1),
        }

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
        The `distance_callback` function extracts and stores distance values from specific ranges in a
        message.
        
        :param msg: It looks like the `distance_callback` function is processing a message `msg` that
        contains information about distances from a sensor array. The function extracts specific ranges of
        distances from the `msg.ranges` array and assigns them to different variables for further processing
        """
        self.front_right_distance = self.find_nearest(msg.ranges[300:340])
        self.right_back_distance = self.find_nearest(msg.ranges[210:230])
        
        self.front_distance = self.find_nearest(msg.ranges[355:360])
        self.right_distance = self.find_nearest(msg.ranges[265:285])
        self.left_distance = self.find_nearest(msg.ranges[85:105])
        
    def movement(self):
        """
        The `movement` function uses fuzzy logic to determine robot movement based on sensor data,
        prioritizing obstacle avoidance or right wall following.
        :return: If the sensor data is incomplete or if there is no firing strength from the fuzzy rules,
        the robot will stop and no movement commands will be published. Otherwise, the robot will either
        prioritize obstacle avoidance or right wall following based on the distances measured by the sensors
        and the fuzzy logic calculations. The movement commands (linear and angular velocities) will be
        published accordingly.
        """
        
        if self.front_distance is None or self.right_distance is None or self.left_distance is None or self.front_right_distance is None or self.right_back_distance is None:
            self.get_logger().warn("Incomplete sensor data, stopping robot.")
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.pub_.publish(twist)
            return

        front_right_membership = self.fuzzy.sensor_membership(self.front_right_distance)
        front_back_membership = self.fuzzy.sensor_membership(self.right_back_distance)
        fuzzy_right_wall_output, fuzzy_right_wall_firing_strength_sum = self.fuzzy_right_wall.make_inference(front_right_membership, front_back_membership)

        front_membership = self.fuzzy.sensor_membership(self.front_distance)
        right_membership = self.fuzzy.sensor_membership(self.right_distance)
        left_membership = self.fuzzy.sensor_membership(self.left_distance)
        fuzzy_avoidance_output, fuzzy_avoidance_firing_strength_sum = self.fuzzy_avoidance.make_inference(right_membership, front_membership, left_membership)

        if fuzzy_right_wall_firing_strength_sum < 0 and fuzzy_avoidance_firing_strength_sum < 0:
            self.get_logger().warn("No firing strength from fuzzy rules.")
            return

        # Defuzzify
        fuzzy_right_wall_speed = self.fuzzy.defuzzify(fuzzy_right_wall_output.get("Speed", {}), fuzzy_right_wall_firing_strength_sum, self.ranges)
        fuzzy_right_wall_direction = self.fuzzy.defuzzify(fuzzy_right_wall_output.get("Direction", {}), fuzzy_right_wall_firing_strength_sum, self.ranges)

        fuzzy_avoidance_speed = self.fuzzy.defuzzify(fuzzy_avoidance_output.get("Speed", {}), fuzzy_avoidance_firing_strength_sum, self.ranges)
        fuzzy_avoidance_direction = self.fuzzy.defuzzify(fuzzy_avoidance_output.get("Direction", {}), fuzzy_avoidance_firing_strength_sum, self.ranges)

        d1 = min(self.front_right_distance, self.right_back_distance)
        d2 = min(self.front_distance, self.right_distance, self.left_distance)


        self.get_logger().info(f"Right Distance: {self.right_distance}, Front Distance: {self.front_distance}, Left Distance: {self.left_distance}")
        self.get_logger().info(f"Right: {right_membership}, Front: {front_membership}, Left: {left_membership}")
        self.get_logger().info(f"ALMS: {fuzzy_avoidance_speed}, ARMS: {fuzzy_avoidance_direction}")

        self.get_logger().info(f"Front Right Distance: {self.front_right_distance}, Right Back Distance: {self.right_back_distance}")
        self.get_logger().info(f"RIGHT: {front_right_membership}, BACK: {front_back_membership}")
        self.get_logger().info(f"RLMS: {fuzzy_right_wall_speed}, RRMS: {fuzzy_right_wall_direction}")

        self.get_logger().info(f"D1: {d1}, D2: {d2}")

        if d2 <= 0.25:
            self.get_logger().info("Obstacle Avoidance Priority...")
            twist = Twist()
            twist.linear.x = fuzzy_avoidance_speed
            twist.angular.z = fuzzy_avoidance_direction
        else :
            self.get_logger().info("Right Wall Following Priority...")
            twist = Twist()
            twist.linear.x = fuzzy_right_wall_speed
            twist.angular.z = fuzzy_right_wall_direction  


        self.pub_.publish(twist)


def main(args=None):
    """
    The main function initializes a ROS node, creates an instance of the FuzzyCombination class, and
    then spins the node until a KeyboardInterrupt is received, at which point it shuts down the node.
    
    :param args: In the provided code snippet, the `args` parameter in the `main` function is used to
    pass command-line arguments to the `rclpy.init()` function. This allows you to customize the
    initialization of the ROS client library (rclpy) based on the arguments provided when running the
    script
    """
    rclpy.init(args=args)
    bot = FuzzyCombination()

    try:
        rclpy.spin(bot)
    except KeyboardInterrupt:
        bot.get_logger().info("Shutting down...")
    finally:
        bot.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()