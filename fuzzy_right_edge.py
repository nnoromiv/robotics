import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy


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
            return (start + end) / 2  # Calculate the midpoint
        else:
            print(key)
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
            middle = self.get_middle_of_range(key)  # Get the middle value of the range
            for item in value:
                weighted_sum += middle * item

        return weighted_sum / firing_strength_sum

class WallFollowingBot(Node):
    def __init__(self):
        super().__init__('wall_following_bot')

        self.fuzzy = FuzzyRightWall()  # Instantiate fuzzy logic class
        self.desired_distance = 0.25  # Target distance from the wall

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub_ = self.create_subscription(LaserScan, '/scan', self.distance_callback, qos)

        self.front_right_distance = None
        self.right_back_distance = None

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
        """Callback to process LaserScan messages and extract sensor distances."""
        self.front_right_distance = self.find_nearest(msg.ranges[300:320])
        self.right_back_distance = self.find_nearest(msg.ranges[210:230])

    def movement(self):
        """
        The `movement` function uses fuzzy logic inference to determine speed and direction values for
        movement based on sensor data.
        :return: The `movement` method returns nothing explicitly, as it ends with a `return` statement
        without a value. However, if certain conditions are met (such as sensor data not being available or
        no firing strength from fuzzy rules), the method will return early without executing the final
        publishing step.
        """
        if self.front_right_distance is None or self.right_back_distance is None:
            self.get_logger().warn("Sensor data not available yet.")
            return

        # Fuzzy logic inference
        right_membership = self.fuzzy.sensor_membership(self.front_right_distance)
        back_membership = self.fuzzy.sensor_membership(self.right_back_distance)
        output, firing_strength_sum = self.fuzzy.make_inference(right_membership, back_membership)

        self.get_logger().info(f"RIGHT: {right_membership}, BACK: {back_membership}")

        if firing_strength_sum == 0:
            self.get_logger().warn("No firing strength from fuzzy rules.")
            return 0.0

        # Defuzzify to get crisp values
        speed = self.fuzzy.defuzzify(output.get("Speed", {}), firing_strength_sum)
        direction = self.fuzzy.defuzzify(output.get("Direction", {}), firing_strength_sum)

        # Log the computed values
        self.get_logger().info(f"Speed: {speed}, Direction: {direction}")

        twist = Twist()
        twist.linear.x = speed
        twist.angular.z = direction
        self.pub_.publish(twist)
        return


def main(args=None):
    """
    The main function initializes a ROS node, creates a WallFollowingBot controller, spins the
    controller, handles keyboard interrupts, and shuts down the node.
    
    :param args: In the `main` function you provided, the `args` parameter is used to pass command-line
    arguments to the `rclpy.init()` function. These arguments can be used to configure the ROS 2 runtime
    environment. When `rclpy.init(args=args)` is called, it initializes the
    """
    rclpy.init(args=args)
    controller = WallFollowingBot()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info("Shutting down...")
    finally:
        controller.destroy_node()
        rclpy.shutdown()
        
if __name__ == "__main__":
    main()
