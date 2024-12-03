import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy


class FuzzyRightWall:
    def __init__(self) -> None:
        self.rule_base = [
            ("Near", "Near", "Medium", "Right"),            
            ("Near", "Medium", "Slow", "Right"),
            ("Near", "Far", "Medium", "Left"),            
            ("Medium", "Near", "Slow", "Forward"), 
            ("Medium", "Medium", "Medium", "Forward"),            
            ("Medium", "Far", "Slow", "Left"),            
            ("Far", "Near", "Slow", "Left"), 
            ("Far", "Medium", "Medium", "Left"),             
            ("Far", "Far", "Fast", "Left"),
        ]
    
    def remove_zero_memberships(self, membership):
        """
        Remove dictionary entries with zero values.
        
        Parameters:
            membership (dict): The membership dictionary.
            
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

    def sensor_membership(self, distance, desired_distance=0.25):
        """
        Compute the membership values for 'Near', 'Medium', and 'Far' fuzzy sets
        considering the desired distance. This method scales the membership values based on the desired distance

        membershipRange = {
            "Near": [0.0, 0.25. 0.75], 
            "Medium": [0.25, 0.75. 1.05], 
            "Far": [0.75, 1.05, >1.05]
            }
        """

        if distance < 0:
            raise ValueError("Distance must be non-negative.")
        
        membership = {"Near": 0.0, "Medium": 0.0, "Far": 0.0}
        
        # Dynamically adjust ranges based on desired distance
        near_threshold = desired_distance + 0.5
        medium_threshold = desired_distance + 0.8

        if 0 <= distance <= near_threshold:
            membership["Near"] = 1.0
        elif near_threshold < distance <= desired_distance:
            membership["Near"] = self.falling_edge(distance, near_threshold, desired_distance)
            membership["Medium"] = self.rising_edge(distance, near_threshold, desired_distance)
        elif near_threshold < distance <= medium_threshold:
            membership["Medium"] = self.falling_edge(distance, desired_distance, medium_threshold)
            membership["Far"] = self.rising_edge(distance, desired_distance, medium_threshold)
        elif distance > medium_threshold:
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
        
        Parameters:
            key (str): The fuzzy set key ('Slow', 'Medium', 'Fast').
            
        Returns:
            float: The middle of the range for the given fuzzy set.
        """
        ranges = {
            "Slow": (0.0, 0.3),
            "Medium": (0.3, 0.6),
            "Fast": (0.6, 1.0),
            "Left": (-2.0, -1.0),
            "Forward": (1.0, 1.0),
            "Right": (1.0, 2.0),
        }
        
        if key in ranges:
            start, end = ranges[key]
            return (start + end) / 2  # Calculate the midpoint
        else:
            print(key)
            raise ValueError("Invalid key.")

    def defuzzify(self, memberships, firing_strength_sum):
            """Compute the weighted sum for a set of fuzzy memberships."""
            weighted_sum = 0
            
            for key, value in memberships.items():
                middle = self.get_middle_of_range(key)  # Get the middle value of the range
                for item in value:
                    weighted_sum += middle * item

            return weighted_sum / firing_strength_sum

class FuzzyObstacleAvoidance:
    
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
        return {key: value for key, value in membership.items() if value != 0}

    def rising_edge(self, x, a, b):
        if x < a:
            return 0.0
        elif x > b:
            return 1.0
        return (x - a) / (b - a)

    def falling_edge(self, x, b, C):
        if x < b:
            return 1.0
        elif x > C:
            return 0.0
        return (C - x) / (C - b)

    def sensor_membership(self, distance, desired_distance=0.25):
        """
        Compute the membership values for 'Near', 'Medium', and 'Far' fuzzy sets
        considering the desired distance.
        """
        if distance < 0:
            raise ValueError("Distance must be non-negative.")
        
        membership = {"Near": 0.0, "Medium": 0.0, "Far": 0.0}
        
        # Dynamically adjust ranges based on desired distance
        near_threshold = desired_distance + 0.5
        medium_threshold = desired_distance + 0.8

        if 0 <= distance <= near_threshold:
            membership["Near"] = 1.0
        elif near_threshold < distance <= desired_distance:
            membership["Near"] = self.falling_edge(distance, near_threshold, desired_distance)
            membership["Medium"] = self.rising_edge(distance, near_threshold, desired_distance)
        elif desired_distance < distance <= medium_threshold:
            membership["Medium"] = self.falling_edge(distance, desired_distance, medium_threshold)
            membership["Far"] = self.rising_edge(distance, desired_distance, medium_threshold)
        elif distance > medium_threshold:
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
        ranges = {
            "Slow": (0.0, 0.3),
            "Medium": (0.3, 0.6),
            "Fast": (0.6, 1.0),
            "Left": (-2.0, -1.0),
            "Forward": (1.0, 1.0),
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
    
class Master(Node):
    def __init__(self):
        super().__init__('master_bot')

        self.fuzzy_avoidance = FuzzyObstacleAvoidance()
        self.fuzzy_right_wall = FuzzyRightWall()

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub_ = self.create_subscription(LaserScan, '/scan', self.distance_callback, qos)

        self.front_right_distance = None
        self.right_back_distance = None
        self.front_distance = None
        self.right_distance = None
        self.left_distance = None

        self.desired_distance = 0.4  # Target distance from the wall

        self.pub_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer_ = self.create_timer(0.1, self.movement)

    def find_nearest(self, l):
        """Return the nearest non-zero distance from a list"""
        f_list = list(filter(lambda item: item > 0.0, l))
        if f_list:
            return min(f_list)
        else:
            return 0.0
    
    def distance_callback(self, msg):
        self.front_right_distance = self.find_nearest(msg.ranges[310:320])
        self.right_back_distance = self.find_nearest(msg.ranges[210:220])
        self.front_distance = self.find_nearest(msg.ranges[355:360])
        self.right_distance = self.find_nearest(msg.ranges[265:275])
        self.left_distance = self.find_nearest(msg.ranges[85:95])
        
    def blended_control(self, fuzzy_wall_speed, fuzzy_wall_direction, fuzzy_avoidance_speed, fuzzy_avoidance_direction):
        # Weighting factors
        wall_weight = 0.6
        avoidance_weight = 0.4

        # Blend speeds and directions
        blended_speed = (fuzzy_wall_speed * wall_weight + fuzzy_avoidance_speed * avoidance_weight)
        blended_direction = (fuzzy_wall_direction * wall_weight + fuzzy_avoidance_direction * avoidance_weight)

        return blended_speed, blended_direction
    
    def gradual_control(self, target_speed):
        max_step = 0.05  # Maximum change in speed per iteration
        if abs(self.current_speed - target_speed) > max_step:
            self.current_speed += max_step if target_speed > self.current_speed else -max_step
        else:
            self.current_speed = target_speed
        return self.current_speed

    def movement(self):
        
        if self.front_distance is None or self.right_distance is None or self.left_distance is None or self.front_right_distance is None or self.right_back_distance is None:
            self.get_logger().warn("Incomplete sensor data, stopping robot.")
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.pub_.publish(twist)
            return

        right_membership = self.fuzzy_right_wall.sensor_membership(self.front_right_distance, self.desired_distance)
        back_membership = self.fuzzy_right_wall.sensor_membership(self.right_back_distance, self.desired_distance)
        fuzzy_right_wall_output, fuzzy_right_wall_firing_strength_sum = self.fuzzy_right_wall.make_inference(right_membership, back_membership)

        front_membership = self.fuzzy_avoidance.sensor_membership(self.front_distance, self.desired_distance)
        right_membership = self.fuzzy_avoidance.sensor_membership(self.right_distance, self.desired_distance)
        left_membership = self.fuzzy_avoidance.sensor_membership(self.left_distance, self.desired_distance)
        fuzzy_avoidance_output, fuzzy_avoidance_firing_strength_sum = self.fuzzy_avoidance.make_inference(right_membership, front_membership, left_membership)

        if fuzzy_right_wall_firing_strength_sum == 0 or fuzzy_avoidance_firing_strength_sum == 0:
            self.get_logger().warn("No firing strength from fuzzy rules.")
            return

        # Defuzzify
        fuzzy_right_wall_speed = self.fuzzy_right_wall.defuzzify(fuzzy_right_wall_output.get("Speed", {}), fuzzy_right_wall_firing_strength_sum)
        fuzzy_right_wall_direction = self.fuzzy_right_wall.defuzzify(fuzzy_right_wall_output.get("Direction", {}), fuzzy_right_wall_firing_strength_sum)

        fuzzy_avoidance_speed = self.fuzzy_avoidance.defuzzify(fuzzy_avoidance_output.get("Speed", {}), fuzzy_avoidance_firing_strength_sum)
        fuzzy_avoidance_direction = self.fuzzy_avoidance.defuzzify(fuzzy_avoidance_output.get("Direction", {}), fuzzy_avoidance_firing_strength_sum)

        self.get_logger().info(f"Right Distance: {self.right_distance}, Front Distance: {self.front_distance}, Left Distance: {self.left_distance}")
        self.get_logger().info(f"Right: {right_membership}, Front: {front_membership}, Left: {left_membership}")

        
        if self.front_distance < self.desired_distance or self.front_right_distance < self.desired_distance:
            self.get_logger().info("Obstacle Avoidance Priority...")
            twist = Twist()
            twist.linear.x = fuzzy_avoidance_speed
            twist.angular.z = fuzzy_avoidance_direction
        elif self.front_right_distance > self.desired_distance and self.right_back_distance > self.desired_distance:
            self.get_logger().info("Right Wall Following Priority...")
            twist = Twist()
            twist.linear.x = fuzzy_right_wall_speed
            twist.angular.z = fuzzy_right_wall_direction
        else:
            self.get_logger().info("Blended Control...")
            twist = Twist()
            blended_speed, blended_direction = self.blended_control(
                fuzzy_right_wall_speed, fuzzy_right_wall_direction,
                fuzzy_avoidance_speed, fuzzy_avoidance_direction
            )
            twist.linear.x = self.gradual_control(blended_speed)
            twist.angular.z = blended_direction

        self.publish_debug_info(f"Right Distance: {self.right_distance}, Front Distance: {self.front_distance}, Left Distance: {self.left_distance}")
        self.pub_.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    bot = Master()

    try:
        rclpy.spin(bot)
    except KeyboardInterrupt:
        bot.get_logger().info("Shutting down...")
    finally:
        bot.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()