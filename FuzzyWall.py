import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy
class FuzzyImplementation:
    def __init__(self) -> None:
        self.rule_base = [
            # If both sensors are "Near," prioritize turning away from the wall.
            ("Near", "Near", "Slow", "Right"),  # Both sides near: turn right.
            
            # If the front-right is "Near" but back-right is "Medium" or "Far," move forward slowly to avoid the wall.
            ("Near", "Medium", "Slow", "Forward"),
            ("Near", "Far", "Slow", "Forward"),
            
            # If the front-right is "Medium," turn left slightly.
            ("Medium", "Near", "Medium", "Left"),  # Right-forward is medium: avoid the wall.
            
            # When both sensors are "Medium," prioritize moving forward at medium speed.
            ("Medium", "Medium", "Medium", "Forward"),
            
            # If back-right is "Far" but the front is "Medium," slow down and avoid the wall.
            ("Medium", "Far", "Slow", "Forward"),
            
            # If front-right is "Far," adjust to stay near the wall by turning left.
            ("Far", "Near", "Medium", "Left"),  # Far front-right, near back-right: turn left.
            ("Far", "Medium", "Medium", "Left"),  # Far front-right, medium back-right: adjust left.
            
            # When both sensors are "Far," move forward at high speed (no obstacles nearby).
            ("Far", "Far", "Fast", "Forward"),
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
            forward_condition, backward_condition, speed_output, direction_output = rule

            # Get the membership values for the current conditions
            forward_value = forward_membership.get(forward_condition, 0.0)
            backward_value = backward_membership.get(backward_condition, 0.0)

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
            "Left": (-2.0, -0.5),
            "Forward": (-0.5, 0.5),
            "Right": (0.5, 2.0),
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

class WallFollowingBot(Node):
    def __init__(self):
        super().__init__('wall_following_bot')

        self.fuzzy = FuzzyImplementation()  # Instantiate fuzzy logic class
        self.desired_distance = 0.25  # Target distance from the wall

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub_ = self.create_subscription(LaserScan, '/scan', self.distance_callback, qos)

        self.right_forward_distance = None
        self.right_backward_distance = None

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
        """Callback to process LaserScan messages and extract sensor distances."""
        
        self.right_forward_distance = self.find_nearest(msg.ranges[310:320])
        self.right_backward_distance = self.find_nearest(msg.ranges[210:220])


    def movement(self):
        if self.right_forward_distance is None or self.right_backward_distance is None:
            self.get_logger().warn("Sensor data not available yet.")
            return

        # Fuzzy logic inference
        right_membership = self.fuzzy.sensor_membership(self.right_forward_distance, self.desired_distance)
        back_membership = self.fuzzy.sensor_membership(self.right_backward_distance, self.desired_distance)
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

        if self.right_forward_distance < self.desired_distance:
            # Generate and publish Twist message
            twist = Twist()
            twist.linear.x = speed
            twist.angular.z = direction * -1
            self.pub_.publish(twist)
            return
        
        twist = Twist()
        twist.linear.x = speed  # Constant forward speed
        twist.angular.z = 0.0

        # Publish the velocity command to the robot
        self.pub_.publish(twist)


def main(args=None):
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