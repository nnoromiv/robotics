import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy

# Membership function parameters for RBS and RFS
def near(x):
    return max(min((30 - x) / 30, 1), 0)

def med(x):
    return max(min((x - 20) / 20, (60 - x) / 20), 0)

def far(x):
    return max(min((x - 50) / 30, 1), 0)

# Membership function parameters for Speed
def low(x):
    return max(min((50 - x) / 50, 1), 0)

def medium(x):
    return max(min((x - 30) / 20, (70 - x) / 20), 0)

def high(x):
    return max(min((x - 60) / 40, 1), 0)

# Membership function parameters for Direction
def left(x):
    return max(min((30 - x) / 30, 1), 0)

def straight(x):
    return max(min((x - 15) / 15, (45 - x) / 15), 0)

def right(x):
    return max(min((x - 30) / 30, 1), 0)

# Rule base: [(RBS, RFS) -> (Speed, Direction)]
rule_base = [
    ("near", "near", "low", "left"),
    ("near", "med", "medium", "left"),
    ("near", "far", "medium", "straight"),
    ("med", "near", "medium", "left"),
    ("med", "med", "medium", "straight"),
    ("med", "far", "high", "straight"),
    ("far", "near", "medium", "right"),
    ("far", "med", "high", "right"),
    ("far", "far", "high", "right"),
]

# Fuzzification function for sensor values
def fuzzify(sensor_value, funcs):
    return {label: func(sensor_value) for label, func in funcs.items()}

# Fuzzification using the centroid method
def centroid_fuzzification(firing_strengths, universe, func):
    numerator = sum(strength * x * func(x) for x, strength in zip(universe, firing_strengths))
    denominator = sum(strength * func(x) for x, strength in zip(universe, firing_strengths))
    return numerator / denominator if denominator != 0 else 0

# Compute the output based on inputs RBS and RFS
def fuzzy_controller(RBS_value, RFS_value):
    # Define fuzzy sets for RBS, RFS, Speed, and Direction
    RBS_funcs = {"near": near, "med": med, "far": far}
    RFS_funcs = {"near": near, "med": med, "far": far}
    speed_funcs = {"low": low, "medium": medium, "high": high}
    direction_funcs = {"left": left, "straight": straight, "right": right}

    # Fuzzify inputs
    RBS_memberships = fuzzify(RBS_value, RBS_funcs)
    RFS_memberships = fuzzify(RFS_value, RFS_funcs)

    # Apply rules and calculate firing strengths
    speed_firing_strengths = []
    direction_firing_strengths = []
    speed_universe = np.linspace(10, 100, 100)  # Speed universe
    direction_universe = np.linspace(-30, 30, 100)  # Direction universe

    for rule in rule_base:
        RBS_label, RFS_label, speed_label, direction_label = rule
        firing_strength = min(RBS_memberships[RBS_label], RFS_memberships[RFS_label])
        speed_firing_strengths.append((firing_strength, speed_funcs[speed_label]))
        direction_firing_strengths.append((firing_strength, direction_funcs[direction_label]))

    # Defuzzify for speed
    speed_values = [centroid_defuzzification([fs for fs, _ in speed_firing_strengths], speed_universe, func)
                    for _, func in speed_firing_strengths]
    direction_values = [centroid_defuzzification([fs for fs, _ in direction_firing_strengths], direction_universe, func)
                        for _, func in direction_firing_strengths]

    # Ensure speed is within a reasonable range
    speed_output = np.clip(np.mean(speed_values), 0.0, 1.0)  # Clip speed to [0, 1]
    direction_output = np.clip(np.mean(direction_values), -1.0, 1.0)  # Clip direction to [-1, 1]


    # Print outputs for debugging
    print(f"Speed Output: {speed_output}, Direction Output: {direction_output}")

    return speed_output, direction_output

# ROS2 Node for handling robot movement
class FuzzyControlNode(Node):
    def __init__(self):
        super().__init__('fuzzy_control_node')
        
        # Create publisher for /cmd_vel
        self.publisher = self.create_publisher(Twist, '/cmd_vel', QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        
        # Create a subscription to the /scan topic
        self.create_subscription(LaserScan, '/scan', self.scan_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))

    def scan_callback(self, msg):
        # Process the LaserScan data to compute fuzzy control values
        valid_ranges = [r for r in msg.ranges if not np.isnan(r) and r != float('inf')]

        # Front distance: Use a wider slice for better coverage of the front (0° to 30° and 330° to 360°)
        front_distance = min(min(valid_ranges[0:30]), min(valid_ranges[-30:]))

        # Right distance: Use a wider slice for the right side (250° to 290°)
        right_distance = min(valid_ranges[250:290])
        
        # For simplicity, we use the front distance as the RBS value and right distance as the RFS value
        RBS_value = front_distance
        RFS_value = right_distance

        # self.get_logger().info(f"RBS: {front_distance}, RFS: {right_distance}")
        
        # Get speed and direction from the fuzzy controller
        speed, direction = fuzzy_controller(RBS_value, RFS_value)

        if speed < 0.1:
            direction = 0.0

        # Send the computed Twist message
        twist = Twist()
        twist.linear.x = speed
        twist.angular.z = direction

        self.get_logger().info(f"Speed: {speed}, Direction: {direction}")
        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)    
    controller = FuzzyControlNode()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.destroy_node()
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
