import numpy as np

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

# Defuzzification using the centroid method
def centroid_defuzzification(firing_strengths, universe, func):
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
    speed_universe = np.linspace(0, 100, 100)  # Speed universe
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

    speed_output = np.mean(speed_values)
    direction_output = np.mean(direction_values)

    return speed_output, direction_output

# Example usage
RBS_value = 25  # Example RBS sensor input
RFS_value = 40  # Example RFS sensor input

speed, direction = fuzzy_controller(RBS_value, RFS_value)
print(f"Computed Speed: {speed}")
print(f"Computed Direction: {direction}")
