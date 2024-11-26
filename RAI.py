class FuzzyImplementation:
    def __init__(self) -> None:
        self.rule_base = [
        ("Near", "Near", "Slow", "Medium"),
        ("Near", "Medium", "Slow", "Medium"),
        ("Near", "Far", "Slow", "Fast"),
        ("Medium", "Near", "Medium", "Slow"),
        ("Medium", "Medium", "Medium", "Medium"),
        ("Medium", "Far", "Slow", "Medium"),
        ("Far", "Near", "Medium", "Slow"),
        ("Far", "Medium", "Medium", "Slow"),
        ("Far", "Far", "Fast", "Slow"),
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
        if a <= x <= b:
            return (x - a) / (b - a)
        return 0.0

    def falling_edge(self, x, b, C):
        """Calculate membership using a falling edge formula."""
        if b <= x <= C:
            return (C - x) / (C - b)
        return 0.0

    def right_forward_sensor_membership(self, distance):
        """
        Compute the membership values for 'Near', 'Medium', and 'Far' fuzzy sets
        for a right forward sensor using rising and falling edges.
        
        Parameters:
            distance (float): The input distance measured by the sensor.
            
        Returns:
            dict: A dictionary with the membership values for 'Near', 'Medium', and 'Far'.
        """
        membership = {"Near": 0.0, "Medium": 0.0, "Far": 0.0}
        
        if 0 <= distance <= 9:
            membership["Near"] = 1.0
        
        if 10 <= distance <= 20:
            membership["Near"] = self.falling_edge(distance, 10, 20)
            membership["Medium"] = self.rising_edge(distance, 10, 20)

        if 21 <= distance <= 31:
            membership["Medium"] = self.falling_edge(distance, 21, 31)
            membership["Far"] = self.rising_edge(distance, 21, 31)
            
        if distance >= 32:
            membership["Far"] = 1.0
        
        return self.remove_zero_memberships(membership)

    def right_backward_sensor_membership(self, distance):
        """
        Compute the membership values for 'Near', 'Medium', and 'Far' fuzzy sets
        for a right forward sensor using rising and falling edges.
        
        Parameters:
            distance (float): The input distance measured by the sensor.
            
        Returns:
            dict: A dictionary with the membership values for 'Near', 'Medium', and 'Far'.
        """
        membership = {"Near": 0.0, "Medium": 0.0, "Far": 0.0}
        
        if 0 <= distance <= 38.9:
            membership["Near"] = 1.0

        if 39.9 <= distance <= 49.9:
            membership["Near"] = self.falling_edge(distance, 39.9, 49.9)
            membership["Medium"] = self.rising_edge(distance, 39.9, 49.9)

        if 50 <= distance <= 60:
            membership["Medium"] = self.falling_edge(distance, 50, 60)
            membership["Far"] = self.rising_edge(distance, 50, 60)
            
        if distance >= 60.1:
            membership["Far"] = 1.0
        
        return self.remove_zero_memberships(membership)

    def make_inference(self, forward_membership, backward_membership):
        """
        Perform fuzzy inference using a rule base and sensor memberships.
        
        Parameters:
            rule_base (list of tuples): A list of rules in the format
                                        (Forward Condition, Backward Condition, Speed, Direction).
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
            "Slow": (10, 30),
            "Medium": (30, 50),
            "Fast": (50, 70),
        }
        
        if key in ranges:
            start, end = ranges[key]
            return (start + end) / 2  # Calculate the midpoint
        else:
            print(key)
            raise ValueError("Invalid key. Choose from 'Slow', 'Medium', 'Fast'.")

    def defuzzify(self, memberships, firing_strength_sum):
            """Compute the weighted sum for a set of fuzzy memberships."""
            weighted_sum = 0
            
            for key, value in memberships.items():
                middle = self.get_middle_of_range(key)  # Get the middle value of the range
                for item in value:
                    weighted_sum += middle * item

            return weighted_sum / firing_strength_sum
        
def main():
    fuzzyImplementation = FuzzyImplementation()
    
    # Example usage
    right = 15
    back = 52
    right_membership_values = fuzzyImplementation.right_forward_sensor_membership(right)
    back_membership_values = fuzzyImplementation.right_backward_sensor_membership(back)
    # print(f"Membership values at right {right}: {right_membership_values}")
    # print(f"Membership values at back {back}: {back_membership_values}")

    output, firing_strength_sum = fuzzyImplementation.make_inference(right_membership_values, back_membership_values)
    print(output)

    # Compute for Speed and Direction
    weighted_speed = fuzzyImplementation.defuzzify(output.get("Speed", {}), firing_strength_sum)
    weighted_direction = fuzzyImplementation.defuzzify(output.get("Direction", {}), firing_strength_sum)

    print({"Speed": weighted_speed, "Direction": weighted_direction})
    
if __name__ == "__main__":
    main()
