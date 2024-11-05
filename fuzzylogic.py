import numpy as np

"""
    Fuzzy logic to control the fan speed based on the room temperature and humidity

    INPUTS: 
        TEMPERATURE: ["LOW", "MEDIUM", "HIGH"]
        HUMIDITY: ["LOW", "MEDIUM", "HIGH"]
        
    OUTPUT:
        FAN SPEED(RPM): ["SLOW", "MODERATE", "FAST"]
"""

# Write a function that sets up your rule base.
"""
TEMPERATURE
    --LOW-- [0 - 25]
    --MEDIUM-- [20 - 30]
    --HIGH--  [25 - 30 - more]
    
HUMIDITY
    --LOW-- [0 - 60]
    --MEDIUM-- [40 - 80]
    --HIGH--  [60 - 80 - more]

"""
def temperature_low(x : int | float):
    if x <= 20:
        return 1
    elif 20 < x <= 25:
        return (25 - x) /5
    else:
        return 0
    
def temperature_mid(x: int | float):
    if 20 < x <= 25:
        return (x - 20)/5
    elif 25 < x <= 30:
        return (30 - x) /5
    else:
        return 0
    
def temperature_high(x : int | float):
    if x <= 25:
        return 0
    elif 25 < x <= 30:
        return (x - 25) /5
    elif x > 30:
        return 1
    
def humidity_low(x : int | float):
    if x <= 40:
        return 1
    elif 40 < x <= 60:
        return (60 - x) /20
    else:
        return 0
    
def humidity_mid(x : int | float):
    if 40 < x <= 60:
        return (x - 40)/20
    elif 60 < x <= 80:
        return (80 - x) /20
    else:
        return 0
    
def humidity_high(x : int | float):
    if x <= 60:
        return 0
    elif 60 < x <= 80:
        return (x - 60) /20
    elif x > 80:
        return 1


def rule_base():
    rule_base = [
        {
            'conditions': ('low', 'low'),
            'output': 'slow'
        },
        {
            'conditions': ('low', 'medium'),
            'output': 'moderate'
        },
        {
            'conditions': ('medium', 'low'),
            'output': 'moderate'
        },
        {
            'conditions': ('medium', 'medium'),
            'output': 'moderate'
        },
        {
            'conditions': ('medium', 'high'),
            'output': 'fast'
        },
        {
            'conditions': ('high', 'medium'),
            'output': 'fast'
        },
        {
            'conditions': ('high', 'high'),
            'output': 'fast'
        },
    ]
    
    return rule_base

# Write a function that returns the firing strength of a rule 
# What do you need to calculate the firing strength of a rule? To deter how strong each rule should be activate

# MEMBERSHIP FUNCTION DICTIONARY
membership_function = {
    'low': lambda x: temperature_low(x) if x < 60 else humidity_low(x), 
    'medium': lambda x: temperature_mid(x) if x < 60 else humidity_mid(x),
    'high': lambda x: temperature_high(x) if x < 60 else humidity_high(x),  
}

def firing_strength(rule: list[dict[str, any]], values: dict[str, float]):
    strengths = []
    for i, condition in enumerate(rule['conditions']):
        name = f'input{i+1}'
        membership_value = membership_function[condition](values[name])
        strengths.append(membership_value)
        
    return min(strengths)

# Write a function that calculates the centroid of any function.: to deter crisp output
def calculate_centroid(x, y):
    numerator = np.sum(x * y)
    denominator = np.sum(y)
    
    return numerator/denominator if denominator !=0 else 0

def output_slow(x):
    return np.maximum(0, 1 - x / 50)

def output_moderate(x):
    return np.where(
        x <= 50, x/50,
        np.where(x > 50, np.maximum(0, (100 - x) / 50), 0)
    )
    
def output_fast(x):
    return np.maximum(x/ 100, 1)

output_function = {
    'slow': output_slow,
    'moderate': output_moderate,
    'fast': output_fast
}

# Write a program that implements your FLC

def fuzzy_logic_controller():
    
    temperature = float(input("What is the current temperature: "))
    humidity = float(input("How humid is your room: "))
    
    if temperature and humidity: 
        values = {
            'input1': temperature,
            'input2': humidity
        }
        
        aggregated_output = []
        
        for rule in rule_base():
            strength = firing_strength(rule, values)
            x = np.linspace(0, 100,1000)
            y = np.minimum(strength, output_function[rule['output']](x))
            
            aggregated_output.append((x, y))
            
        combined_y = np.max([y for _, y in aggregated_output], axis=0)
        combined_x = aggregated_output[0][0]
        
        return round(calculate_centroid(combined_x, combined_y), 2)
    
def label_output(crisp_value):
    if crisp_value < 33:
        return "slow"
    elif 33 <= crisp_value < 66:
        return "moderate"
    else:
        return "fast"

if __name__ == '__main__':
    result = fuzzy_logic_controller()
    print("Crisp Output, Fan Speed (RPM):",result, "which is ",label_output(result))