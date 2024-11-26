# Ziegler-Nichols

The Ziegler-Nichols method is a popular technique for tuning the proportional-integral-derivative (PID) controller. It provides a systematic way to set the PID parameters (Kp, Ki, and Kd) based on experimental data, aiming to achieve a balance between fast response, minimal overshoot, and good steady-state performance.

## Ziegler-Nichols Tuning Method

Ziegler and Nichols developed two primary methods for PID tuning: the **Continuous Cycling Method** and the **Ultimate Gain Method**.

Here’s a breakdown of how the **Ziegler-Nichols Tuning Method** works:

### **Continuous Cycling Method**

This method is used for processes where the output can be controlled and oscillated (e.g., heating, temperature control systems, or process control systems).

- **Steps**:
    1. **Set Ki = 0 and Kd = 0**: Initially, start with only a proportional controller.
    2. **Increase Kp**: Increase the proportional gain (Kp) until the system starts to oscillate continuously, i.e., when the system reaches a stable oscillation, where the output does not grow unbounded but instead oscillates with a constant amplitude.
    3. **Note the Critical Gain (Ku) and Period (Pu)**:
        - `Ku` = the value of the proportional gain (Kp) at which the system begins to oscillate continuously.
        - `Pu` = the period of the oscillation (how long it takes to complete one full cycle).
    4. **Set the PID values**:
        - Use Ziegler-Nichols formulas to compute the PID parameters:
            - **Proportional gain (Kp)** = 0.6 * Ku
            - **Integral time (Ti)** = Pu / 2
            - **Derivative time (Td)** = Pu / 8

    These values should give you a good starting point. However, some fine-tuning might still be needed based on the specific behavior of the system.

### **Ultimate Gain Method** (Another approach in Ziegler-Nichols)

This method is more suitable for processes where oscillation is not ideal or applicable.

- **Steps**:
    1. **Increase Kp while keeping Ki = 0 and Kd = 0**.
    2. **Look for the "ultimate gain"** where the system begins to oscillate with a consistent amplitude (as mentioned before, this is `Ku`).
    3. **Measure the period of oscillation** (`Pu`).
    4. **Apply the formulas for Kp, Ki, and Kd**:
        - `Kp = 0.45 * Ku`
        - `Ki = 1.2 * Kp / Pu`
        - `Kd = 0.075 * Kp * Pu`

### Why Use Ziegler-Nichols Tuning?

- **Simple to Use**: It does not require a deep understanding of the underlying mathematical models of the system, which makes it relatively straightforward to implement.
- **Experimental**: It’s based on real-world experimentation and observation of the system’s response.
- **Effective Starting Point**: It provides reasonable values for the PID parameters, which can then be fine-tuned further to improve performance.

### Example of PID Tuning with Ziegler-Nichols (Based on Your Code):

In the code you provided, you are using pre-tuned PID values based on the Ziegler-Nichols method:

```python
# PID tuned with Ziegler-Nichols
Kp = 0.4
Kd = 0.08
Ki = 0.02
```

In your case:

- **Kp (Proportional Gain)**: Controls how aggressively the robot tries to correct the error (the difference between the current distance and the desired distance).
- **Ki (Integral Gain)**: Handles accumulated error over time (i.e., if the robot has been too far from the wall for too long).
- **Kd (Derivative Gain)**: Dampen the rate of error change, preventing overshooting or oscillation in the robot's behavior.

These values are derived through trial and error, applying the Ziegler-Nichols method to get the **ultimate gain (Ku)** and **period (Pu)**, and then using the formulae to calculate the PID parameters.

### Benefits of Ziegler-Nichols Tuning

- **Quick Tuning**: By performing experimental measurements, you can quickly arrive at a good set of PID values.
- **Robustness**: The method tends to provide a well-rounded and stable response for most systems, especially when fine-tuned further.
- **Real-World Application**: It’s widely used in industries for process control, robotics, and automation where real-time tuning is often necessary.

### Limitations

- **Not Optimal for All Systems**: The method may not always provide the best performance in highly dynamic systems or those with noise.
- **Oscillation**: The system has to oscillate to determine `Ku` and `Pu`, so it might not be ideal for sensitive or safety-critical applications.
  
### Conclusion

To summarize, the Ziegler-Nichols method is a good starting point for tuning PID controllers. It helps find the right balance between aggressive correction (Kp), eliminating steady-state errors (Ki), and reducing oscillation (Kd). After applying Ziegler-Nichols, you might need to fine-tune the parameters based on the robot’s performance, especially when dealing with real-world factors like noise, irregular walls, or varying environments.