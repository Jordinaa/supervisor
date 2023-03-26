# eulers class
import numpy as np 

class EulersMethod:
    def __init__(self, derivative_func):
        self.derivative_func = derivative_func

    def predict(self, initial_value, time_step, num_steps, *args):
        values = [initial_value]

        for step in range(num_steps):
            current_value = values[-1]

            # Calculate the derivative
            derivative = self.derivative_func(current_value, *args)

            # Update the value using Euler's method
            new_value = current_value + time_step * derivative
            values.append(new_value)

        return values
