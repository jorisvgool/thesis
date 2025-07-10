#!/usr/bin/env python3

import numpy as np
import os
import re

def log_path(log_dir_name="logs", log_filename_prefix="drone_log"):
    script_dir = os.path.dirname(os.path.abspath(__file__))
    log_dir = os.path.join(script_dir, log_dir_name)
    os.makedirs(log_dir, exist_ok=True)

    existing_logs = [f for f in os.listdir(log_dir) if re.match(rf"{log_filename_prefix}\d+\.csv", f)]
    existing_numbers = [int(re.search(r'\d+', f).group()) for f in existing_logs]
    next_number = max(existing_numbers) + 1 if existing_numbers else 1

    log_path = os.path.join(log_dir, f"{log_filename_prefix}{next_number}.csv")
    return log_path

class PID:
    def __init__(self, Kp, Ki, Kd, dt, output_limits=(None, None)):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.dt = dt
        self.integral = 0.0
        self.prev_error = 0.0
        self.output_limits = output_limits

    def compute(self, error):
        self.integral += error * self.dt
        derivative = (error - self.prev_error) / self.dt
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error

        lower, upper = self.output_limits
        if lower is not None:
            output = max(lower, output)
        if upper is not None:
            output = min(upper, output)

        return output

def save_log(log_data, path):
    import csv
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["time", "e1", "e2", "e3", "z1", "z2", "z3", "x", "y", "h"])
        writer.writerows(log_data)

def block_diag(z):
    Dz = np.zeros((6, 3))
    for i in range(3):
        Dz[2*i : 2*i+2, i] = z[2*i : 2*i+2]
    return Dz
