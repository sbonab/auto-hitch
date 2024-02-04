#!/usr/bin/env python3

import threading
import time
import math
import matplotlib.pyplot as plt
import random
from dataclasses import dataclass
import os
import time

@dataclass
class Vehicle:
    wb: float = 2.0
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0
    alpha: float = 0.0
    vel: float = 0.0
    s: float = 0.0
    tire_l: float = wb/5
    tire_w: float = wb/15

    def move(self, dt: float):
        self.x += self.vel * math.cos(self.theta) * dt
        self.y += self.vel * math.sin(self.theta) * dt
        self.theta += self.vel / self.wb * math.tan(self.alpha) * dt
        self.s += abs(self.vel) * dt

# Define the named pipe (FIFO) for communication
INPUT_PIPE_PATH = "/tmp/vehicle_input.pipe"
OUTPUT_PIPE_PATH = "/tmp/vehicle_output.pipe"

try:
    os.mkfifo(INPUT_PIPE_PATH)
except FileExistsError:
    print(f"Input pipe already exists")

try:
    os.mkfifo(OUTPUT_PIPE_PATH)
except FileExistsError:
    print(f"Output pipe already exists")

class Simulator:
    def __init__(self, vehicle: Vehicle):
        self.vehicle = vehicle
        self.lock = threading.Lock()
        self.running = False
        
    def start_sim(self, freq=100):
        dt = 1/freq
        while self.running:
            with self.lock:
                self.vehicle.move(dt)
            time.sleep(dt)

    def start_plot(self, freq=10):
        plt.ion()  # Enable interactive mode
        fig, ax = plt.subplots()
        tire_r, = ax.plot([], [], 'k-', linewidth=3)
        tire_f,  = ax.plot([], [], 'k-', linewidth=3)
        # draw the line between p_r and p_f
        body, = ax.plot([], [], 'k-')
        ax.set_xlim(-1, 5)
        ax.set_ylim(-1, 1)
        ax.set_aspect('equal')
        # Turn the grid on
        ax.grid(True)

        while self.running:
            with self.lock:
                x_r = self.vehicle.x
                y_r = self.vehicle.y
                x_f = self.vehicle.x + self.vehicle.wb * math.cos(self.vehicle.theta)
                y_f = self.vehicle.y + self.vehicle.wb * math.sin(self.vehicle.theta)
                
            body.set_data([x_r, x_f], [y_r, y_f])
            
            dx_tire_r = self.vehicle.tire_l * math.cos(self.vehicle.theta)
            dy_tire_r = self.vehicle.tire_l * math.sin(self.vehicle.theta)
            # Why the following line fails?
            tire_r.set_data([x_r - dx_tire_r/2, x_r + dx_tire_r/2], [y_r - dy_tire_r/2, y_r + dy_tire_r/2])

            dx_tire_f = self.vehicle.tire_l * math.cos(self.vehicle.theta + self.vehicle.alpha)
            dy_tire_f = self.vehicle.tire_l * math.sin(self.vehicle.theta + self.vehicle.alpha)
            tire_f.set_data([x_f - dx_tire_f/2, x_f + dx_tire_f/2], [y_f - dy_tire_f/2, y_f + dy_tire_f/2])

            fig.canvas.draw()
            fig.canvas.flush_events()

            dt = 1/freq
            time.sleep(dt)
        plt.close(fig)

    def publish_output(self, freq=50):
        print(f"Publishing output on {OUTPUT_PIPE_PATH}")
        while self.running:
            with self.lock:
                x = self.vehicle.x
                y = self.vehicle.y
                theta = self.vehicle.theta
                s = self.vehicle.s
            with open(OUTPUT_PIPE_PATH, 'w') as pipe:
                pipe.write(f"{x} {y} {theta} {s}\n")
                pipe.flush()
                print(f"{time.time()} Updated data | x: {x}, y: {y}, theta: {theta}, s: {s}")
            dt = 1/freq
            time.sleep(dt)

    def listen_for_input(self):
        print(f"Listening for input on {INPUT_PIPE_PATH}")
        while self.running:
            with open(INPUT_PIPE_PATH, 'r') as pipe:
                data = pipe.readline().strip()
                if data:
                    try:
                        vel_str, alpha_str = data.split(' ')
                        with self.lock:
                            print(f"{time.time()} Received data | vel: {vel_str}, alpha: {alpha_str}")
                            self.vehicle.vel = float(vel_str)
                            self.vehicle.alpha = float(alpha_str)
                    except ValueError as e:
                        print(f"ValueError: {e}")
                    except Exception as e:
                        print(f"An unexpected error occurred: {e}")

    def start(self):
        self.running = True
        # Start the simulation in a new thread
        sim_thread = threading.Thread(target=self.start_sim)
        sim_thread.start()
        # Start listening for input in a new thread
        listen_thread = threading.Thread(target=self.listen_for_input)
        listen_thread.start()
        # Start publishing output in a new thread
        publish_thread = threading.Thread(target=self.publish_output)
        publish_thread.start()
        # Start plotting for data in the main thread
        self.start_plot()
        # Once listen_for_input is done, stop the threads
        self.running = False
        sim_thread.join()
        listen_thread.join()
        publish_thread.join()  

vehicle = Vehicle(wb=2.0, x=2.0)
#input_controller = InputController()
# Usage
simulator = Simulator(vehicle)
simulator.start()

os.remove(INPUT_PIPE_PATH)  # Remove the pipe once we're done
os.remove(OUTPUT_PIPE_PATH)  # Remove the pipe once we're done
