import threading
import time
import math
import matplotlib.pyplot as plt
import random
from dataclasses import dataclass
import os

@dataclass
class Vehicle:
    wb: float = 2.0
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0
    alpha: float = 0.0
    vel: float = 0.0
    tire_l: float = wb/7
    tire_w: float = wb/18

    def move(self, dt: float):
        self.x += self.vel * math.cos(self.theta) * dt
        self.y += self.vel * math.sin(self.theta) * dt
        self.theta += self.vel / self.wb * math.tan(self.alpha) * dt

# Define the named pipe (FIFO) for communication
PIPE_PATH = "/tmp/vehicle_pipe"

try:
    os.mkfifo(PIPE_PATH)
except FileExistsError:
    pass  # If the pipe already exists, there's no need to create it

class InputController:
    def __init__(self, pipe_path: str = PIPE_PATH):
        self.pipe_path = pipe_path
        self.running = False

    def start(self, freq=10):
        self.running = True
        while self.running:
            with open(self.pipe_path, 'w') as pipe:
                # Write some example values to the pipe
                # change vel to some random value between 0 and 5
                vel = random.uniform(0, 5)
                alpha =random.uniform(0, 0.1)
                pipe.write(f"{vel},{alpha}\n")
                pipe.flush()
            dt = 1/freq
            time.sleep(dt)  # Write new values every 0.1 seconds

    def stop(self):
        self.running = False

    def get_pipe_path(self):
        return self.pipe_path


class Simulator:
    def __init__(self, vehicle: Vehicle, input_controller: InputController):
        self.vehicle = vehicle
        self.input_controller = input_controller
        self.lock = threading.Lock()
        self.running = False
        
    def start_update(self, freq=50):
        dt = 1/freq
        while self.running:
            with self.lock:
                self.vehicle.move(dt)
            time.sleep(dt)

    def start_plot(self, freq=10):
        plt.ion()  # Enable interactive mode
        fig, ax = plt.subplots()
        tire_r, = ax.plot([], [], 'k-')
        tire_f,  = ax.plot([], [], 'k-')
        # draw the line between p_r and p_f
        body, = ax.plot([], [], 'k-')
        ax.set_xlim(-10, 10)
        ax.set_ylim(-10, 10)
        ax.set_aspect('equal')
        # Turn the grid on
        ax.grid(True)

        while self.running:
            with self.lock:
                x_r = self.vehicle.x
                y_r = self.vehicle.y
                x_f = self.vehicle.x + self.vehicle.wb * math.cos(self.vehicle.theta)
                y_f = self.vehicle.y + self.vehicle.wb * math.sin(self.vehicle.theta)
                # increase vehicle.vel by a random value between -1 and 1
                self.vehicle.vel += random.uniform(-.1, .1)
                # cap the vehicle.vel between 0 and 5
                self.vehicle.vel = max(min(self.vehicle.vel, 5), 0)
                # increase vehicle.alpha by a random value between -0.1 and 0.1
                self.vehicle.alpha += random.uniform(-0.1, 0.1)
                # cap the vehicle.alpha between -0.4 and 0.4
                self.vehicle.alpha = max(min(self.vehicle.alpha, 0.4), -0.4)
                
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

    def listen_for_data(self):
        while self.running:
            pipe_path = self.input_controller.get_pipe_path()   
            with open(pipe_path, 'r') as pipe:
                data = pipe.readline().strip()
                if data:
                    try:
                        vel_str, alpha_str = data.split(',')
                        with self.lock:
                            print(f"Received data: {vel_str}, {alpha_str}")
                            self.vehicle.vel = float(vel_str)
                            self.vehicle.alpha = float(alpha_str)
                    except ValueError as e:
                        print(f"ValueError: {e}")
                    except Exception as e:
                        print(f"An unexpected error occurred: {e}")

    def start(self):
        self.running = True
        # Start the input controller in a new thread
        input_thread = threading.Thread(target=self.input_controller.start)
        input_thread.start()
        # Start the update method in a new thread
        update_thread = threading.Thread(target=self.start_update)
        update_thread.start()
        # Start the plot method in a new thread
        plot_thread = threading.Thread(target=self.start_plot)
        plot_thread.start()
        # Start listening for data in the main thread
        self.listen_for_data()
        # Once listen_for_data is done, stop the threads
        self.running = False
        input_thread.join()
        update_thread.join()
        plot_thread.join()

vehicle = Vehicle(wb=2.0)
input_controller = InputController()
# Usage
simulator = Simulator(vehicle, input_controller)
simulator.start()

os.remove(PIPE_PATH)  # Remove the pipe once we're done
