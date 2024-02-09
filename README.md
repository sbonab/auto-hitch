# Auto-Hitch
This project includes a vehicle simulator and controller for a scenario where the controller tries to rear the vehicle, from a starting position and orientation, back to the origin. This might be similar to a truck backing to attach a trailer hitch autonomously. 

## Simulator
Simulator is implemented in python and the graphics are displayed using matplotlib. For vehicle dynamcis, bicycle model have been used. This is a fair assumption as the vehicle operates at low speed where tire slip is negligble.
Simulator process has publisher thread, that publishes vehicle states to a pipe for the controller to read, listener thread to listen to controller inputs from another pipe, and simulation thread.
To run the simulator, simply execute the following code from the root of the project
```
./simulator/simulator.py
```

## Controller
Controller is implemented using C++. For path planning, it has been assumed that starts on an arc of circle with given radius (turning radius), then follows the straight co-tangent line, and finally approaches the origin by following another circle arc (see the video below).
Trajectory of the vehicle is planned in a way for the vehicle to ramp up to a target speed, continue with the target speed for some stretch of the planned path, and finally to ramp down to zero while approaching the target point.
Controller has read/write thread to communicate with simulator. 

## Output
Output of the controller on the vehicle with starting point `x = 6m`, `y = -1.3m`, `theta = 0`, `radius = 6 m`, and `wheelbase = 3.6 m`.
![auto-hitch](https://github.com/sbonab/auto-hitch/assets/63617509/2c9e8df3-0c81-4eba-b4c6-c5b37d4f2b22)
