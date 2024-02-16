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
Controller is implemented in C++. For path planning, it has been assumed that the vehicle tries to follow a reference path that is conssited of an arc of circle with given radius (turning radius) at vehicle's current position, an arc of a circule at vehicle's target position, and the co-tangent line between these two circles (videos below help demonstrate this visually).
Trajectory of the vehicle is planned in a way for the vehicle to ramp up to a target speed, continue with the target speed for some stretch of the planned path, and finally to ramp down to zero while approaching the target point.
Controller has read/write threads to communicate with simulator. It listens to the vehicle states published by the simulator and in turn, published controller inputs, vehicle speed and steering angle, for the simulator to use.

### Path Tracking
Below, video outputs of the controller on the vehicle with starting point `x = 6m`, `y = 1.3m`, starting orientation `theta = 0`, turning radius `radius = 6 m`, and wheelbase `wb = 3.6 m` are shown below.
For trajectory controller, there are two controllers designed and implemented for the vehicle to be able to track the reference path. First controller is an open-loop controller. It simply calculates a schedule of inputs based on how long vehicle has moved along the path (path length) and later applies the inputs according to this input. The video below shows the performance of this controller in the ideal world.

It is all well for the open-loop controller without any uncertainties on the vehicle states, controller inputs, and assuming actual vehicle behaviour exactly matches the bicycle model used in deriving open-loop controller inputs. However, this will be most likely not the case. The video below shows how the performance of the open-loop controller deteriorates in presense of uncertainties.


To develop a more robust controller, a second controller has been designed and implemented which is a sliding mode controller. It defines a sliding surface based on vehicle states where the dynamics of the vehicle path tracking error is asymptotically stable on the this surface. Outside the surface, the controller calculates the controller inputs in a way that the system converges to the sliding surface in a finite time. The video below shows the performance of this controller **in presense of the uncertainties**.

![auto-hitch](https://github.com/sbonab/auto-hitch/assets/63617509/2c9e8df3-0c81-4eba-b4c6-c5b37d4f2b22)
