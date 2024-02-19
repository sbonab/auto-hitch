# Auto-Hitch
This project includes a vehicle simulator and controller for a scenario where the controller tries to rear the vehicle, from a starting position and orientation, back to the origin where the vehicle is aligned with the x axis. This might be similar to a truck backing to attach a trailer hitch autonomously. 

## Simulator
Simulator is implemented in python and the graphics are displayed using matplotlib. For vehicle dynamcis, the bicycle model has been used. This is a fair assumption as the vehicle operates at low speed where the tire slip is negligible.
Simulator process has publisher thread, that publishes vehicle states to a pipe for the controller to read, a listener thread to listen to the controller inputs from another pipe, and a simulation thread.
To run the simulator, simply execute the following code from the root of the project
```
./simulator/simulator.py
```

## Controller
Controller is implemented in C++. For path planning, it has been assumed that the vehicle tries to follow a reference path that is consisted of an arc of a circle with the given radius (turning radius) at vehicle's current position, an arc of a circule at vehicle's target position, and the co-tangent line between these two circles (videos below help demonstrate this visually).
Trajectory of the vehicle is planned in a way for the vehicle to ramp up to a target speed, continue with the target speed for some stretch of the planned path, and finally to ramp down to zero while approaching the target point.
Controller has read/write threads to communicate with the simulator. It listens to the vehicle states published by the simulator and in turn, publishes controller inputs, vehicle speed and steering angle, for the simulator to use.
To run the controller, please build the project first and then execute the `controller` executable.

### Path Tracking
Below, video outputs of the controller on the vehicle with starting point `x = 6m`, `y = 1.3m`, starting orientation `theta = 0`, turning radius `radius = 6 m`, and wheelbase `wb = 3.6 m` are shown.
For trajectory controller, there are two controllers designed and implemented for the vehicle to be able to track the reference path. First controller is an open-loop controller. It simply calculates a schedule of inputs based on how long vehicle has moved along the path (path length) and later applies the inputs according to this schedule. The video below shows the performance of this controller in the ideal world with no uncertainties (click on the image to get redirected to the video).

[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/cOCaXRPYjiI/0.jpg)](https://youtu.be/cOCaXRPYjiI)

It is all well for the open-loop controller without any uncertainties on the vehicle states, controller inputs, and assuming that the actual vehicle dynamics exactly matches the bicycle model used in deriving open-loop controller inputs. However, this will most likely not be the case. The video below shows how the performance of the open-loop controller deteriorates in the presense of uncertainties (click on the image to get redirected to the video).

[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/qAqqdhyA1rw/0.jpg)](https://youtu.be/qAqqdhyA1rw)

To develop a more robust controller, a second controller has been designed and implemented which is a sliding mode controller. It defines a sliding surface based on vehicle states where the dynamics of the vehicle path tracking error is asymptotically stable on the this surface. Outside the surface, the controller calculates the controller inputs in a way that the system converges to the sliding surface in a finite time. The video below shows the performance of this controller **in the presense of the uncertainties** (click on the image to get redirected to the video).

[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/kyHB2-2MIo8/0.jpg)](https://youtu.be/kyHB2-2MIo8)

References
Zhang, Jiaxu, et al. "Trajectory planning and tracking control for autonomous parallel parking of a non-holonomic vehicle." Measurement and Control 53.9-10 (2020): 1800-1816.
