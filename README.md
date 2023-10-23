# Machina Labs HW Challenge
---

### Overview

This repository contains the code and documentation for the Machina Labs HW Challenge (Robotics Software Engineer Role)

### Project Introduction

In this project, two packages are created and used to read data from a simulated 3DOF sensor and published onto multiple ros2 topics.

The first package ```robot_sensor``` contains 3 ROS2 nodes.

* The first ```custom_service_sensor``` allows us to request simulated data from a 3DOF sensor.
* The second ```sensor_server``` continuously reads data from a 3DOF sensor and allows us to request the latest data.
* The third ```sensor_client``` continuously makes requests to both services and publishes the sensor data onto two ROS2 topics.

The second package ```robot_sensor_interfaces``` defines the interface type ```SensorData``` for the above services.

### Strategy/Approach

The number of samples you need to obtain to represent a signal for feedback control depends on various factors, including the system dynamics, control loop requirements, and the nature of the signal itself.
It also depends on the length of the interval over which the samples are collected.
Here are some considerations to help determine a suitable number of samples:

* Time Delay: The delay introduced by the sensor should be prioritized.
  To account for this delay and provide real-time control, we should sample at a rate that ensures we receive new data before the control algorithm operates.
If we want our control loop to operate at 500Hz, this only allows a delay of 1 ms for each sensor request.
If we want 50 samples per loop, the time delay is 25 ms (to sample) + 1-3 ms, if this is an important factor we can further reduce the number of samples.

So for a delay of 2 ms in total, we can only afford 1-2 samples for the custom service call.
For the sensor server service call, there should be no delay as it is asynchronously reading the data. So we should be able
to increase the number of samples as needed for that call. I selected a number of samples of 100.

* System Dynamics: The number of samples depends on how fast the state of the system changes. For the application of robotic arms this depends on joint acceleration limits. If the arm joints change at around 20Hz then we need atleast 40 samples in each iteration of the control loop to capture these dynamics.

* Control Loop Frequency: The sampling rate should be aligned with the control loop frequency. If your control algorithm operates at a specific frequency, it's advisable to sample at a rate that's an integer multiple of that control loop frequency. This ensures that the control actions are applied at regular intervals and leads to more predictable and stable control. 

### Note on the Sensor Client Implementation

When implementing the sensor client node I weighted using synchronous versus asynchronous clients.
The upside of asynchronous clients is it is guaranteed to avoid deadlock, but because we need to use the acquired sensor data immediately 
in the timer callback (publishing it) this use case requires a synchronous client even though this is not the best practice.
I decided to proceed with this using different callback groups and multi threaded executor to ensure no deadlocks between the timer callback
and synchronous client calls.

### Running the Code

Clone the repository and navigate to the root of the ```Robotic_solution``` directory

Run ```colcon build``` to build the two packages.

In the first terminal run ```python3 robot_sensor/robot_sensor/sensor.py``` this will startup the two sensors and the sensor server node.

In the second terminal run ```ros2 run robot_sensor custom_service_sensor``` this will startup the custom service node.

Finally startup the client to start sending requests by running ```ros2 run robot_sensor sensor_client```

To output the data on the two rostopics run

```ros2 topic echo /sensor_data_discrete```

```ros2 topic echo /sensor_data_continuous```
