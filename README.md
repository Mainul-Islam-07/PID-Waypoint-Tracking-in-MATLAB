# PID-Waypoint-Tracking-in-MATLAB
A Matlab Code which shows a object (eg. car) following a waypoint having constraint steering angle, using PID.

This MATLAB code implements a simple PID controller for waypoint tracking using a kinematic bicycle model. Let's break down the code step by step:

![image](https://github.com/Mainul-Islam-07/PID-Waypoint-Tracking-in-MATLAB/assets/78782260/d4a46665-24f5-40e4-95e7-9ad318e896eb)


1. **Simulation Parameters and Map Definition**:
   - `sim.simLength`: Simulation time in seconds.
   - `sim.simTs`: Timestep for the simulation.
   - `sim.target_velocity`: Target velocity for the vehicle.
   - `Map.points`: Defines a set of waypoints that the vehicle will navigate through.
   - `Map.acc_bd`, `Map.WPT_Now`, `Map.WPT_max`: Parameters related to the map and waypoint navigation.

2. **Initial Parameters**:
   - `sim.state`: Initial state of the vehicle (position and orientation).
   - `Kp`, `Ki`, `Kd`: Proportional, integral, and derivative gains for the PID controller.
   - `error_integral`, `prev_error`: Variables to track the integral of the error and the previous error for the PID controller.

3. **PID Controller Loop**:
   - The code enters a loop to simulate the vehicle's movement and control using a PID controller until the simulation time exceeds `sim.simLength`.
   - Inside the loop, it calculates the error between the current position of the vehicle and the target waypoint position.
   - The error is then used to update the PID controller and calculate the steering angle using the PID gains.
   - The steering angle is constrained within a certain range (`-pi/6` to `pi/6`).
   - The throttle is set to maintain a constant velocity (`sim.target_velocity`).
   - The vehicle's state is updated using a kinematic bicycle model (`integrate_car_pid` function).
   - If the vehicle is close to the current waypoint, the next waypoint is selected.

4. **Visualization**:
   - The code includes visualization commands to plot the map, waypoints, vehicle trajectory, and current vehicle state.
   - It uses `plot` and `quiver` functions for visualization.
   - The visualization can be controlled by setting `plot_now` to `1`.

5. **Helper Function**:
   - `integrate_car_pid`: Implements the kinematic bicycle model to update the vehicle's state based on the current state, steering angle, throttle, and timestep.

6. **Execution**:
   - The code executes the simulation loop and updates the vehicle's state until the simulation time exceeds `sim.simLength`.

7. **Output**:
   - During the simulation loop, it prints the steering angle at each iteration.

8. **Completion**:
   - After the simulation loop is complete, it prints "PID Waypoint Tracking complete".

This code simulates a vehicle navigating through a set of waypoints using a PID controller for steering control based on a kinematic bicycle model.
