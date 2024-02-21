clc
close all
clear all
warning('off')

sim.simLength = 210;  % Simulation time
sim.simTs = 0.4;     % Timestep
plot_now = 1;
sim.target_velocity = 0.5;  % Target velocity (m/s)

%% MAP
Map.points = [0 5 10 10 20 25 30 35 35 30 30 20 10  0  -5   0 ;...
              0 0 0  10 10 15 15 10 5  0  -5 -5 -5 -5  -2.5 0];

Map.acc_bd = 0.7;
Map.WPT_Now = 1;
Map.WPT_max = size(Map.points, 2);

%% INITIAL PARAMETERS SIMULATION
sim.state = [0, 0, 0];
sim.iter = 0;
sim.time = 0;

%% PID Controller Parameters
Kp = 0.8;  % Proportional gain
Ki = 0.00;  % Integral gain
Kd = 0.00; % Derivative gain

error_integral = 0;
prev_error = 0;

%% SIMULATION LOOP
disp('PID Waypoint Tracking start');
while sim.time(end) < sim.simLength
    % Calculate error
    current_pos = sim.state(1:2)'
    target_pos = Map.points(:, Map.WPT_Now)
    steering_angle = target_pos - current_pos;
    dir = [cos(sim.state(end, 3)) sin(sim.state(end, 3))];
    error = -atan2(steering_angle(1) * dir(2) - steering_angle(2) * dir(1), dir(1) * dir(2) + steering_angle(1) * steering_angle(2));

    % Update PID controller
    error_integral = error_integral + error * sim.simTs;
    error_derivative = (error - prev_error) / sim.simTs;

    
    % Calculate control inputs using PID
    steering_angle = Kp * error + Ki * error_integral + Kd * error_derivative
    if steering_angle < -pi/6
        steering_angle = -pi/6;
    elseif steering_angle > pi/6
        steering_angle = pi/6;
    end
    throttle = sim.target_velocity;
    
    % Update vehicle state using kinematic bicycle model
    sim.state = integrate_car_pid(sim.state, steering_angle, throttle, sim.simTs);
    
    % Update iteration and time
    sim.iter = sim.iter + 1;
    sim.time = [sim.time; sim.iter * sim.simTs];
    
    % Update waypoint if the vehicle is close to the current waypoint
    if norm(current_pos - target_pos) < 1.0
        Map.WPT_Now = min(Map.WPT_Now + 1, Map.WPT_max);
    end
    
    % Visualize
    if plot_now
        figure(1)
        hold on
        grid on
        axis equal
        xlim([-5 40])
        ylim([-15 25])

        % Plot map
        plot(Map.points(1,:), Map.points(2,:), 'b','LineWidth',1)
        plot(target_pos(1), target_pos(2), 'ro','LineWidth',1)

        % Plot trajectory 
        plot(sim.state(1), sim.state(2), 'k.','MarkerSize',10)
        plot(sim.state(end,1), sim.state(end,2), 'bo','LineWidth',3)
        quiver(sim.state(end,1), sim.state(end,2), 2*cos(sim.state(end,3)), 2*sin(sim.state(end,3)), 'LineWidth',2)
        
        
        disp(steering_angle);

        drawnow
        
        %exportgraphics(gcf,'pidtest.gif','Append',true);
        pause(0.1); % Optional pause to slow down the visualization
    end
    prev_error = error; % Update previous error for next iteration
end

disp('PID Waypoint Tracking complete');

%% Helper function for PID Controller
function new_state = integrate_car_pid(current_state, steering_angle, throttle, dt)
    % Kinematic bicycle model
    L = 0.5;  % Wheelbase
    beta = atan(0.5 * tan(steering_angle));
    x_dot = throttle * cos(current_state(3) + beta);
    y_dot = throttle * sin(current_state(3) + beta);
    theta_dot = (throttle / L) * tan(steering_angle);

    % Update state
    new_state = current_state + [x_dot, y_dot, theta_dot] * dt;
end
