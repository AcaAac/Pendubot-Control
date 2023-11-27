k_factor = 42.058;
a = 0.531;
u_1 = 1.49;
x_1 = 0.0244;
m = 20.7;
g = 9.81;
A = [0 1; 2 * (k_factor/m) * u_1 / (x_1 + a)^3 0];
B = [0; -(k_factor/m) * 1 / (x_1 + a)^2];
C = [1 0];

% Controller
Q = [1 0; 
    0 1];
R = 1;
K = lqr(A, B, Q, R);

sys = ss((A - B * K), B, C, 0);

% Initial Conditions
x0 = [1; 0];  % Initial state [position; velocity]

% Simulation parameters
timestep = 0.01;  % Adjust as needed
tspan = 0:timestep:5;  % Adjust the simulation time as needed

% Create a figure and axes for plotting
figure;
subplot(2, 1, 1);
title('Position of the Ball');
xlabel('Time');
ylabel('Position');
grid on;
hold on;
subplot(2, 1, 2);
title('Velocity of the Ball');
xlabel('Time');
ylabel('Velocity');
grid on;
hold on;

for t = tspan
    % Update the control input using the state feedback controller
    u = -K * x0;

    % Update the state using the system dynamics
    xdot = A * x0 + B * u;
    x0 = x0 + xdot * timestep;

    % Get position and velocity
    position = x0(1);
    velocity = x0(2);

    % Update and plot the data
    subplot(2, 1, 1);
    plot(t, position, 'r.');
    drawnow;

    subplot(2, 1, 2);
    plot(t, velocity, 'b.');
    drawnow;
end