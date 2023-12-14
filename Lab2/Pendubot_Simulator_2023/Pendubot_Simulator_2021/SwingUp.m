% Define the system parameters
p1 = 0.0148; p2 = 0.0051; p3 = 0.0046; p4 = 0.1003; p5 = 0.0303;
g = 9.81; % acceleration due to gravity

% Control parameters
k_swing = 0.6; % Gain for swing-up phase
k_deriv = 0; % Derivative gain
tau_max = 3.8; % Torque limit

% Reference energy at the upright position
E_ref = (p4 + p5) * g;

% Initial conditions: [alpha1, alpha2, dalpha1, dalpha2]
x0 = [0; 0; 0; 0];

% Time span for the simulation
tspan = [0, 20];

% Run the simulation
options = odeset('RelTol', 1e-5, 'AbsTol', 1e-8);
[t, x, V_lyap] = ode45(@(t, x) pendulumDynamics(t, x, k_swing, k_deriv, E_ref, p1, p2, p3, p4, p5, g, tau_max), tspan, x0, options);

% Plot results
figure;
subplot(3,1,1);
plot(t, x(:,1:2));
title('Pendulum Angles');
xlabel('Time (s)');
ylabel('Angle (rad)');
legend('alpha1', 'alpha2');

subplot(3,1,2);
plot(t, x(:,3:4));
title('Pendulum Angular Velocities');
xlabel('Time (s)');
ylabel('Angular Velocity (rad/s)');
legend('dalpha1', 'dalpha2');

subplot(3,1,3);
plot(t, V_lyap);
title('Lyapunov Function');
xlabel('Time (s)');
ylabel('V');
