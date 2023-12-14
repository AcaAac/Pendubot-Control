function dx = lyapunovPendulumDynamics(t, x, k, E_ref, p1, p2, p3, p4, p5, g)
    % Unpack the state variables
    alpha1 = x(1);
    alpha2 = x(2);
    dalpha1 = x(3);
    dalpha2 = x(4);

    % Mass matrix M(alpha)
    M = [p1 + p2 + 2*p3*cos(alpha2 - alpha1), p2 + p3*cos(alpha2 - alpha1);
         p2 + p3*cos(alpha2 - alpha1), p2];

    % Potential energy V
    V = p4*g*cos(alpha1) + p5*g*cos(alpha2);

    % Kinetic energy T
    T = 0.5 * [dalpha1 dalpha2] * M * [dalpha1; dalpha2];

    % Total energy
    E = T + V;

    % Lyapunov function V
    V_lyapunov = 0.5 * (E - E_ref)^2;

    % Calculate the derivative of the Lyapunov function
    dV_lyapunov = (E - E_ref) * (dalpha1*(p1+p2+2*p3*cos(alpha2-alpha1))*dalpha1 + dalpha2*(p2+p3*cos(alpha2-alpha1))*dalpha2 - g*(p4*sin(alpha1)*dalpha1 + p5*sin(alpha2)*dalpha2));

    % Control input u(t) based on the derivative of Lyapunov function
    u = -k * dV_lyapunov;

    % Torque, with saturation
    tau_max = 10; % Adjust based on system capabilities
    tau = min(max(u, -tau_max), tau_max);

    % Inverse of the mass matrix
    M_inv = inv(M);

    % Gravity vector G
    G = [p4*g*sin(alpha1); p5*g*sin(alpha2)];

    % Input matrix
    B = [1; 0];

    % Acceleration [dalpha1_ddot; dalpha2_ddot]
    accel = M_inv * (B * tau - G);

    % Constructing the state derivative vector
    dx = zeros(4,1);
    dx(1) = dalpha1; % alpha1_dot
    dx(2) = dalpha2; % alpha2_dot
    dx(3) = accel(1); % alpha1_ddot
    dx(4) = accel(2); % alpha2_ddot
end
