function [dx, V_lyap] = pendulumDynamics(t, x, k_p, k_d, E_ref, p1, p2, p3, p4, p5, g, tau_max)
    % Unpack the state variables
    alpha1 = x(1);
    alpha2 = x(2);
    dalpha1 = x(3);
    dalpha2 = x(4);

    % Mass matrix M(alpha)
    M = [p1 + p2 + 2*p3*cos(alpha2 - alpha1), p2 + p3*cos(alpha2 - alpha1);
         p2 + p3*cos(alpha2 - alpha1), p2];

    % Potential energy V
    V = -p4*g*cos(alpha1) - p5*g*cos(alpha2);

    % Kinetic energy T
    T = 0.5 * [dalpha1 dalpha2] * M * [dalpha1; dalpha2];

    % Total energy
    E = T + V;

    % Lyapunov Function (Energy-based)
    V_lyap = (E - E_ref);

    % Derivative of the Lyapunov Function
    dV_lyap = [dalpha1; dalpha2]' * (0.5 * M * [dalpha1; dalpha2] + [p4*g*sin(alpha1); p5*g*sin(alpha2)]);


    % Control Law for Swing-Up using PD controller
    u = -k_p * V_lyap - k_d * dV_lyap;

    % Torque, with saturation limit
    tau = min(max(u, -tau_max), tau_max);

    % Inverse of the mass matrix
    M_inv = inv(M);

    % Gravity vector G
    G = [p4*g*sin(alpha1); p5*g*sin(alpha2)];

    % Input matrix B
    B = [1; 0];

    % Acceleration [dalpha1_ddot; dalpha2_ddot]
    accel = M_inv * (B * tau - G);

    % Constructing the state derivative vector
    dx = zeros(4,1);
    % integrate
    dx(1) = dalpha1; % alpha1_dot
    dx(2) = dalpha2;
    dx(3) = accel(1); % alpha1_ddot
    dx(4) = accel(2); % alpha2_ddot
end
