function u = pendulumNonLinear(alpha1r,x)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    alpha1 = x(1);
    alpha2 = x(2);
    dalpha1 = x(3);
    dalpha2 = x(4);

    p1 = 0.0148; p2 = 0.0051; p3 = 0.0046; p4 = 0.1003; p5 = 0.0303;
    g = 9.81;
    E_ref = (p4 + p5) * g;

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
    k_p = 0.5;
    k_d = 0.1;
    % Derivative of the Lyapunov Function
    dV_lyap = [dalpha1; dalpha2]' * (0.5 * M * [dalpha1; dalpha2] + [p4*g*sin(alpha1); p5*g*sin(alpha2)]);


    % Control Law for Swing-Up using PD controller
    u = -k_p * V_lyap - k_d * dV_lyap;

end

