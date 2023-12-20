function u=C3(alpha1r,x)
%function u=pendubot_reg(alpha21,x)
%
%INPUTS:
% alpha1r : reference value for the first link angle [rad]
% x       : measured state vector x =   [
%										x1 = alpha_1 - pi
%										x2 = alpha_2 - pi
%										x3 = dalpha_1/dt
%										x4 = dalpha_2/dt
%                                       ]
%
%OUTPUT:
% u       : control signal (motor torque) to be applied [Nm]

%Set the equilibrium point
x_e = [pi; pi; 0; 0];
t_e = 0;
if abs(x(1)) > 0.01 && abs(x(1)+x(2)) > 0.2
    
    a1 = x(1);
    a2 = x(2);
    da1 = x(3);
    da2 = x(4);

    % Define Constants
    p1 = 0.0148;
    p2 = 0.0051;
    p3 = 0.0046;
    p4 = 0.1003;
    p5 = 0.0303;
    grav  = 9.81;
    torque = 3.9621;
    k = 3.9621/8;
    % tau = [torque ; 0];

    % Dynamic Matrices

    M  = [p1+p2+p3*cos(a2), p2+p3*cos(a2); p2+p3*cos(a2), p2];
    C = p3*sin(a2) * [-da2 , -da1 - da2 ; -da1 , 0];
    G  = [p4*grav*cos(a1) + p5 * grav * cos(a1 + a2); p5*grav*cos(a2 + a1)];
    F = [0.00545 * da1 + 0.0023 * tanh(30 * da1) ; 0.00047 * da2 + 0.0025 * tanh(30 * da2)];

    % Energy Data

    U = p4 * sin(a1) + p5 * sin(a1 + a2);

    K = 0.5 * [da1 , da2] * M * [da1 ; da2];
    % dK = [da1 , da2] * (tau - C * [da1 ; da2] - G - F) + 0.5 * [da1 , da2] * dM * [da1 ; da2];

    E = K + U;

    E_TOP = (p4 + p5) * grav;

    E_est = E - E_TOP;

    % Lyapunov Data

    kk = 0.1;  %6.359;
    kd = 30; %8.71;
    a1_est = a1 - alpha1r;
    a2_est = a1 + a2 - alpha1r;

    V = kk * K + 0.5 * [a1_est , a2_est] * kd * [a1_est ; a2_est];

    % Control Law
    G_1 = p4 * grav * sin(a1);
    tau = -1/kk * (da1 + kd * a1_est) + G_1 ;
    u = tau; 
else
    K = [-0.3243  -15.7274   -1.5410   -2.2446];
    corr_x_measured = [x(1)+pi-x_e(1); x(2)+pi-x_e(2); x(3)-0; x(4)-0];
    v = -K*corr_x_measured;
    u= v+t_e;
end