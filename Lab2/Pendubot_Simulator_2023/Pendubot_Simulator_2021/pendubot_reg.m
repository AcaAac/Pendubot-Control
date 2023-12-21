function u=pendubot_reg(alpha1r,x)
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

% Define the constants
p4 = 0.1003;
grav  = 9.81;

% Set the equilibrium point
x_e = [pi; pi; 0; 0]; %[4*pi/5; pi; 0; 0];
t_e = 0; %sin(x_e(1))*p4*grav;


K = [-0.4519  -17.1142   -1.6944   -2.4519];
corr_x_measured = [wrapToPi(x(1))+pi-x_e(1); wrapToPi(x(2))+pi-x_e(2); x(3)-0; x(4)-0];
v = -K*corr_x_measured;
u= v+t_e;