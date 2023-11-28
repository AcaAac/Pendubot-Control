syms a1 a2 da1 da2 t

% Define the constants
p1 = 0.0148;
p2 = 0.0051;
p3 = 0.0046;
p4 = 0.1003;
p5 = 0.0303;
g  = 9.81;
kA = 1; % ???
kt = 1; % ???

% Define matrices M, Vm, G
M  = [p1+p2+2*p3*cos(a2-a1), p2+p3*cos(a2-a1); p2+p3*cos(a2-a1), p2];
Vm = p3*sin(a2-a1) * [da1-da2, -da2; da1, 0];
G  = [p4*g*sin(a1); p5*g*sin(a2)];

% Define the state vector and input
x = [a1; a2; da1; da2];
u = t;

% Define the nonlinear dynamics f(x) and g(x)
f1 = da1;
f2 = da2;
f34 = mldivide(M, -Vm*[da1; da2]-G);
f3 = f34(1);
f4 = f34(2);

g34 = mldivide(M, -Vm*[kA*kt; 0]);
g3 = g34(1);
g4 = g34(2);

f = [f1; f2; f3; f4];
g = [0;0;g3;g4];

% Compute the Jacobian matrix A
A = jacobian(f, x) + jacobian(g, x)*u;

% Compute the Jacobian matrix B
B = g;

% Define the output vector y (in this case, all state variables are outputs)
C = eye(length(x));

% Set the equilibrium point
x_tilde = zeros(length(x), 1);
u_tilde = 0;

% Substitute the equilibrium point into the matrices A, B, and C
A_tilde = subs(A, {a1, a2, da1, da2, u}, {x_tilde(1), x_tilde(2), x_tilde(3), x_tilde(4), u_tilde});
B_tilde = subs(B, {a1, a2, da1, da2, u}, {x_tilde(1), x_tilde(2), x_tilde(3), x_tilde(4), u_tilde});
C_tilde = subs(C, {a1, a2, da1, da2, u}, {x_tilde(1), x_tilde(2), x_tilde(3), x_tilde(4), u_tilde});

% Construct the linear state-space representation
%sys = ss(A_tilde, B_tilde, C_tilde, []);

% Display the matrices
disp('A_tilde:');
disp(A_tilde);

disp('B_tilde:');
disp(B_tilde);

disp('C_tilde:');
disp(C_tilde);

% Display the linear state-space system
%disp('Linear State-Space System:');
%disp(sys);
