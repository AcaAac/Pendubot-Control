syms a1 a2 da1 da2 t;

% Define the constants
p1 = 0.0148;
p2 = 0.0051;
p3 = 0.0046;
p4 = 0.1003;
p5 = 0.0303;
g  = 9.81;
k = 3.9621/8;

% Define matrices M, Vm, G
M  = [p1+p2+2*p3*cos(a2-a1), p2+p3*cos(a2-a1); p2+p3*cos(a2-a1), p2];
Vmda = p3*sin(a2-a1) * [da1^2-da1*da2-da2^2; da1^2];
G  = [p4*g*sin(a1); p5*g*sin(a2)];

% Define the state vector and input
x = [a1; a2; da1; da2];

% Define the nonlinear dynamics f(x) and g(x)
f1 = da1;
f2 = da2;
f34 = M\(-Vmda-G);
f3 = f34(1);
f4 = f34(2);

g34 = mldivide(M, [k; 0]);
g3 = g34(1);
g4 = g34(2);

f = [f1; f2; f3; f4];
g = [0;0;g3;g4];

% Compute the Jacobian matrix A
A = jacobian(f, x) + jacobian(g, x)*t;

% Compute the Jacobian matrix B
B = g;

% Define the output vector y (in this case, all state variables are outputs)
C = eye(length(x));

% Set the equilibrium point
x_e = [4*pi/5; pi; 0; 0];
t_e = 10; % change

% Substitute the equilibrium point into the matrices A, B, and C
A_e = double(subs(A, {a1, a2, da1, da2, t}, {x_e(1), x_e(2), x_e(3), x_e(4), t_e}));
B_e = double(subs(B, {a1, a2, da1, da2, t}, {x_e(1), x_e(2), x_e(3), x_e(4), t_e}));
C_e = C;
D_e = [0;0;0;0];

% Construct the linear state-space representation
sys = ss(A_e, B_e, C_e, D_e);

% Display the matrices
disp('A_e:');
disp(A_e);

disp('B_e:');
disp(B_e);

disp('C_e:');
disp(C_e);

% Control Law
Q = [1, 0, 0, 0; 0, 1, 0, 0; 0, 0, 1, 0; 0, 0, 0, 1];
R = 1;
K = lqr(A_e, B_e, Q, R);

% Displaying
disp('Linear State-Space System:');
disp(sys);
disp('LQR, K :');
disp(K);
