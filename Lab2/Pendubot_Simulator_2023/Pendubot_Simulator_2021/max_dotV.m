function max_dV = max_dotV(K)
    % Grid of test points
    [X1, X2, X3, X4] = ndgrid(-2:0.2:2, -2:0.2:2, -2:0.2:2, -2:0.2:2);
    max_dV = -inf;

    % Define the system parameters
    p1 = 0.0148; p2 = 0.0051; p3 = 0.0046; p4 = 0.1003; p5 = 0.0303;
    g = 9.81; % acceleration due to gravity
    k = 3.9621/8;
    
    % Define the nonlinear dynamics f(x) and g(x)
    % f1 = X3;
    % f2 = X4;
    % f34 = M\(-Vmda-G);
    % f3 = f34(1);
    % f4 = f34(2);
    % 
    % g34 = M\([k; 0]);
    % g3 = g34(1);
    % g4 = g34(2);

    % Evaluate dot(V(x)) at each point and find the maximum
    for i = 1:numel(X1)
        x = [X1(i); X2(i); X3(i); X4(i)];

        % Define matrices M, Vm, G
        M  = [p1+p2+2*p3*cos(x(2)-x(1)), p2+p3*cos(x(2)-x(1)); p2+p3*cos(x(2)-x(1)), p2];
        Vmda = p3*sin(x(2)-x(1)) * [x(3)^2-x(3)*x(4)-x(4)^2; x(3)^2];
        G  = [p4*g*sin(x(1)); p5*g*sin(x(2))];
        
        % Define Control affine Function
        f34 = M\(-Vmda-G);
        f_x = [x(3) ; x(4) ; f34(1) ; f34(2)];
        g34 = M\([k; 0]);
        g_x = [0; 0; g34(1); g34(2)];

        % Define Lyapunov Function
        
        % Potential energy V
        U = -p4*g*cos(x(1)) - p5*g*cos(x(2));
    
        % Kinetic energy T
        T = 0.5 * [x(3) x(4)] * M * [x(3); x(4)];
    
        % Total energy
        E = T + U;

        V_lyapu = E;

        dotV = [x(3) , x(4)] * (M * [f_x(3) + g_x(3) * (-K * x) ; ...
            f_x(4) + g_x(4) * (-K * x)] + [g * p4 * sin(x(1)) ; g * p5 * sin(x(2))]);
        if dotV > max_dV
            max_dV = dotV;
        end
    end
end