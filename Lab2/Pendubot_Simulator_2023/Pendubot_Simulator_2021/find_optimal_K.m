function k_optimized = find_optimal_K()
    % Initial guess for K
    K0 = [1, 1, 1, 1];

    % Using fmincon to minimize the maximum of dot(V(x))
    options = optimset('Display', 'iter', 'Algorithm', 'sqp');
    k_optimized = fmincon(@max_dotV, K0, [], [], [], [], [0, 0, 0, 0], [], [], options);
end

