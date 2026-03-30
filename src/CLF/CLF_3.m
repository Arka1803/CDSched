clc; clear; yalmip('clear');

%% 1. Plant Definition
h = 2;
Ad_nom = [1, 0.099591; 0, 0.99005];
Bd_nom = [0.00498337; 0.09950166];
[Ac, Bc] = d2c(Ad_nom, Bd_nom, h);
nx = size(Ac,1);

Q_aug = diag([10 1 1]);
R_aug = 1;
Phi = eye(nx) + Ac*h;
decay = 0.8;

%% 2. Iterative Search for Stability Limit
% We test delta from 0 up to the sampling period h
delta_test_step = 0.1; 
test_range = 0:delta_test_step:0.08;

last_stable_delta = 0;
found_failure = false;

fprintf('Testing stability for increasing delta...\n');

for d_max = test_range
    % Define the switching instances: [No delay, Max delay]
    % If a CLF exists for these two extremes, it covers the switching between them
    current_Dn = [0, d_max]; 
    num_d = length(current_Dn);
    
    A_cl_stack = cell(num_d,1);
    for i = 1:num_d
        delta = current_Dn(i);
        Gamma0 = (h - delta)*Bc;
        Gamma1 = delta*Bc;
        
        % Augmented matrices
        Ai = [Phi, Gamma1; zeros(1,nx), 0];
        Bi = [Gamma0; 1];
        
        % Design delay-aware LQR for this specific delay instance
        Ki = dlqr(Ai, Bi, Q_aug, R_aug);
        A_cl_stack{i} = Ai - Bi*Ki;
    end
    
    % --- LMI Solver (YALMIP) ---
    Pm = sdpvar(3,3);
    Constraints = [Pm >= 1e-6*eye(3)];
    for i = 1:num_d
        Constraints = [Constraints, A_cl_stack{i}'*Pm*A_cl_stack{i} <= decay*Pm];
    end
    
    % Silent solve
    sol = optimize(Constraints, [], sdpsettings('solver','mosek','verbose',0));
    
    if sol.problem == 0
        last_stable_delta = d_max;
    else
        % Print the requested values and break
        fprintf('\n===========================================\n');
        fprintf('CLF not found at: delta_max = %.4f s\n', d_max);
        fprintf('Last successful:  delta_max = %.4f s\n', last_stable_delta);
        fprintf('===========================================\n');
        found_failure = true;
        break;
    end
end

if ~found_failure
    fprintf('CLF found for all delays up to h = %.2f\n', h);
end

%% 3. Optional: Simulation for the Last Stable Case
% (You can use last_stable_delta here to run your simulation section)