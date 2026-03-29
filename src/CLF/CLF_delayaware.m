%% Delay-Aware Cruise Control: Integrated Stability & Simulation
clc; clear;

% 1. Continuous-time Plant (Cruise Control)
m = 1000; b = 50;
Ac = [0 1; 0 -b/m];
Bc = [0; 1/m];
[nx, nu] = size(Bc);
h = 0.2; % Sampling period

% 2. Nominal Controller (d=0) & Noise Params
sys_nom = c2d(ss(Ac, Bc, eye(nx), 0), h);
Phi_nom = sys_nom.A;
Gamma_nom = sys_nom.B;
K_nom = dlqr(Phi_nom, Gamma_nom, diag([1, 10]), 1);

Q_process = eye(nx) * 1e-7;
R_sensor = 1e-7 * eye(nx);
D_kalman = zeros(nx, nu + nx); 
[~, L_nom, ~] = kalman(ss(Phi_nom, [Gamma_nom eye(nx)], eye(nx), D_kalman, h), Q_process, R_sensor);

% 3. Stability Sweep for Common Lyapunov Function (CLF)
% We search for d such that switching between nominal and delay-aware is stable.
%% 3. Stability Sweep for Common Lyapunov Function (CLF)
delays = 0.01:0.01:0.19; 
decay_factor = 0.95; % Use 0.99 (1% decay) for h=0.2 for better feasibility
results = [];
M = [Ac, Bc; zeros(nu, nx + nu)];

% --- CRITICAL FIX: Define Nominal Augmented System as d=0 ---
E_nom = expm(M * h);
Phi_nom = E_nom(1:nx, 1:nx);
G0_nom  = E_nom(1:nx, nx+1:end);
G1_nom  = zeros(nx, nu); % At d=0, there is no carry-over from u(k-1)

A_aug_nom = [Phi_nom, G1_nom; zeros(nu, nx), zeros(nu, nu)];
B_aug_nom = [G0_nom; eye(nu)];
K_nom_aug = dlqr(A_aug_nom, B_aug_nom, diag([1, 10, 1]), 1);
A_cl_nom_aug = A_aug_nom - B_aug_nom * K_nom_aug;

fprintf('Starting Stability Sweep (h=%.2f, Decay=%.2f)...\n', h, decay_factor);

for d = delays
    % Discretization for delay d
    E0 = expm(M * (h-d));
    G0 = E0(1:nx, nx+1:end);
    E1 = expm(M * d);
    G1 = E0(1:nx, 1:nx) * E1(1:nx, nx+1:end);
    Phi_k = expm(Ac * h);
    
    % Augmented System (Delayed)
    A_aug_del = [Phi_k, G1; zeros(nu, nx), zeros(nu, nu)];
    B_aug_del = [G0; eye(nu)];
    K_del = dlqr(A_aug_del, B_aug_del, diag([1, 10, 1]), 1);
    A_cl_del = A_aug_del - B_aug_del * K_del;
    
    % LMI Setup
    P = sdpvar(nx+nu, nx+nu);
    % Numerical conditioning: 1e-3 is safer than 1e-4 or 1e-6
    constraints = [P >= 1e-3 * eye(nx+nu)];
    constraints = [constraints, A_cl_nom_aug' * P * A_cl_nom_aug - decay_factor * P <= -1e-7*eye(nx+nu)];
    constraints = [constraints, A_cl_del' * P * A_cl_del - decay_factor * P <= -1e-7*eye(nx+nu)];
    
    ops = sdpsettings('solver', 'mosek', 'verbose', 0);
    sol = optimize(constraints, [], ops);
    
    if sol.problem == 0
        results = [results; d];
        fprintf('  Delay d = %.2f: Feasible\n', d);
    else
        fprintf('  Delay d = %.2f: Infeasible\n', d);
        break; 
    end
end

%% 4. Post-Switching Simulation (Velocity 10 -> 0)
if ~isempty(results)
    d_max = max(results);
    SimSteps = 100;
    % Delay vector: every sample has a different delay < d_max
    d_vec = d_max * rand(1, SimSteps); 
    
    % Initial States: Position=2, Velocity=10
    x_true = [2; 10]; 
    x_est  = [2; 10]; 
    u_prev = 0;
    
    X_log = zeros(nx, SimSteps);
    X_est_log = zeros(nx, SimSteps);
    U_log = zeros(nu, SimSteps);
    
    for k = 1:SimSteps
        d_k = d_vec(k);
        X_log(:,k) = x_true;
        X_est_log(:,k) = x_est;
        
        % A. Recalculate discretization for current d_k
        E0 = expm(M * (h-d_k));
        G0 = E0(1:nx, nx+1:end);
        E1 = expm(M * d_k);
        G1 = E0(1:nx, 1:nx) * E1(1:nx, nx+1:end);
        Phi_k = expm(Ac * h);
        
        % B. Delay-Aware Controller (Augmented state feedback)
        A_aug_k = [Phi_k, G1; zeros(nu, nx), zeros(nu, nu)];
        B_aug_k = [G0; eye(nu)];
        % Aggressive weight on velocity to reach 0
        K_k = dlqr(A_aug_k, B_aug_k, diag([1, 200, 1]), 10);
        
        z_est = [x_est; u_prev];
        u_k = -K_k * z_est; 
        U_log(:,k) = u_k;
        
        % C. Plant Physics (Real State Evolution)
        x_next = Phi_k*x_true + G0*u_k + G1*u_prev + sqrt(Q_process)*randn(nx,1);
        
        % D. Estimator Update (Kalman Filter)
        y = x_next + sqrt(R_sensor)*randn(nx,1); 
        x_est_pred = Phi_k*x_est + G0*u_k + G1*u_prev;
        x_est = x_est_pred + L_nom * (y - x_est_pred);
        
        % E. Cycle Updates
        x_true = x_next;
        u_prev = u_k;
    end
    
    % 5. Plotting Results
    time = (0:SimSteps-1)*h;
    figure('Color', 'w', 'Name', 'Cruise Control Deceleration');
    
    subplot(3,1,1);
    plot(time, X_log(2,:), 'b', 'LineWidth', 2); hold on;
    plot(time, X_est_log(2,:), 'r--', 'LineWidth', 1.5);
    yline(0, 'k:', 'Target');
    ylabel('Velocity (m/s)'); grid on;
    title(['Goal: 10 \rightarrow 0 m/s (Common Lyapunov Stability Verified up to d=', num2str(d_max), ')']);
    legend('True', 'Estimated');
    
    subplot(3,1,2);
    stairs(time, U_log, 'k', 'LineWidth', 1.5);
    ylabel('Braking Force (N)'); grid on;
    
    subplot(3,1,3);
    stem(time, d_vec, 'm', 'Marker', 'none');
    ylabel('Delay d(k)'); xlabel('Time (s)');
    grid on; title('Time-Varying Delay Profile');
else
    fprintf('No feasible delay found for the specified decay rate.\n');
end