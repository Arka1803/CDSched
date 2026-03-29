clc; clear; yalmip('clear');

%% ============================================================
% 1. Plant Definition
% ============================================================

h = 0.2;

Ad_nom = [1, 0.099591; 
          0, 0.99005];

Bd_nom = [0.00498337; 
          0.09950166];

[Ac, Bc] = d2c(Ad_nom, Bd_nom, h);

nx = size(Ac,1);

Q = diag([10 1]);
R = 1;

K_nom = dlqr(Ad_nom, Bd_nom, Q, R);

% 3. Delay-Aware Augmented Systems


Dn = [0.08, 0, 0.05, 0, 0.05, 0.08, 0.05, 0, 0.05, 0];
num_d = length(Dn);

Q_aug = diag([10 1 1]);
R_aug = 1;

% First-order approximation (Eq. 3 in paper)
Phi = eye(nx) + Ac*h;

A_sys = cell(num_d,1);
B_sys = cell(num_d,1);
K_stack = cell(num_d,1);
A_cl_stack = cell(num_d,1);

fprintf('Designing Delay-Aware Controllers (Paper Model)...\n');

for i = 1:num_d
    
    delta = Dn(i);
    
    % Γ0(δ) and Γ1(δ)
    Gamma0 = (h - delta)*Bc;
    Gamma1 = delta*Bc;
    
    % Augmented matrices 
    A_sys{i} = [Phi          Gamma1;
                zeros(1,nx)      0];
    
    B_sys{i} = [Gamma0;
                1];
    
    % LQR gain
    K_stack{i} = dlqr(A_sys{i}, B_sys{i}, Q_aug, R_aug);
    
    % Closed-loop matrix
    A_cl_stack{i} = A_sys{i} - B_sys{i}*K_stack{i};
end

% 4. Common Lyapunov Function (Switching Stability)
Pm = sdpvar(3,3);

%alpha = 0.1;                      % continuous-time decay rate
decay=0.98;
%decay = exp(-alpha*h);            % discrete contraction

Constraints = [Pm >= 1e-4*eye(3)];

for i = 1:num_d
    Constraints = [Constraints, ...
        A_cl_stack{i}'*Pm*A_cl_stack{i} ...
        <= decay*Pm];
end

sol = optimize(Constraints, [], ...
    sdpsettings('solver','mosek','verbose',0));

if sol.problem ~= 0
    error('Common Lyapunov Function NOT found');
end

fprintf('CLF Found Successfully.\n');

%% ============================================================
% 5. Simulation
% ============================================================

SimSteps = 100;
time = (0:SimSteps-1)*h;

x0 = [10; 2];

x_nom = x0;
x_del = x0;

u_prev_del = 0;

X_log_nom = zeros(2,SimSteps);
X_log_del = zeros(2,SimSteps);

U_log_nom = zeros(1,SimSteps);
U_log_del = zeros(1,SimSteps);

random_switching = false;

for k = 1:SimSteps
    
    X_log_nom(:,k) = x_nom;
    
    u_nom = -K_nom*x_nom;
    U_log_nom(k) = u_nom;
    
    x_nom = Ad_nom*x_nom + Bd_nom*u_nom;
    
    %switch
    
    if random_switching
        idx = randi(num_d);
    else
        idx = mod(k-1, num_d) + 1;   % cyclic switching
    end
    
    X_log_del(:,k) = x_del;
    
    z = [x_del; u_prev_del];
    
    u_del = -K_stack{idx}*z;
    U_log_del(k) = u_del;
    
    z_next = A_sys{idx}*z + B_sys{idx}*u_del;
    
    x_del = z_next(1:nx);
    u_prev_del = u_del;
end

%% ============================================================
% 6. Performance Metrics
% ============================================================

% ISE_nom = sum(X_log_nom(1,:).^2)*h;
% ISE_del = sum(X_log_del(1,:).^2)*h;
% 
% Effort_nom = sum(U_log_nom.^2)*h;
% Effort_del = sum(U_log_del.^2)*h;
% 
% threshold = 0.02 * abs(x0(1));
% 
% st_nom_idx = find(abs(X_log_nom(1,:)) > threshold, 1, 'last');
% st_del_idx = find(abs(X_log_del(1,:)) > threshold, 1, 'last');
% 
% if isempty(st_nom_idx), st_nom_idx = 0; end
% if isempty(st_del_idx), st_del_idx = 0; end
% 
% ST_nom = time(min(st_nom_idx+1, SimSteps));
% ST_del = time(min(st_del_idx+1, SimSteps));

%% ============================================================
% 7. Display Performance Table
% ============================================================

% fprintf('\n===========================================\n');
% fprintf('     CONTROL PERFORMANCE COMPARISON\n');
% fprintf('===========================================\n');
% 
% Criteria = {'ISE (Position)';
%             'Control Effort';
%             'Settling Time (s)'};
% 
% Nominal_Perf = [ISE_nom;
%                 Effort_nom;
%                 ST_nom];
% 
% Switched_Perf = [ISE_del;
%                  Effort_del;
%                  ST_del];
% 
% T = table(Criteria, Nominal_Perf, Switched_Perf);
% disp(T);

%% ============================================================
% 8. Plot Results
% ============================================================

figure('Color','w','Position',[100 100 900 700]);

subplot(2,1,1);
plot(time, X_log_nom(1,:), '--k','LineWidth',1.5); hold on;
plot(time, X_log_del(1,:), 'b','LineWidth',2);
grid on;
ylabel('Position (m)');
legend('Zero Delay','Delay-Aware Switching');
title('State Plot');

subplot(2,1,2);
stairs(time, U_log_nom,'--k','LineWidth',1); hold on;
stairs(time, U_log_del,'r','LineWidth',1.5);
grid on;
ylabel('Force (N)');
xlabel('Time (s)');
title('Control Input');
