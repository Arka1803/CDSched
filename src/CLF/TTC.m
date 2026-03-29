clc; clear; yalmip('clear');
h = 0.2;

%Discrete-time plant matrices
Ad_nom = [1, 0.099591; 
          0, 0.99005]; 

Bd_nom = [0.00498337; 
          0.09950166];

[Ac, Bc] = d2c(Ad_nom, Bd_nom, h);

% Compute nominal gain

Q = diag([10 1]);
R = 1;
K_nom = dlqr(Ad_nom, Bd_nom, Q, R);
% Delay-Aware Augmented System 

Dn = [0.08, 0, 0.05, 0, 0.05, 0.08, 0.05, 0, 0.05, 0]; %Delay vector
num_d = length(Dn);

Q_aug = diag([10 1 1]); 
R_aug = 1;

A_sys = cell(num_d,1);
B_sys = cell(num_d,1);
K_stack = cell(num_d,1);
A_cl_stack = cell(num_d,1);

fprintf('Designing Delay-Aware Controllers...\n');

for i = 1:num_d
    
    if Dn(i) == 0
        % Augmented zero-delay system
        A_sys{i} = [Ad_nom Bd_nom;
                    zeros(1,2) 0];
        B_sys{i} = [zeros(2,1);
                    1];
    else
        % Fractional input delay
        sys_c = ss(Ac, Bc, eye(2), 0);
        sys_c.InputDelay = Dn(i);
        sys_d = absorbDelay(c2d(sys_c, h));
        [Ad_aug, Bd_aug] = ssdata(sys_d);
        
        A_sys{i} = Ad_aug;
        B_sys{i} = Bd_aug;
    end
    
    % LQR gain for augmented system
    K_stack{i} = dlqr(A_sys{i}, B_sys{i}, Q_aug, R_aug);
    A_cl_stack{i} = A_sys{i} - B_sys{i}*K_stack{i};
end

%% ============================================================
% 4. Common Lyapunov Function (Switching Stability)
% =============================================================

Pm = sdpvar(3,3);
decay = 0.95;

Constraints = [Pm >= 1e-4*eye(3)];

for i = 1:num_d
    Constraints = [Constraints, ...
        A_cl_stack{i}'*Pm*A_cl_stack{i} - decay*Pm <= -1e-6*eye(3)];
end

sol = optimize(Constraints, [], sdpsettings('solver','mosek','verbose',1));

if sol.problem ~= 0
    error('Common Lyapunov Function NOT found');
end

P_val = value(Pm);
fprintf('CLF Found Successfully.\n');

%% ============================================================
% 5. Simulation
% =============================================================

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

% Choose switching type:
random_switching = false;

for k = 1:SimSteps
    
    % ---------- TRUE NOMINAL ----------
    X_log_nom(:,k) = x_nom;
    
    uk_nom = -K_nom * x_nom;
    U_log_nom(k) = uk_nom;
    
    x_nom = Ad_nom*x_nom + Bd_nom*uk_nom;
    
    % ---------- DELAY-AWARE SWITCHING ----------
    
    if random_switching
        idx = randi(num_d);
    else
        idx = mod(k-1, num_d) + 1;  % cyclic
    end
    
    X_log_del(:,k) = x_del;
    
    z_del = [x_del; u_prev_del];
    uk_del = -K_stack{idx} * z_del;
    U_log_del(k) = uk_del;
    
    z_next = A_sys{idx}*z_del + B_sys{idx}*uk_del;
    
    x_del = z_next(1:2);
    u_prev_del = uk_del;
end

%% ============================================================
% 6. Performance Metrics
% =============================================================

% Integral Squared Error (ISE)
ISE_nom = sum(X_log_nom(1,:).^2)*h;
ISE_del = sum(X_log_del(1,:).^2)*h;

% Control Effort
Effort_nom = sum(U_log_nom.^2)*h;
Effort_del = sum(U_log_del.^2)*h;

% Settling Time (2% of initial position)
threshold = 0.02 * abs(x0(1));

st_nom_idx = find(abs(X_log_nom(1,:)) > threshold, 1, 'last');
st_del_idx = find(abs(X_log_del(1,:)) > threshold, 1, 'last');

if isempty(st_nom_idx), st_nom_idx = 0; end
if isempty(st_del_idx), st_del_idx = 0; end

ST_nom = time(min(st_nom_idx+1, SimSteps));
ST_del = time(min(st_del_idx+1, SimSteps));

%% ============================================================
% 7. Display Performance Table
% =============================================================

fprintf('\n===========================================\n');
fprintf('     CONTROL PERFORMANCE COMPARISON\n');
fprintf('===========================================\n');

Criteria = {'ISE (Position)';
            'Control Effort';
            'Settling Time (s)'};

Nominal_Perf = [ISE_nom;
                Effort_nom;
                ST_nom];

Switched_Perf = [ISE_del;
                 Effort_del;
                 ST_del];

T = table(Criteria, Nominal_Perf, Switched_Perf);
disp(T);

%% ============================================================
% 8. Plot Results
% =============================================================

figure('Color','w','Position',[100 100 900 700]);

subplot(2,1,1);
plot(time, X_log_nom(1,:), '--k','LineWidth',1.5); hold on;
plot(time, X_log_del(1,:), 'b','LineWidth',2);
grid on;
ylabel('Position (m)');
legend('Nominal (Zero Delay)','Delay-Aware Switching');
title('Trajectory Comparison');

subplot(2,1,2);
stairs(time, U_log_nom,'--k','LineWidth',1); hold on;
stairs(time, U_log_del,'r','LineWidth',1.5);
grid on;
ylabel('Force (N)');
xlabel('Time (s)');
title('Control Effort');
