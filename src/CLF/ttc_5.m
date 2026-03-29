clc; clear; yalmip('clear');

%% ============================================================
% 1. Plant Definition
% ============================================================
h = 0.2; 

Ad_nom = [1, 0.099591; 
          0, 0.99005]; 
Bd_nom = [0.00498337; 
          0.09950166];

% Recover continuous-time model
[Ac, Bc] = d2c(Ad_nom, Bd_nom, h);

nx = size(Ac,1);
nu = size(Bc,2);

%% ============================================================
% 2. Delay Parameters
% ============================================================
d_test = 0.02;      % Delay for second subsystem

decay = 0.95;                 % desired decay rate (continuous-time)
%decay = exp(-alpha*h);        % equivalent discrete contraction factor

Q_aug = diag([10, 1, 1]);
R_aug = 1;

%% ============================================================
% 3. Construct Augmented System (Paper Method)
% ============================================================

% Discrete-time approximation (Eq. 3 in paper)
Phi = eye(nx) + Ac*h;

% ----- Subsystem 1 (δ = 0) -----
delta1 = 0;

Gamma0_1 = (h - delta1)*Bc;
Gamma1_1 = delta1*Bc;

A1 = [Phi      Gamma1_1;
      zeros(1,nx)   0];

B1 = [Gamma0_1;
      1];

K1 = dlqr(A1, B1, Q_aug, R_aug);
Acl1 = A1 - B1*K1;

% ----- Subsystem 2 (δ = d_test) -----
delta2 = d_test;

Gamma0_2 = (h - delta2)*Bc;
Gamma1_2 = delta2*Bc;

A2 = [Phi      Gamma1_2;
      zeros(1,nx)   0];

B2 = [Gamma0_2;
      1];

K2 = dlqr(A2, B2, Q_aug, R_aug);
Acl2 = A2 - B2*K2;

%% ============================================================
% 4. Stability Verification (Common Lyapunov Function)
% ============================================================
Pm = sdpvar(3,3);

C = [Pm >= 1e-4*eye(3)];
C = [C, Acl1'*Pm*Acl1 <= decay*Pm];
C = [C, Acl2'*Pm*Acl2 <= decay*Pm];

sol = optimize(C, [], sdpsettings('solver','mosek','verbose',0));

if sol.problem ~= 0
    error('CLF NOT found for this delay and decay rate.');
end

fprintf('CLF Found. System is stable under switching.\n');

%% ============================================================
% 5. Simulation (Single Switch)
% ============================================================
SimSteps   = 400;
switch_step = 10;    % Switch at sample 10
time = (0:SimSteps-1)*h;

x_state = [10; 2];
u_prev  = 0;

X_log = zeros(2, SimSteps);
U_log = zeros(1, SimSteps);

for k = 1:SimSteps
    
    X_log(:,k) = x_state;
    z = [x_state; u_prev];
    
    if k < switch_step
        Ak = A1; Bk = B1; Kk = K1;
    else
        Ak = A2; Bk = B2; Kk = K2;
    end
    
    uk = -Kk * z;
    U_log(k) = uk;
    
    z_next = Ak*z + Bk*uk;
    
    x_state = z_next(1:nx);
    u_prev  = uk;
end

%% ============================================================
% 6. Performance Metrics
% ============================================================
ISE    = sum(X_log(1,:).^2)*h;
Effort = sum(U_log.^2)*h;

fprintf('\n--- Performance Metrics ---\n');
fprintf('ISE (Position): %.4f\n', ISE);
fprintf('Control Effort: %.4f\n', Effort);

%% ============================================================
% 7. Plotting
% ============================================================
figure('Color','w');

subplot(2,1,1);
plot(time, X_log(1,:), 'b', 'LineWidth', 2); hold on;
xline(time(switch_step), '--r', 'Switch Instance');
grid on;
ylabel('Position (m)');
title('Single-Switch Response (Nominal to Delayed - Augmented Model)');

subplot(2,1,2);
stairs(time, U_log, 'k', 'LineWidth', 1.5); hold on;
xline(time(switch_step), '--r');
grid on;
ylabel('Force (N)');
xlabel('Time (s)');
