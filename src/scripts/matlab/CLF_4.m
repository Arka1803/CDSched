clc; clear; yalmip('clear');

%% 1. Plant Definition
h = 0.2; 
Ad_nom = [1, 0.099591; 0, 0.99005]; 
Bd_nom = [0.00498337; 0.09950166];
[Ac, Bc] = d2c(Ad_nom, Bd_nom, h);

%% 2. Design Two Subsystems
d_test = 0.05; % Delay for the second subsystem
decay = 0.85; % xnew = x_old * decay
Q_aug = diag([5, 1, 1]); R_aug = 1;

% Subsystem 1: Nominal (d=0)
A1 = [Ad_nom, Bd_nom; zeros(1,3)];
B1 = [zeros(2,1); 1];
K1 = dlqr(A1, B1, Q_aug, R_aug);
Acl1 = A1 - B1*K1;

% Subsystem 2: Delayed (d=d_test)
sys_c = ss(Ac, Bc, eye(2), 0);
sys_c.InputDelay = d_test;
sys_d = absorbDelay(c2d(sys_c, h));
[A2, B2] = ssdata(sys_d);
K2 = dlqr(A2, B2, Q_aug, R_aug);
Acl2 = A2 - B2*K2;

%% 3. Stability Verification (CLF)
Pm = sdpvar(3,3);
C = [Pm >= 1e-4*eye(3)];
C = [C,  '*Pm*Acl1 <= decay*Pm];
C = [C, Acl2'*Pm*Acl2 <= decay*Pm];

sol = optimize(C, [], sdpsettings('solver','mosek','verbose',0));
if sol.problem ~= 0, error('CLF NOT found for this delay and decay rate.'); end
fprintf('CLF Found. System is stable under switching.\n');

%% 4. Simulation (One-time switch at step 10)
SimSteps = 60;
switch_step = 10; % Switch at t = 2.0s
time = (0:SimSteps-1)*h;
x_state = [5; 2]; u_prev = 0;
X_log = zeros(2, SimSteps); U_log = zeros(1, SimSteps);

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
    x_state = z_next(1:2);
    u_prev = uk;
end

%% 5. Performance Metrics & Plotting
ISE = sum(X_log(1,:).^2)*h;
Effort = sum(U_log.^2)*h;

fprintf('\n--- Performance Metrics ---\n');
fprintf('ISE (Position): %.4f\n', ISE);
fprintf('Control Effort: %.4f\n', Effort);

figure('Color','w');
subplot(2,1,1);
plot(time, X_log(1,:), 'b', 'LineWidth', 2); hold on;
xline(time(switch_step), '--r', 'Switch Instance');
grid on; ylabel('Position (m)'); title('Single-Switch Response (Nominal to Delayed)');

subplot(2,1,2);
stairs(time, U_log, 'k', 'LineWidth', 1.5); hold on;
xline(time(switch_step), '--r');
grid on; ylabel('Force (N)'); xlabel('Time (s)');