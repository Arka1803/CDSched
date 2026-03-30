clc; clear; yalmip('clear');
h = 0.02; 
Ad_nom = [1, 0.099591; 0, 0.99005]; 
Bd_nom = [0.00498337; 0.09950166];
[Ac, Bc] = d2c(Ad_nom, Bd_nom, h);

% --- THE CHANGE: Tighten the Decay Rate ---
% If decay is 0.84, it's very easy to satisfy. 
% Let's see what the actual eigenvalues are.
decay = 0.6276558699; 
d_design = 0.001; 
Q_aug = diag([50, 10, 1]); R_aug = 0.1; % Higher gain makes it more sensitive

% Subsystem 1: Nominal
A1_nom = [Ad_nom, Bd_nom; zeros(1,3)];
B1_nom = [zeros(2,1); 1];
K1 = dlqr(A1_nom, B1_nom, Q_aug, R_aug);
Acl1 = A1_nom - B1_nom*K1;
rho1 = max(abs(eig(Acl1)))^2;

% Subsystem 2 Design
sys_c_design = ss(Ac, Bc, eye(2), 0);
sys_c_design.InputDelay = d_design;
sys_d_design = absorbDelay(c2d(sys_c_design, h));
[A2_design, B2_design] = ssdata(sys_d_design);
K2_fixed = dlqr(A2_design, B2_design, Q_aug, R_aug);

fprintf('System 1 Natural Decay (rho): %.4f\n', rho1);
fprintf('Target Decay Rate: %.4f\n\n', decay);

current_delay = 0.001; 
while current_delay <= 0.009
    sys_c_actual = ss(Ac, Bc, eye(2), 0);
    sys_c_actual.InputDelay = current_delay;
    sys_d_actual = absorbDelay(c2d(sys_c_actual, h));
    [A_actual, B_actual] = ssdata(sys_d_actual);
    
    Acl2_actual = A_actual - B_actual * K2_fixed;
    rho2 = max(abs(eig(Acl2_actual)))^2;
    
    % --- THE LMI ---
    Pm = sdpvar(3,3);
    Constraints = [Pm >= 1e-4*eye(3)];
    Constraints = [Constraints, Acl1'*Pm*Acl1 <= decay*Pm];
    Constraints = [Constraints, Acl2_actual'*Pm*Acl2_actual <= decay*Pm];
    
    sol = optimize(Constraints, [], sdpsettings('solver','mosek','verbose',0));
    
    if sol.problem == 0
        fprintf('Delay %.3fs | rho2: %.4f | CLF: YES\n', current_delay, rho2);
        current_delay = current_delay + 0.001;
    else
        fprintf('Delay %.3fs | rho2: %.4f | CLF: NO (FAILED)\n', current_delay, rho2);
        break;
    end
end