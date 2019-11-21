%% Symbolic Setup
syms theta phiL phiR real
syms dtheta dphiL dphiR real
syms tauL tauR real

q = [theta;phiL;phiR];
dq = [dtheta;dphiL;dphiR];
Y = [0;tauL;tauR];

syms mw Iw mb Ib real % Mass and Inertia of wheels, Mass and Inertia of body
syms r l b g real% Radius of wheel, length of body shaft, baseline between wheels, gravity

%% Potential Energy

V = mb*g*l*cos(theta);
N = jacobian(V,q)';

%% Kinetic Energy

% dxl = r*dphiL; % Linear velocity of left wheel
% Tl = 1/2 * mw * dxl.^2 + 1/2 * Iw * (dphiL + dtheta).^2; % Assuming wheel phi's are measured from theta
% 
% dxr = r*dphiR; % Linear velocity of right wheel
% Tr = 1/2 * mw * dxr.^2 + 1/2 * Iw * (dphiR + dtheta).^2;
% 
% dxb = dxl/2 + dxr/2 + l*cos(theta)*dtheta; % Forward velocity
% dyb = -l*sin(theta)*dtheta; % Downward velocity
% Tb = 1/2 * mb * (dxb.^2 + dyb.^2) + 1/2 * Ib * dtheta.^2;
% 
% T = simplify(Tl + Tr + Tb);

Jl = [0 r 0; 1 1 0];
Jr = [0 r 0; 1 0 1];
Jb = [l*cos(theta) r/2 r/2; -l*sin(theta) 0 0; 1 0 0];

Ml = Jl'*diag([mw Iw])*Jl;
Mr = Jr'*diag([mw Iw])*Jr;
Mb = Jb'*diag([mb mb Ib])*Jb;

M_bar = simplify(Ml+Mr+Mb); % System inertia matrix
C_bar = get_coriolis_matrix(M_bar, q, dq);

%% Save symbolic functions as Matlab functions
matlabFunction(M_bar,'File','compute_M_bar','Comments','Version: 1.1')
matlabFunction(C_bar,'File','compute_C_bar','Comments','Version: 1.1')
matlabFunction(N,'File','compute_N','Comments','Version: 1.1')

%% Combine into symbolic expression for accelerations
ddq = simplify(inv(M_bar)*(Y - C_bar*dq - N));

%% Plug in numbers
mw_n = 0.05;
Iw_n = 0.0005;
mb_n = 1;
Ib_n = 0.01;

r_n = 0.025;
l_n = 0.1;
b_n = 0.15;
g_n = 9.81;

sym_vars = [mw Iw mb Ib r l b g];
num_vars = [mw_n Iw_n mb_n Ib_n r_n l_n b_n g_n];
ddq_n = simplify(subs(ddq, sym_vars, num_vars), 5);
matlabFunction(ddq_n,'File','compute_ddq','Comments','Version: 1.1', 'Vars', [theta dtheta tauL tauR])

%% Compute Linear Update Model
lin_sym_vars = [theta;dtheta;tauL;tauR];
lin_num_vars = [0;0;0;0];
grad1 = gradient(ddq_n(1), lin_sym_vars);
grad2 = gradient(ddq_n(2), lin_sym_vars);
grad3 = gradient(ddq_n(3), lin_sym_vars);

J = jacobian(ddq_n, lin_sym_vars);
J_n = double(subs(J, lin_sym_vars, lin_num_vars));

ddq_lin = simplify(subs(ddq_n, lin_sym_vars, lin_num_vars) + J_n * (lin_sym_vars - lin_num_vars),5);
matlabFunction(ddq_lin,'File','compute_linearized_ddq','Comments','Version: 1.1', 'Vars', lin_sym_vars)

%% Test linearization locally
thetas_test = -0.5:0.001:0.5;
zeros_test = zeros(size(thetas_test));
ddq_lin_test = compute_linearized_ddq(thetas_test, zeros_test, zeros_test, zeros_test);
ddq_test = compute_ddq(thetas_test, zeros_test, zeros_test, zeros_test);

figure(1)

subplot(1,3,1)
hold on
plot(thetas_test,ddq_lin_test(1,:))
plot(thetas_test,ddq_test(1,:))
title("Linearized ddTheta")

subplot(1,3,2)
hold on
plot(thetas_test,ddq_lin_test(2,:))
plot(thetas_test,ddq_test(2,:))
title("Linearized ddPhiL")

subplot(1,3,3)
hold on
plot(thetas_test,ddq_lin_test(3,:))
plot(thetas_test,ddq_test(3,:))
title("Linearized ddPhiR")

%% Test in Simulation
tspan = [0 2];

% Theta PhiL PhiR
q0 = [0.2;0;0];

% dTheta dPhiL dPhiR
dq0 = [0;0;0];

[t,y] = ode45(@state_update_fn,tspan,[q0;dq0]);

figure(2)

subplot(2,3,1)
plot(t,y(:,1))
title("Theta")

subplot(2,3,2)
plot(t,y(:,2))
title("PhiL")

subplot(2,3,3)
plot(t,y(:,3))
title("PhiR")

subplot(2,3,4)
plot(t,y(:,4))
title("dTheta")

subplot(2,3,5)
plot(t,y(:,5))
title("dPhiL")

subplot(2,3,6)
plot(t,y(:,6))
title("dPhiR")

