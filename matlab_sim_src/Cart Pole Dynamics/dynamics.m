%% Numerical constants
Ts_n = 0.01; % Update rate (s)
mw_n = 0.5313; % Mass of wheels (kg)
Iw_n = 0.00273; % Inertia of wheels (kg*m^2)
mb_n = 8.22; % Mass of body (kg)
Ib_n = 0.57; % Inertia of body (kg*m^2)

r_n = 0.101; % Wheel radius (m)
l_n = 0.49; % Length from wheel axle to body COM (m)
b_n = 0.406; % Baseline distance between wheels (m)

g_n = 9.81; % Gravitational Constant (m/s^2)
dt = 0.01;
% Variables
syms x dx ddx phi dphi ddphi N F real

q = [x phi]';
dq = [dx dphi]';

M = 2*mw_n;
m = mb_n;
l = l_n;
I = Ib_n;
g = g_n;

% Nonlinear Dynamics Derivation
V = m*g*l*cos(phi);

J1 = [1 0; 0 0; 0 0];
M1 = diag([M M 0]);
    
J2 = [1 l*cos(phi);
     0 -l*sin(phi);
     0 1];
 
M2 = diag([m m I]);

M_comb = J1'*M1*J1 + J2'*M2*J2;

C_comb = get_coriolis_matrix(M_comb, q, dq);

N_comb = jacobian(V,q)';

U = [F;0];
ddq = simplify(inv(M_comb)*(U-C_comb*dq-N_comb), 5);

ddq_fn = matlabFunction(ddq, 'Vars', [q; dq; F]);

%% Linearize
lin_sym_vars = [q;dq];
lin_num_vars = [0;0;0;0];

J = jacobian(ddq, lin_sym_vars);
J_n = double(subs(J, lin_sym_vars, lin_num_vars));

ddq_lin = simplify(subs(ddq, lin_sym_vars, lin_num_vars) + J_n * (lin_sym_vars - lin_num_vars),5);
A_con = [zeros(2) eye(2); double(equationsToMatrix(ddq_lin, q)) zeros(2,2)];
B_con = [zeros(2,1); double(equationsToMatrix(ddq_lin, F))];

[Ad, Bd] = c2d(A_con, B_con, dt);

Q = diag([0.01 10 0.5 1]);
R = 1;

[K,S,E] = dlqr(Ad,Bd,Q,R);

%% Simulate
tspan = 0:dt:10;
y0 = [0 0 0.5 0.2]';
[tout, yout] = ode45(@(t,y) update_fn(t,y,ddq_fn, K), tspan, y0);

figure(1)
subplot(2,2,1)
plot(tout, yout(:,1))
title("X")

subplot(2,2,2)
plot(tout, yout(:,2))
title("Phi")

subplot(2,2,3)
plot(tout, yout(:,3))
title("DX")

subplot(2,2,4)
plot(tout, yout(:,4))
title("DPhi")

