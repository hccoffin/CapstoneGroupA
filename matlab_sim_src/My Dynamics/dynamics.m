R = 0.10;
L = 0.50;
mw = 0.5;
mb = 8;
Iw = 0.05;
Ib = 0.6;

g = 9.81;

syms theta phiL phiR dtheta dphiL dphiR real
syms tauL tauR real

q = [phiL;phiR;theta];
dq = [dphiL;dphiR;dtheta];

Y = [tauL; tauR; -tauL-tauR];

V = mb*g*L*cos(theta);

J = [R/2 R/2 0;
     R/2 R/2 L*cos(theta);
     0 0 -L*sin(theta);
     1 0 0;
     0 1 0;
     0 0 1];
 
M = diag([2*mw, mb, mb, Iw, Iw, Ib+mb*L^2]);

M_bar = J'*M*J;

C_bar = get_coriolis_matrix(M_bar,q,dq);

N_bar = jacobian(V,q)';

ddq = inv(M_bar)*(Y-C_bar*dq-N_bar);


%% Compute Linear Update Model
lin_sym_vars = [q;dq;Y(1:2)];
lin_num_vars = [0;0;0;0;0;0;0;0];

J_ddq = jacobian(ddq, lin_sym_vars);
J_ddq_n = double(subs(J_ddq, lin_sym_vars, lin_num_vars));

ddq_lin = simplify(subs(ddq, lin_sym_vars, lin_num_vars) + J_ddq_n * (lin_sym_vars - lin_num_vars),5);

matlabFunction(ddq_lin,'File','compute_linearized_ddq','Comments','Version: 1.0', 'Vars', {lin_sym_vars})
matlabFunction(ddq,'File','compute_ddq','Comments','Version: 1.1', 'Vars', {lin_sym_vars})

A_con = [zeros(3) eye(3); double(equationsToMatrix(ddq_lin, [q;dq]))];

B_con = [zeros(3,2); double(equationsToMatrix(ddq_lin, Y(1:2)))];

%% Quick test of linearization
q_test = [0,0,0.3,0,0,0]';
u_test = [0,0]';

compute_ddq([q_test; u_test])
compute_linearized_ddq([q_test; u_test])
A_con*q_test + B_con*u_test

%% LQR

Q = diag([0.01 0.01 10 0.1 0.1 1]);
R = 1;

K = lqr(A_con,B_con,Q,R);

%% ODE45
tau_min = -4; tau_max = 4;
limit_torque = @(tau) max(min(tau, tau_max),tau_min);

update_fn = @(t,x) [x(4:6); compute_ddq([x; limit_torque(-K*x)])];

x0 = [0 0 0 0 0 1.5]';
tspan = [0 5];
[tout,xout] = ode45(update_fn, tspan, x0);

figure(1)
hold on
titles = {'phiL','phiR','theta','dphiL','dphiR','dtheta','tauL','tauR'};
for i = 1:8
    subplot(4,2,i)
    if (i < 7)
        plot(tout,xout(:,i))
    else
        u = limit_torque(-K*xout(:,:)');
        plot(tout,u(i-6,:)')
    end
    title(titles{i})
end



