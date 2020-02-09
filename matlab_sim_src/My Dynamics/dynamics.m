R = 0.10; % Wheel radius
L = 0.40; % COM distance from axle
mw = 0.5; % Mass of one wheel (kg)
mb = 6; % Mass of body (kg)
Iw = 0.02; % Wheel Inertia about axis (one wheel) (kg*m^2)
Ib = 0.4; % Body Inertia about COM (kg*m^2)
By = 0.2; % Rolling damping ratio (floor)
Bm = 0.02; % Friction damping ratio (joint)

params_n = [R L mw mb Iw Ib By Bm];
syms R L mw mb Iw Ib By Bm real
params = [R L mw mb Iw Ib By Bm];


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

Y_dissipated = [By+Bm 0     -Bm;
                0     By+Bm -Bm;
                -Bm   -Bm    Bm]*dq;

ddq = inv(M_bar)*(Y-C_bar*dq-N_bar-Y_dissipated);

%% Compute Linear Update Model
lin_sym_vars = [q;dq;Y(1:2)];
lin_num_vars = [0;0;0;0;0;0;0;0];

J_ddq = jacobian(ddq, lin_sym_vars);
J_ddq_n = subs(J_ddq, lin_sym_vars, lin_num_vars);

ddq_lin = simplify(subs(ddq, lin_sym_vars, lin_num_vars) + J_ddq_n * (lin_sym_vars - lin_num_vars),5);

A_con = simplify([zeros(3) eye(3); equationsToMatrix(ddq_lin, [q;dq])]);
B_con = simplify([zeros(3,2); equationsToMatrix(ddq_lin, Y(1:2))]);

ddq_n = subs(ddq, params, params_n);
ddq_lin_n = subs(ddq_lin, params, params_n);

matlabFunction(ddq_lin_n,'File','compute_linearized_ddq','Comments','Version: 1.0', 'Vars', {lin_sym_vars})
matlabFunction(ddq_n,'File','compute_ddq','Comments','Version: 1.1', 'Vars', {lin_sym_vars})
matlabFunction(A_con,'File','compute_A','Comments','Version: 1.0', 'Vars', params)
matlabFunction(B_con,'File','compute_B','Comments','Version: 1.0', 'Vars', params)

params_arg = mat2cell(params_n,1,ones(1,numel(params_n)));
A = compute_A(params_arg{:});
B = compute_B(params_arg{:});


%% Quick test of linearization
q_test = [0,0,0,0,0,0]';
u_test = [0,0]';

compute_ddq([q_test; u_test]);
compute_linearized_ddq([q_test; u_test]);
A*q_test + B*u_test;

%% LQR

Q = diag([0.01 0.01 10 0.1 0.1 1]);
R = 1;

K = lqr(A,B,Q,R);

%% ODE45
tau_min = -3; tau_max = 3;
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



