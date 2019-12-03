clear all
load('Dynamics/dynamics.mat')
dt = Ts_n;
[nx, nu] = size(Bd); % Length of State Vector, Action Vector
N = 500; % Num States to Look Ahead

xr = zeros(nx,1); % desired state
x0 = [1;0;0;0;0;0]; % current state

% Setup objective (1/2*x'*P*x + q'x)
Q = eye(nx, nx);
QN = 3*eye(nx, nx);
R = eye(nu,nu);

% Save to YAML
control_params = struct('Ad', Ad, 'Bd', Bd, 'Q', Q, 'R', R, 'N', N, 'dt', dt);
yaml.WriteYaml('../ros_ws/src/controller/config/control_params.yaml',x)

P = blkdiag(kron(speye(N), Q), QN, kron(speye(N), R) );
q = [repmat(-Q*xr, N, 1); -QN*xr; zeros(N*nu, 1)];

% Setup equality constraint (dynamics)
Adx = [kron(eye(N), Ad), zeros(N*nx,nx)];
x_plus = [zeros(N*nx,nx), kron(eye(N), eye(nx))];
Bdu = kron(eye(N), Bd);

Aeq = [Adx-x_plus, Bdu];

leq = zeros(N*nx, 1);
ueq = zeros(N*nx, 1);

% Setup inequality constraints (initial conditions, bounds)
xmin = [-inf;-inf;-inf;-inf;-inf;-inf];
xmax = [ inf; inf; inf; inf; inf; inf];
umin = [-2;-2];
umax = [2;2];

Aineq = eye(nx + N*nx + N*nu ); % N+1 constraints on x, N constraint on u
lineq = [x0; repmat(xmin, N, 1); repmat(umin, N, 1)];
uineq = [x0; repmat(xmax, N, 1); repmat(umax, N, 1)];

A = [Aeq;Aineq];
l = [leq;lineq];
u = [ueq;uineq];

% Create an OSQP object
prob = osqp;

% Setup workspace
prob.setup(P, q, A, l, u, 'warm_start', true);

res = prob.solve();

theta = res.x(1:6:N*nx);
phil = res.x(2:6:N*nx);
phir = res.x(3:6:N*nx);

figure(1)

subplot(1,3,1)
plot(theta)
title("Theta")

subplot(1,3,2)
plot(phil)
title("Phi Left")

subplot(1,3,3)
plot(phir)
title("Phi Right")
