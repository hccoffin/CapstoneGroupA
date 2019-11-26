% Parameters
tstep = 0.01;

% Problem specific
t0 = 0;
tf = 1;
t = t0:tstep:tf;
N = length(t); % Number of timesteps

% Initial condition constraint
q0 = [0;0;0];
dq0 = [0;0;0];

% Final condition constraint
qf = [0;2*pi;2*pi];
dqf = [0;0;0];

% Zero initial values
x0 = zeros(8*N, 1);

% Setup initial and final condition constraints
Aeq1 = zeros(12,8*N);

Aeq1(1,1) = 1; % Initial q1
Aeq1(2,N) = 1; % Final q1
Aeq1(3,N+1) = 1; % Initial q2
Aeq1(4,2*N) = 1; % Final q2
Aeq1(5,2*N+1) = 1; % Initial q3
Aeq1(6,3*N) = 1; % Final q3

Aeq1(7,3*N+1) = 1; % Initial dq1
Aeq1(8,4*N) = 1; % Final dq1
Aeq1(9,4*N+1) = 1; % Initial dq2
Aeq1(10,5*N) = 1; % Final dq2
Aeq1(11,5*N+1) = 1; % Initial dq2
Aeq1(12,6*N) = 1; % Final dq2

Beq1 = [q0(1);qf(1);q0(2);qf(2);q0(2);qf(3);dq0(1);dqf(1);dq0(2);dqf(2);dq0(3);dqf(3)];

%% Setup nonlinear problem
problem.objective = @(x) objective(x,tstep,N);
problem.x0 = x0;
problem.Aeq = Aeq1;
problem.beq = Beq1;
problem.nonlcon = @(x) nonlinear_constraint(x,tstep,N);
problem.solver = 'fmincon';
problem.options = optimoptions('fmincon','Display', 'iter', 'MaxFunctionEvaluations', 1e7, 'MaxIterations', 1e5);

%% Call fmincon for nonlinear dynamics
tic
%[X_nonlinear_out,final_cost_nonlinear] = fmincon(problem);
toc

%% Setup linear problem
problem.x0 = x0;
%% Call fmincon for linear dynamics
tic
[X_linear_out,final_cost_linear] = fmincon(problem);
toc

figure(4)
subplot(1,2,1)
plot(t,X_linear_out(1:N))

subplot(1,2,2)
hold on
plot(t,X_linear_out(N+1:2*N))
plot(t,X_linear_out(2*N+1:3*N))
