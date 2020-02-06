function [c,ceq] = nonlinear_constraint(x,tstep,N)
%	objective Computes the constraint value for each constraint
%   x: column vector of decision variables
%   tstep: elapsed time
%   N: number of states
%   c: 4*N by 1 vector of dynamic constraint values, q then dq
q1 = x(1:N)';
q2 = x(N+1:2*N)';
q3 = x(2*N+1:3*N)';
dq1 = x(3*N+1:4*N)';
dq2 = x(4*N+1:5*N)';
dq3 = x(5*N+1:6*N)';
tau1 = x(6*N+1:7*N)';
tau2 = x(7*N+1:8*N)';

q = [q1; q2; q3];
dq = [dq1; dq2; dq3];

ddq_n = compute_ddq(q(1,1),dq(1,1),tau1(1),tau2(1));

ceq = zeros(N,6);
for i = 1:N-1
    ddq_n_plus = compute_ddq(q(1,i+1),dq(1,i+1),tau1(i+1),tau2(i+1));
    ceq(i,1:3) = dq(:,i) - dq(:,i+1) + tstep/2 * (ddq_n_plus + ddq_n);
    ceq(i,4:6) = q(:,i) - q(:,i+1) + tstep/2 * (dq(:,i+1) + dq(:,i));          
    ddq_n = ddq_n_plus;
end
c = [];
end