function dy = lqr_state_update_fn(t,y,y_des,K)
%	state_update_function Returns the derivative of state y
%   Inputs:
%       t: current time
%       y: current state (theta, phiL, phiR, dtheta, dphiL, dphiR)
%       y_des: desired_state
%       K: LQR gains matrix
%   Outputs
%       dy: current state derivative

y_error = y - y_des;
tau_desired = -K*y_error;
tau_actual = check_feasible_torque(tau_desired, y_error(4:6));

% Motor modeling (not used when designing LQR controller) 
theta = y(1);
dtheta = y(4);
ddq = compute_ddq(theta, dtheta, tau_actual(1), tau_actual(2));
dq = y(4:6);

dy = [dq;ddq];
end
