function dy = state_update_fn(t,y,y_des,K)
%	state_update_function Returns the derivative of state y
%   Inputs:
%       t: current time
%       y: current state (theta, phiL, phiR, dtheta, dphiL, dphiR)
%   Outputs
%       dy: current state derivative

theta = y(1);
dtheta = y(4);

y_error = y - y_des;
tau_desired = -K*y_error;
tau_actual = check_feasible_torque(tau_desired, y_error(4:6));

% Motor modeling (not used when designing LQR controller) 

ddq = compute_ddq(theta, dtheta, tau_actual(2), tau_actual(3));
dq = y(4:6);

dy = [dq;ddq];
end
