function dy = state_update_fn(t,y,K)
%	state_update_function Returns the derivative of state y
%   Inputs:
%       t: current time
%       y: current state (theta, phiL, phiR, dtheta, dphiL, dphiR)
%   Outputs
%       dy: current state derivative

theta = y(1);
dtheta = y(4);

%tau = 0.4*theta + 0.1*dtheta;
%tauL = tau;
%tauR = tau;
tau = -K*y;
tauL = tau(1);
tauR = tau(2);

ddq = compute_ddq(theta, dtheta, tauL, tauR);
dq = y(4:6);

dy = [dq;ddq];
end

