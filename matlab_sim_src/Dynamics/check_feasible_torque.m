function tau_applied = check_feasible_torque(tau_desired, dq)
%	torque_bounds_check
    stall_torque = 250 * 0.0070615518333333; % convert to NM
    nl_speed = 251*0.104719755; % convert to rads/s
    
    tau_applied = tau_desired;
end

