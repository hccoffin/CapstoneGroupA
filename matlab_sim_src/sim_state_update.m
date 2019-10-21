function dy = sim_state_update(t,y,g)

torque = 20*y(3) + 0.80 * y(4); % Arbitrary proportional controller

ddq = g(y(3), y(4), torque); %phi, dphi, torque

dy = [y(2);
      ddq(1);
      y(4);
      ddq(2)];

end

