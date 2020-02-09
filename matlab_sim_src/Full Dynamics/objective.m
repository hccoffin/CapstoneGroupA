function value = objective(x,tstep,N)
%	objective Computes the objective value of the function at the point x
%   x: column vector of decision variables
%   tstep: elapsed time
%   N: number of states
%   value: function that we're are minimizing evaluated at f
	value = 0;
    tau1 = x(6*N+1:7*N);
    tau2 = x(7*N+1:8*N);
    tau = [tau1'; tau2']; % 2 x N torques
    for i = 1:N-1
       value = value + tstep/2*(tau(:,i)'*tau(:,i) + tau(:,i+1)'*tau(:,i+1));
    end
  
end

