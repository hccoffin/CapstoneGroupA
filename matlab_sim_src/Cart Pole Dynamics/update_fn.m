function dy = update_fn(t,y,ddq_fn,K)

F_range = [-20 20];
F = min(F_range(2), max(-K*y, F_range(1)));

ddq = ddq_fn(y(1), y(2), y(3), y(4), F);

dy = [y(3), y(4), ddq(1), ddq(2)]';
end

