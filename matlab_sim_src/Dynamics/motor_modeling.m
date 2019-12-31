syms tau dq

eq = (0.35 + 6.65/250*tau == 250 - dq);

tau_max = solve(eq, tau)
