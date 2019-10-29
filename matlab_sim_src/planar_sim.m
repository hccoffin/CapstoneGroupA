close all

syms m_b i_b g phi dphi ddphi theta dtheta ddtheta l r torque real

m_b = 20;
g = 9.81;
l = 0.4;
i_b = m_b*l^2/12;
r = 0.06;


q = [theta; phi];
dq = [dtheta; dphi];

Jb = [r l*cos(phi); 0 -l*sin(phi); 0 1];

M_b = [m_b 0 0;
       0 m_b 0;
       0 0 i_b];
 
M = Jb'*M_b*Jb;
C = get_coriolis_matrix(M,q,dq);

V = m_b*g*l*cos(phi);
N = [diff(V,theta); diff(V,phi)];

Tau = [torque;0];

ddq = simplify(inv(M)*(Tau - C*dq - N));
%simplify(subs(ddq, [theta, dtheta, phi, dphi], [0, 0, 0, 0]))

g = matlabFunction(ddq, 'Vars', [phi dphi torque]);

tspan = [0 5];
q0 = [0 0 0.2 0]';
[t, y] = ode45(@(t,y) sim_state_update(t,y,g), tspan, q0);
visualize(t,y,r,l)


