%% Numerical constants
dt = 0.01; % Update rate (s)

mw = 0.5313; % Mass of wheels (kg)
Iw = 0.00273; % Inertia of wheels (kg*m^2)
m = 8.22; % Mass of body (kg)
I = 0.57; % Inertia of body (kg*m^2)

R = 0.101; % Wheel radius (m)
L = 0.49; % Length from wheel axle to body COM (m)

g = 9.81; % Gravitational Constant (m/s^2)

By = 0.1;
Bm = 0.1;

E = [Iw + (mw + m)*R^2, m*R*L;
     m*R*L, I+m*L^2];
 
F = [By + Bm, -Bm;
     -Bm, Bm];

G = [0; -m*g*L];

H = [1;-1];

A = [zeros(2,2), eye(2,2);
     zeros(2,1), -inv(E)*G, -inv(E)*F];

B = [zeros(2,1);
     -inv(E)*H];


[Ad, Bd] = c2d(A, B, dt);
 
Q = diag([0.01 10 0.1 1]);
R = 3;

[K,S,E] = lqr(A,B,Q,R);
