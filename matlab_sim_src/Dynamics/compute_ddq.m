function ddq_n = compute_ddq(theta,dtheta,tauL,tauR)
%COMPUTE_DDQ
%    DDQ_N = COMPUTE_DDQ(THETA,DTHETA,TAUL,TAUR)

%    This function was generated by the Symbolic Math Toolbox version 8.3.
%    03-Dec-2019 18:18:08

%Version: 1.1
t2 = cos(theta);
t3 = sin(theta);
t4 = dtheta.^2;
t5 = theta.*2.0;
t6 = t2.^2;
t7 = sin(t5);
t8 = t2.*6.8e+2;
t10 = t2.*tauL.*6.4e+5;
t11 = t2.*tauR.*6.4e+5;
t9 = t6.*8.5e+2;
t12 = t6.*tauL.*8.0e+5;
t13 = t6.*tauR.*8.0e+5;
t14 = t8+t9-4.673e+3;
t15 = 1.0./t14;
ddq_n = [t15.*(t3.*-2.24649e+5+tauL.*1.28e+5+tauR.*1.44e+5+t3.*t4.*3.4e+2+t2.*tauL.*3.2e+5+t2.*tauR.*3.6e+5+t2.*t3.*t4.*8.5e+2);t15.*(t3.*1.25568e+5+t7.*1.5696e+5+t10-t11+t12-t13-tauL.*6.928e+6+tauR.*1.552e+6-t3.*t4.*6.72e+3);t15.*(t3.*1.41264e+5+t7.*1.7658e+5-t10+t11-t12+t13+tauL.*1.552e+6-tauR.*7.6e+6-t3.*t4.*7.56e+3)];
