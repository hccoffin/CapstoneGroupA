function M_bar = compute_M_bar(Ib,Iw,l,mb,mw,r,theta)
%COMPUTE_M_BAR
%    M_BAR = COMPUTE_M_BAR(IB,IW,L,MB,MW,R,THETA)

%    This function was generated by the Symbolic Math Toolbox version 8.3.
%    29-Nov-2019 17:21:48

%Version: 1.1
t2 = cos(theta);
t3 = r.^2;
t4 = (mb.*t3)./4.0;
t5 = (l.*mb.*r.*t2)./2.0;
t6 = Iw+t5;
M_bar = reshape([Ib+Iw.*2.0+l.^2.*mb,t6,t6,t6,Iw+t4+mw.*t3.*2.0,t4,t6,t4,Iw+t4],[3,3]);
