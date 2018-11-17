function xdot = Zeom_fcn(forces,moments, g, thrust,x)

xdot = zeros(12,1);

%TODO fuel to be added
weight = 83000; %Weight of Aircraft (lb)
gd     = g*3.28084; %Gravitational Acceleration
m      = weight/gd; %Mass(slug)

Heng  = 0;

Jxx = 562000;
Jyy = 1.473e+06;
Jzz = 1.894e+06;
Jxz = 8000;

Vt    = x(1);
alpha = x(2);
beta  = x(3);
phi   = x(4);
theta = x(5);
ksi   = x(6);
P     = x(7);
Q     = x(8);
R     = x(9);

T   = thrust;

Fx = forces(1); %CD
Fy = forces(2); %CL
Fz = forces(3); %CC

L = moments(1); %Cl
M = moments(2); %Cm
N = moments(3); %Cn

cp = cos(phi);
ct = cos(theta);
ck = cos(ksi);
sp = sin(phi);
st = sin(theta);
sk = sin(ksi);
tt = tan(theta);

%**************FORCE EQUATIONS**************%  

U = Vt * cos(alpha)*cos(beta);
V = Vt * sin(beta);
W = Vt * sin(alpha)*cos(beta);

UDOT  = R*V  - Q*W - gd*st + (Fx + T)/m; 
VDOT  = P*W  - R*U + gd*sp*ct + Fy/m;
WDOT  = Q*U  - P*V + gd*cp*ct + Fz/m;

xdot(1) = (U*UDOT + V*VDOT + W*WDOT)/Vt;     %Vtdot
xdot(2) = (U*WDOT - W*UDOT)/(U*U + W*W);     %alphadot
xdot(3) = (Vt*VDOT- V*xdot(1))/(U*U + W*W);  %betadot

%**************KINEMATIC EQUATIONS**************%

xdot(4) = P + tt*(Q*sp + R*cp); %phidot
xdot(5) = Q*cp - R*sp;          %thetadot
xdot(6) = Q*sp/ct + R*cp/ct;    %ksidot

%**************MOMENT EQUATIONS**************%

xdot(7) = (Jxz*(Jxx-Jyy+Jzz)*P*Q - (Jzz*(Jzz-Jyy)+Jxz^2)*Q*R + Jzz*L + Jxz*(N + Q*Heng))/(Jxx*Jzz-Jxz^2);  %pdot
xdot(8) = ((Jzz-Jxx)*P*R - Jxz*(P^2 - R^2) + M - R*Heng )/Jyy; %qdot
xdot(9) = (((Jxx-Jyy)*Jxx+Jxz^2)*P*Q - Jxz*(Jxx-Jyy+Jzz)*Q*R + Jxz*L + Jxx*(N + Q*Heng))/(Jxx*Jzz-Jxz^2);  %rdot

%**************NAVIGATION EQUATIONS**************%

xdot(10) = U*ct*ck + V*(-cp*sk + sp*st*ck) + W*(sp*sk + cp*st*ck);  %xdot
xdot(11) = U*ct*sk + V*(cp*ck + sp*st*sk) + W*(-sp*ck + cp*st*sk);  %ydot
xdot(12) = U*st - V*sp*ct - W*cp*ct;                                %altdot