function [forces,moments] =   Zaero_fn(V0, M, def_elevator, def_aileron, def_rudder, alpha, beta, alpha_dot, dyn_pressure, P, Q, R) 
%Parameters
S = 1171; %ft^2 area of cannard
b = 94.70; %ft wingspan
c = 12.31; %ft mean chord

table = [-1.5700    1.5000
   -0.2600    0.0420
         0    0.0210
    0.2600    0.0420
    1.5700    1.5000];
C_D_0 = fixpt_interp1(table(:,1),table(:,2),alpha,'double',1,'double',1,'Nearest');

table = [0         0
    0.7900         0
    1.1000    0.0230
    1.8000    0.0150];
C_D_mach = fixpt_interp1(table(:,1),table(:,2),M,'double',1,'double',1,'Nearest');

table = [-1.5700    1.2300
   -0.2600    0.0500
         0         0
    0.2600    0.0500
    1.5700    1.2300];
C_D_beta = fixpt_interp1(table(:,1),table(:,2),beta,'double',1,'double',1,'Nearest');

C_D_d_ele = 0.0590;

C_Y_beta = 1; %this was reported as -1, however sideslip derivatives turns out that this should be +1

table = [
    -0.2000   -0.6800
    0         0.2000
    0.2300    1.2000
    0.4600    0.2000];

C_L_0 = fixpt_interp1(table(:,1),table(:,2),alpha,'double',1,'double',1,'Nearest');

C_L_d_ele = 0.2;
C_L_tot = C_L_0+C_L_d_ele*def_elevator;

C_Di = C_L_tot*C_L_tot*0.043;
C_D_0 = C_D_0 + C_Di + C_D_mach + C_D_beta;

C_l_beta = -0.0900;
table = [         0    0.1000
    2.0000    0.0330];
C_l_d_ail = fixpt_interp1(table(:,1),table(:,2),M,'double',1,'double',1,'Nearest'); 

C_l_d_rud = 0.0100;
C_l_p = -0.40;
C_l_r = 0.09;

C_m_alp = -0.6;
table = [0         -1.2
    2.0000    -0.3];
C_m_d_ele = fixpt_interp1(table(:,1),table(:,2),M,'double',1,'double',1,'Nearest'); 

C_m_alpdot = -16;
C_m_q = -27;

C_n_beta = 0.2600;
C_n_d_rud = -0.2000;
C_n_r = -0.3500;

D = dyn_pressure*S*(C_D_0+C_D_d_ele*def_elevator/0.3);
Y = dyn_pressure*S*(C_Y_beta*beta);
l = dyn_pressure*S*(C_L_tot);

L = dyn_pressure*S*b*(C_l_beta*beta+C_l_d_ail*def_aileron+C_l_d_rud*def_rudder+C_l_p*(P*b/(2*V0))+C_l_r*(R*b/(2*V0)));
M = dyn_pressure*S*c*(C_m_alp*alpha+C_m_d_ele*def_elevator+C_m_alpdot*(alpha_dot*c/(2*V0))+C_m_q*(Q*c/(2*V0)));
N = dyn_pressure*S*b*(C_n_beta*beta+C_n_d_rud*def_rudder+C_n_r*(R*b/(2*V0)));

ca = cos(alpha); cb = cos(beta);
sa = sin(alpha); sb = sin(beta);
dcm = [ca*cb, sb, sa*cb; -ca*sb, cb, -sa*sb; -sa 0 ca];

forces = (-1.*dcm\[D,Y,l]')';
moments = (dcm\[L,M,N]')';
end