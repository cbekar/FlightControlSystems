%% Trim
global S Tmil engineCount engineLUT;
Zinit('737');
optimalTrimIndex = trim('737PDT','MMCR');
S.simtime = 300;
timeVector = (0:S.simtime * 1/0.01 +1) * 0.01;
S.usignal(:,1) = timeVector';
S.usignal(:,2:5) = S.trim(optimalTrimIndex,3:6).*ones(1,size(timeVector,2))';
S.xu = [S.trim(optimalTrimIndex,8),S.trim(optimalTrimIndex,1),S.trim(optimalTrimIndex,2),...
    S.trim(optimalTrimIndex,7),S.trim(optimalTrimIndex,1)+S.states.gamma,0,0,0,0,0,0,S.trim(optimalTrimIndex,9)]; 
S.uu = [S.trim(optimalTrimIndex,3),S.trim(optimalTrimIndex,4),S.trim(optimalTrimIndex,5),S.trim(optimalTrimIndex,6)]; 
sim('B733_JSB');
%% Linearization
B733_linear_sys = linmod('B733_JSB',S.xu,S.uu); close all;
A = B733_linear_sys.a; B = B733_linear_sys.b;
% Swap states and inputs for the ease of generating A B matrices.
Along = [A(1,1) A(1,2) A(1,5) A(1,8);
        A(2,1) A(2,2) A(2,5) A(2,8);
        A(5,1) A(5,2) A(5,5) A(5,8);
        A(8,1) A(8,2) A(8,5) A(8,8)
       ];
Alat  = [A(3,3) A(3,4) A(3,7) A(3,9);
        A(4,3) A(4,4) A(4,7) A(4,9);
        A(7,3) A(7,4) A(7,7) A(7,9);
        A(9,3) A(9,4) A(9,7) A(9,9)
       ];
Blong = [B(1,1:2); B(2,1:2); B(5,1:2); B(8,1:2)];
Blat  = [B(3,3:4); B(4,3:4); B(7,3:4); B(9,3:4)];
C  = eye(2,4);
D  = zeros(2);
%% Controller designs
% 1) Find SAS gains
% 1-A) Longitudinal
% 1-A-i) Pitch Damper
%getGain(Along, Blong);
Bpd = Blong(:,2);
Apd_aug = [Along, -Bpd, zeros(4,1); [0 0 0 0 -10 0]; [0 10 0 0 0 -10]];
Bpd_aug = [zeros(4,1); 10; 0];
Cpd_aug = [C zeros(2); zeros(1,5) 1];
k= logspace(-2,1,2000);
% Here we can observe performance degredation for alpha feedback
rlocus(Apd_aug,Bpd_aug,Cpd_aug(3,:),0,k); 
grid on
axis([-15,1,-10,10])

Apd_aug = [Along, -Bpd, zeros(4,1); [0 0 0 0 -10 0]; [0 0 0 10 0 -10]];
Bpd_aug = [zeros(4,1); 10; 0];
Cpd_aug = [C zeros(2); zeros(1,5) 1];
figure
rlocus(Apd_aug,Bpd_aug,Cpd_aug(3,:),0,k); 
grid on
axis([-15,1,-10,10])
% Here we can observe flying quality for 1 pitch rate feedback gives good
% longitudinal frequencies and damping ratios (according to Nelson pg. 167)
% [pitchSASnum,pitchSASden] = ss2tf(Along,Blong,C,D,2);
% alpElv = tf(pitchSASnum(1,:),pitchSASden);
% rlocus(alpElv)
% qElv   = tf(pitchSASnum(2,:),pitchSASden);
% rlocus(qElv)
%%
kq = 1.5;
Apd_cl = Apd_aug - Bpd_aug.*kq*Cpd_aug(3,:);
figure
rlocus(Apd_cl,Bpd_aug,Cpd_aug(3,:),0,k);
grid on
axis([-15,1,-10,10])

t= [0:.02:10]; % 501 points for plot
%u= [zeros(1,50),1.8*ones(1,451)]'; %step
u= [-1.8*ones(1,51),1.8*ones(1,50),zeros(1,400)]'; % Doublet
[y,x]= lsim(Apd_cl,Bpd_aug(:,1),Cpd_aug(3,:),0,u,t); % Linear simulation
plot(t,y,t,u)
grid on
hold on
[y,x]= lsim(Apd_aug,Bpd_aug(:,1),Cpd_aug(3,:),0,u,t); % Linear simulation
plot(t,y)