%% Trim
global S Tmil engineCount engineLUT;
Zinit('737');
S = struct('states',...
               struct('v',0,'alpha',0,'beta',0,'gamma',0,...
               'phi',0,'theta',0,'psi',0,...
               'p',0,'q',0,'r',0,...
               'n',0,'e',0,'h',0),...
          'controls',...
               struct('t',0,'e',0,'a',0,'r',0));
[S.states.v,~,f1] = trimloop(700);
simtime = 300;
sampleTime = 0.01;
numberOfSamples = simtime * 1/sampleTime +1;
timeVector = (0:numberOfSamples) * sampleTime;

usignal(:,1) = timeVector';
usignal(:,2) = f1(3)*ones(1,size(timeVector,2))';
usignal(:,3) = f1(4)*ones(1,size(timeVector,2))';
usignal(:,4) = f1(5)*ones(1,size(timeVector,2))';
usignal(:,5) = f1(6)*ones(1,size(timeVector,2))';
xu = [S.states.v,f1(1),f1(2),f1(7),f1(1)+S.states.gamma,0,0,0,0,0,0,S.states.h]; 
uu = [f1(3),f1(4),f1(5),f1(6)]; 
sim('B733_JSB');
%% Linearization
[A,B,C,D] = linmod('B733_JSB',xu,uu);
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
kq = 1;
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
%%
function [optspeed,cost,trim] = trimloop(speeds)
%% trimloop returns the optimal speed (wrt linearization cost) for a chosen altitude
global S
for i = 1:length(speeds)
    i
    S.states.v  = speeds(i);
    S.states.h  = 30000;
    %S.rad  = 100000;
    S.rad  = Inf;
    S.states.gamma = deg2rad(0);
    S.gd   = Zgravity_fn(S.states.n,S.states.e,S.states.h)*3.28084;
    Sarr = zeros(6,1)';

    %options =  optimset('TolFun',1e-25,'TolX',1e-25,'MaxFunEvals',15e+9,...
    %    'MaxIter',15e+9,'FunValCheck','on','PlotFcns',@optimplotfval);
    options =  optimset('TolFun',1e-25,'TolX',1e-25,'MaxFunEvals',15e+9,...
        'MaxIter',15e+9,'FunValCheck','on');
    [f1(i,1:6), costs(i),~,out] = ...
        fmincon('Zcostfn',Sarr,[],[],[],[],...
        [-pi/2 -pi/2 0 -0.3 -0.35 -0.35],[pi/2 pi/2 1 0.3 0.35 0.35],...
        [],options);
    f1(i,7) = S.states.phi;
end
[cost,ind] = min(costs);
sprintf('Optimum speed is %d fps \nCost function is %.2d, \nTrim state is: \nalpha: %.2d, \nbeta: %.2d, \nphi: %.2d; \nControl Inputs are:\nthrottle: %.2d, \nelevator: %.2d, \nleft_aileron: %.2d, \nrudder: %.2d', speeds(ind), costs(ind), f1(ind,1)*180/pi,f1(ind,2)*180/pi,f1(ind,7)*180/pi,f1(ind,3),f1(ind,4),f1(ind,5),f1(ind,6))
optspeed = speeds(ind);
trim = f1(ind,:);
end