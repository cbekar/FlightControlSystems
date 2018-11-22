%% Trim
global S Tmil engineCount engineLUT;
Zinit('737');
S = struct('states',...
               struct('v',0,'alpha',0,'beta',0,'gamma',0,...
               'phi',0,'theta',0,'ksi',0,...
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
A
B
eig(A)

function [optspeed,cost,trim] = trimloop(speeds)
%% trimloop returns the optimal speed (wrt linearization cost) for a chosen altitude
global S
for i = 1:length(speeds)
    i
    S.states.v  = speeds(i);
    S.states.h  = 20000;
    %S.rad  = 40000;
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