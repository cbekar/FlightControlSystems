%% Trim
global S Tmil engineCount engineLUT;
Zinit('737');
S = struct('states',...
               struct('v',0,'alpha',0,'beta',0,'gamma',0,'phi',0,'theta',0,'ksi',0,...
                      'p',0,'q',0,'r',0,'n',0,'e',0,'h',0),...
          'controls',...
               struct('t',0,'e',0,'a',0,'r',0));
[S.states.v,~,f1] = trimloop(650:50:800);
xu = [S.states.v,f1(1),f1(2),f1(7),f1(1)+S.states.gamma,0,0,0,0,0,0,S.states.h]; 
uu = [f1(3),f1(4),f1(5),f1(6)]; 
sim('B733_JSB');
%% Linearization
[A,B,C,D] = linmod('B733_JSB')
eig(A)

function [optspeed,cost,trim] = trimloop(speeds)
global S
for i = 1:length(speeds)
    i
    S.states.v  = speeds(i);
    S.states.h  = 20000;
    S.rad  = 40000;
    %S.rad  = Inf;
    S.states.gamma = deg2rad(0);
    S.gd   = Zgravity_fcn(S.states.n,S.states.e,S.states.h)*3.28084;
    Sarr = zeros(6,1)';

    options =  optimset('Display','iter','TolFun',1e-15,'TolX', 1e-15, 'TolCon', 1e-15);
    [f1(i,1:6), costs(i),~,out] = fmincon('Zcostfn',Sarr,[],[],[],[],[-pi/2 -pi/2 0 -0.3 -0.35 -0.35],[pi/2 pi/2 1 0.3 0.35 0.35],[],options);
    f1(i,7) = S.states.phi;
end
[cost,ind] = min(costs);
sprintf('Optimum speed is %d fps \nCost function is %.2d, \nTrim state is: \nalpha: %.2d, \nbeta: %.2d, \nphi: %.2d; \nControl Inputs are:\nthrottle: %.2d, \nelevator: %.2d, \nleft_aileron: %.2d, \nrudder: %.2d', speeds(ind), costs(ind), f1(ind,1)*180/pi,f1(ind,2)*180/pi,f1(ind,7)*180/pi,f1(ind,3),f1(ind,4),f1(ind,5),f1(ind,6))
%sprintf('fval: %.2d @%.2d iterations, %d stepsize',val,output.iterations, output.stepsize)
optspeed = speeds(ind);
trim = f1(ind,:);
end