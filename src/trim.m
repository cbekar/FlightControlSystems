function [ind, opttrim, optspeed] = trim(pdt, type)
%% trimloop returns the optimal speed (wrt linearization cost) for a chosen altitude
global S
txt = dlmread(strcat(pdt,'.txt'));
index = floor(strfind(('LMCLMMCLHMCLLMDEMMDEHMDELMCRMMCRHMCR'),type)/4)*26;
progressbar
for i = 26:-1:1
    v0  = txt(index+i,6)*1.68781;  S.states.v = v0;
    h0  = txt(index+i,1)*100;  S.states.h = h0;
    %S.rad  = 100000;
    S.rad  = Inf;
    S.states.gamma = deg2rad(txt(index+i,15));
    %S.weight = txt(index+i,9)*2.2;
    S.gd   = Zgravity_fn(S.states.n,S.states.e,S.states.h)*3.28084;
    Sarr = zeros(6,1)';
    S.M(i) = txt(index+i,8);
    %options =  optimset('TolFun',1e-25,'TolX',1e-25,'MaxFunEvals',15e+9,...
    %    'MaxIter',15e+9,'FunValCheck','on','PlotFcns',@optimplotfval);
    options =  optimset('TolFun',1e-25,'TolX',1e-25,'MaxFunEvals',15e+9,...
        'MaxIter',15e+9,'FunValCheck','on','Display','off');
    [f1(i,1:6), costs(i),~,out] = ...
        fmincon('Zcostfn',Sarr,[],[],[],[],...
        [-pi/2 -pi/2 0 -0.3 -0.35 -0.35],[pi/2 pi/2 1 0.3 0.35 0.35],...
        [],options);
    progressbar((27-i)/26);
    S.trim(i,1:6) = f1(i,1:6);
    S.trim(i,7) = S.states.phi;
    S.trim(i,8:9) = [v0,h0];
end
[~,ind] = min(costs);
%sprintf('Optimum speed is %d fps \nCost function is %.2d, \nTrim state is: \nalpha: %.2d, \nbeta: %.2d, \nphi: %.2d; \nControl Inputs are:\nthrottle: %.2d, \nelevator: %.2d, \nleft_aileron: %.2d, \nrudder: %.2d',...
%    floor(txt(index+ind,6)*1.68781), costs(ind), S.trim(ind,1)*180/pi,...
%    S.trim(ind,2)*180/pi,S.trim(ind,7)*180/pi,S.trim(ind,3),...
%    S.trim(ind,4),S.trim(ind,5),S.trim(ind,6))
optspeed = txt(index+ind,6)*1.68781;
opttrim =  S.trim(ind,:);
fixLUT_breakpoints();
end
function fixLUT_breakpoints()
    global S;
    for i = 1:length(S.M)-1
        if S.M(i+1) <= S.M(i)
            S.M(i+1) = S.M(i+1) + 0.0001*i;
        end
    end
end

