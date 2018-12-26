function cost = Zcostfn(SS)
global S
S.states.alpha  = SS(1);
S.states.beta   = SS(2);
a = cos(S.states.alpha)*cos(S.states.beta);
b = sin(S.states.phi)*sin(S.states.beta)+cos(S.states.phi)*sin(S.states.alpha)*cos(S.states.beta);
S.states.theta  = atan((a*b+sin(S.states.gamma)*sqrt(a^2-sin(S.states.alpha)^2+b^2))/(a^2-sin(S.states.gamma)^2));
G = S.states.v^2/(S.rad*S.gd);
a = 1 - G*tan(S.states.alpha)*sin(S.states.beta);
b = sin(S.states.gamma)/cos(S.states.beta);
c = 1 + G^2*cos(S.states.beta)^2;
nom = (a-b^2)+b*tan(S.states.alpha)*sqrt(c*(1-b^2)+G^2*sin(S.states.beta)^2);
den = (a^2-b^2*(1+c*tan(S.states.alpha)^2));
S.states.phi    = atan(G*(cos(S.states.beta)/cos(S.states.alpha))*(nom/den));
S.controls.t    = SS(3);
S.controls.e    = SS(4);
S.controls.a    = SS(5);
S.controls.r    = SS(6);
%%
stateDot = ZgetStateDerivativesNonlinearSystem();

w = [1; 
     1000; 
     100;
     0;
     0; 
     0;
     10;
     10;
     10;
     0;
     0;
     0 
     ];
 
cost = w'*(stateDot.^2);
if ~isreal(cost)
    cost = 10;
end
end