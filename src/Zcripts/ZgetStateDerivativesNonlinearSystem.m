function [stateDot] = ZgetStateDerivativesNonlinearSystem()
global S
g = Zgravity_fn(S.states.n, S.states.e, S.states.h);
[Mach_number, qbar] = Zatmosphere_fn(S.states.v, S.states.h);
thrust = Zengine_fn(S.controls.t,S.states.h,Mach_number);
[forces,moments] = Zaero_fn(S.states.v,Mach_number,S.controls.e,S.controls.a,...
    S.controls.r,S.states.alpha,S.states.beta,0,qbar,S.states.p,S.states.q,S.states.r);
stateDot = Zeom_fn(forces,moments,g,thrust,[S.states.v,S.states.alpha,S.states.beta,...
    S.states.phi,S.states.theta,S.states.psi,S.states.p,S.states.q,S.states.r]);
end