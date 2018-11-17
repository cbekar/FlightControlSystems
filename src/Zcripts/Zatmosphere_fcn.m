function [Mach_number, qbar] = Zatmosphere_fcn(Vt, alt)
[rho,a] = atmos(alt);
Mach_number = Vt/a;
qbar = 0.5*Vt*Vt*rho;
end

