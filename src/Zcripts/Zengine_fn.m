function thrust = Zengine_fn(dt,alt,Mach_number)
global S
thrust = interpn(0:0.2:1,-10000:10000:50000,0:1,S.engineLUT,Mach_number,alt,dt)*S.engineCount*S.Tmil;
end