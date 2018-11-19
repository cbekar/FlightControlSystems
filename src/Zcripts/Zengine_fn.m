function thrust = Zengine_fn(dt,alt,Mach_number)
global engineLUT engineCount Tmil
thrust = interpn(0:0.2:1,-10000:10000:50000,0:1,engineLUT,Mach_number,alt,dt)*engineCount*Tmil;
end