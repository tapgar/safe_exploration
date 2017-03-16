function [ xdd ] = researchSim_accel( x, u, d)

c = 0.1;
m = 1;
f = 0.2;

xdd = (u - c*x(2) - f - d)/m;

end

