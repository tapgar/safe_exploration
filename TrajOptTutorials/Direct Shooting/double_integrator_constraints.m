function [ c, ceq ] = double_integrator_constraints( x )

c = []; %no nonlinear inequatlity constraint needed

sim_time = x(1);
delta_time = sim_time / (length(x) - 1);
times = 0: delta_time : sim_time - delta_time;

accs = x(2:end);

vels = cumtrapz(times, accs);
pos = trapz(times, vels);

%end pos equals 1 and velocity should equal 0
ceq = [pos - 1; vels(end)];

end

