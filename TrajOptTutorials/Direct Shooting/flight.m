
% the function to be minimized E = vx^2 + vy^2
energy_fun = @(x) (x(2)^2 + x(3)^2);

% initial parameter guess; time = 1; vx = 2, vy = 3
x0 = [1; 2; 3];

% no linear inequality or equality constraints
A = [];
b = [];
Aeq = [];
Beq = [];

% Lower bound the simulation time at zero, leave initial velocities
% unbounded
lb = [0; -Inf; -Inf];
ub = [Inf; Inf; Inf];

% Solve for the best simulation time + control input
optimal = fmincon(energy_fun, x0, A, b, Aeq, Beq, lb, ub, @flight_constraint);

opt_time = optimal(1);
opt_vx0 = optimal(2);
opt_vy0 = optimal(3);

plot(opt_time, opt_vx0)
hold on
plot(opt_time, opt_vy0)