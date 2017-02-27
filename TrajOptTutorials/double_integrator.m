% close all
clear

global gridN
gridN = 20;

v_max = 10;
v_min = -10;
a_max = 4;
a_min = -4;



x_start = 0;
v_start = 0;
a_start = 0;
t_start = 0;
x_end = 10;
v_end = 0;
a_end = 0;

tic
while (x_end - x_start)/x_end > 0.01 %within 1%
i = 1;
% while i == 1
    i = i + 1;
%minimize the simulation time
time_min = @(x) x(1);

% The initial parameter guess; 1 second, gridN positions, gridN velocities,
% gridN accelerations
x0 = [1; linspace(x_start,x_end,gridN)'; linspace(0,1,gridN)'; ones(gridN, 1)];

% No linear inequality or equality constraints
A = [];
b = [];
Aeq = zeros(3, length(x0));
Aeq(1,2) = 1;
Aeq(2, gridN + 2) = 1;
Aeq(3, 2*gridN + 2) = 1;
% Aeq(4, gridN + 1) = 1;
% Aeq(5, 2*gridN + 1) = 1;
% Aeq(6, 3*gridN + 1) = 1;
Beq = [x_start, v_start, a_start]; % , x_end, v_end, a_end];
% Lower bound the simulation time at zero seconds, and bound the
% accelerations
lb = [0;    ones(gridN, 1) * x_start; ones(gridN, 1) * v_min;  ones(gridN, 1) * a_min];
ub = [Inf;   ones(gridN, 1) * x_end; ones(gridN, 1) * v_max;  ones(gridN, 1) * a_max];
% Options for fmincon
options = optimoptions(@fmincon, 'TolFun', 0.00000001, 'MaxIter', 10000, ...
                       'MaxFunEvals', 100000, 'Display', 'none', ...
                       'DiffMinChange', 0.001, 'Algorithm', 'sqp');
% Solve for the best simulation time + control input
optimal = fmincon(time_min, x0, A, b, Aeq, Beq, lb, ub, ...
              @double_integrator_constraints, options);
% Discretize the times
sim_time = optimal(1);
delta_time = sim_time / gridN;
times = 0 : delta_time : sim_time - delta_time;
% Get the state + accelerations (control inputs) out of the vector
positions = optimal(2             : 1 + gridN);
vels      = optimal(2 + gridN     : 1 + gridN * 2);
accs      = optimal(2 + gridN * 2 : end);
% Make the plots
figure(1);
plot(times + t_start, accs);
title('Control Input (Acceleration) vs Time');
xlabel('Time (s)');
ylabel('Acceleration (m/s^2)');
figure(2);
plot(times + t_start, vels);
title('Velocity vs Time');
xlabel('Time (s)');
ylabel('Velocity (m/s)');
figure(3);
% hold on
plot(times + t_start, positions, 'bo');
title('Position vs Time');
xlabel('Time (s)');
ylabel('Position (m)');
disp(sprintf('Finished in %f seconds', toc));
pause(0.1)
x_start = positions(2);
v_start = min(vels(2), v_max);
a_start = accs(2);
t_start = times(2) + t_start;
fprintf('L: %.3f, V: %.3f, A:%.3f, T:%.5f \n', x_start, v_start, a_start, sim_time);
end