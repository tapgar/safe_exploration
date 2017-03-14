close all
clear

hp = struct('y_std',0.01,'sig_std',1.39978,'W',0.1.*eye(3));
GP = LocalGP(200, 10, 0.5, hp);

env = PendulumEnv();
start_pts = 10;
[x,y] = env.generateLocalData([0,0,0],start_pts);

for i = 1:1:start_pts
    GP = GP.add_training_data(x(i,:),y(i,:));
end

% global gridN
% gridN = 20;
% 
% tic
% %minimize the simulation time
% time_min = @(x) (x(1) + 10.*sum(x(2 + gridN * 3 : end).^2));
% 
% % The initial parameter guess; 1 second, gridN positions, gridN velocities,
% % gridN accelerations
% x0 = [10; linspace(0,3.1415,gridN)'; linspace(3.1415/gridN,3.1415/gridN,gridN)'; ones(gridN, 1)*1; zeros(gridN, 1)];
% 
% % No linear inequality or equality constraints
% A = [];
% b = [];
% Aeq = [];
% Beq = [];
% 
% maxU = 2;
% 
% f = @(x)pendulum_constraints(x, GP);
% 
% 
% lb = [0;    ones(gridN * 2, 1) * -Inf;  ones(gridN, 1) * -maxU;    ones(gridN * 1, 1) * -Inf];
% ub = [Inf;  ones(gridN * 2, 1) * Inf;   ones(gridN, 1) * maxU;    ones(gridN * 1, 1) * Inf];
% % Options for fmincon
% options = optimoptions(@fmincon, 'TolFun', 0.00000001, 'MaxIter', 10000, ...
%                        'MaxFunEvals', 100000, 'Display', 'iter', ...
%                        'DiffMinChange', 0.001, 'Algorithm', 'sqp');
% % Solve for the best simulation time + control input
% optimal = fmincon(time_min, x0, A, b, Aeq, Beq, lb, ub, ...
%               f, options);

optimal = env.DirColTrajectory(GP);

gridN = env.dircolPts;
          
% Discretize the times
sim_time = optimal(1);
delta_time = sim_time / gridN;
times = 0 : delta_time : sim_time - delta_time;
% Get the state + accelerations (control inputs) out of the vector
positions = optimal(2             : 1 + gridN);
vels      = optimal(2 + gridN     : 1 + gridN * 2);
accs      = optimal(2 + gridN * 2 : 1 + gridN * 3);
% Make the plots
figure();
plot(times, accs);
title('Control Input (Acceleration) vs Time');
xlabel('Time (s)');
ylabel('Acceleration (m/s^2)');
figure();
plot(times, vels);
title('Velocity vs Time');
xlabel('Time (s)');
ylabel('Velocity (m/s)');
figure();
plot(times, positions);
title('Position vs Time');
xlabel('Time (s)');
ylabel('Position (m)');
