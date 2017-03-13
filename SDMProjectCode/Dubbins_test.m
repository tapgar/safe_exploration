%% straight line

dt = 0.1;
u = ones(100,1) * [1, 0];
q0 = [0, 0, 0, 0, 0];
D = DubbinsEnv(@dubbins_map);
[q] = D.forward_traj(q0, u);

plot(q(:,1), q(:, 2), 'ro')
hold on


%% Simple Turn - shows unconstrained dynamics issue
dt = 0.1;
u = ones(100,1) * [.2, 0.00];
q0 = [0, 0, 0, 1.9, 0.1];
D = DubbinsEnv(@dubbins_map);
[q, c] = D.forward_traj(q0, u);
hold on
for i_u = 1:length(u)
    if c(i_u) == 1
        col = 'ro';
    else
        col = 'bo';
    end
    plot(q(i_u,1), q(i_u, 2), col)
    pause(0.01)
end
hold off
%% Multiple Turns
dt = 0.1;
u = ones(100,1) * [1, 0.1];
u(50:end,2) = -0.1;
q0 = [0, 0, 0, 0, 0];
q = dubbinsDynamics(u, q0, dt);

plot(q(:,1), q(:, 2), 'ro')

%% Multiple Turns
dt = 0.1;
t = 0:dt:10-dt;
u = ones(100,1) * [1, 0.1];
u(:,2) = sin(t);
q0 = [0, 0, 0, 0, 0];
q = dubbinsDynamics(u, q0, dt);

plot(q(:,1), q(:, 2), 'ro')

%% straight line - changing velocity

dt = 0.1;
u = ones(100,1) * [1, 0];
u(20:end, 1) = -1;
q0 = [0, 0, 0, 0, 0];
q = dubbinsDynamics(u, q0, dt);

plot(q(:,1), q(:, 2), 'ro')

%% initial theta - reversing

dt = 0.1;
u = ones(100,1) * [1, 0.1];
u(20:end, 1) = -1;
q0 = [0, 0, pi/8, 0, 0];
q = dubbinsDynamics(u, q0, dt);

plot(q(:,1), q(:, 2), 'ro')

%% Simple Turn - changing velocity

dt = 0.1;
u = ones(100,1) * [1, 0.1];
u(20:end, 1) = -1;
q0 = [0, 0, 0, 0, 0];
q = dubbinsDynamics(u, q0, dt);

plot(q(:,1), q(:, 2), 'ro')

%% Max Turn
dt = 0.1;
u = ones(100,1) * [1, 1];
q0 = [0, 0, 0, 0, 0];
q = dubbinsDynamics(u, q0, dt);

plot(q(:,1), q(:, 2), 'ro')

%% Check Map - Collision
s = 1:10; s = s' * [1, 1];
safe = dubbins_map(s);
fprintf('Expected: 0, Predicted: %d \n', safe)

%% Check Map - No Collision
t = linspace(0,pi/2, 10)';
s = [10 * cos(t), 10 * sin(t)]; 
safe = dubbins_map(s);
fprintf('Expected: 1, Predicted: %d \n', safe)


%% Test Object
D = DubbinsEnv(@dubbins_map);
D = D.NominalTrajectory();
plot(D.U_NOM(:,1), D.U_NOM(:,2))

%% Test ICE
dt = 0.1;
u = ones(100,1) * [.5, 0];
q0 = [0, 0, 0, 0, 0.3];
D = DubbinsEnv(@dubbins_map);
[q, c] = D.forward_traj(q0, u);
hold on
for i_u = 1:length(u)
    if c(i_u) == 1
        col = 'ro';
    else
        col = 'bo';
    end
    plot(q(i_u,1), q(i_u, 2), col)
    pause(0.01)
end
hold off