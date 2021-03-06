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

%% Test ICE - 1
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

%% Test ICE - 2
dt = 0.1;
u = ones(150,1) * [0.5, 0.0];
u(:,2) = linspace(0.04, -0.04, size(u,1));
for i1 = 1:1
    for i2 = 1:1
        q0 = [0, 0, 0, 0, 0];
        D = DubbinsEnv(@dubbins_map);
        [q, c, s] = D.forward_traj(q0, u);
        hold on
        for i_u = 1:length(u)
            if c(i_u) ~= 0.05
                col = 'ro'; % regular road
            else
                col = 'go'; % ice
            end
            if s(i_u) == 1
                fill = 'white'; %collision
            else
                fill = 'black'; %no collision
            end

            plot(q(i_u,1), q(i_u, 2), col, 'MarkerFaceColor', fill)
            pause(0.01)
        end
    end
end
hold off

%% Plan Path and Follow along map showing collisions and ice
dt = 0.1;
D = DubbinsEnv(@dubbins_map);
D = D.NominalTrajectory;
traj = D.U_NOM;
controller = @(e, ed) Kp*e + Kd*ed;
cur = D.START_STATE;
u = zeros(size(traj,1),2);
% for i_p = 1:size(traj,1)-1
%     e = 
%     u(i_p, :) = 
%     
% end

traj_diff = diff(traj(end:-1:1, :),2)./ dt^2; % get acceleration from path
traj_acc = traj_diff(:, :).^2; % square everything
traj_acc = traj_acc(:,1) + traj_acc(:,2); % sum up x and y acc
traj_acc = traj_acc .^ (1/2); %get norm
u = [traj_acc, traj_diff(:,3)];
q0 = [0, 0, 0, 0, 0];
[q, c, s] = D.forward_traj(q0, u);
for i1 = 1:2
    for i2 = 1:2
        q0 = [0, 0, 0, 0, 0];
        [q, c, s] = D.forward_traj(q0, u);
        hold on
        for i_u = 1:length(u)
            if c(i_u) == 1
                col = 'ro';
            else
                col = 'go';
            end
            if s(i_u) == 1
                fill = 'black';
            else
                fill = 'white';
            end

            plot(q(i_u,1), q(i_u, 2), col, 'MarkerFaceColor', fill)
            pause(0.01)
        end
    end
end
hold off