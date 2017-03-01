function SafeAlgo()
    close all
    %#######################################
    %############## CONSTANTS ##############
    %#######################################
    K = 10;                         % noisy trajectories for STOMP
    STEPS = 10;                     % number of steps to linearize for LQG
    
    SAFETY_THRESHOLD = 0.95;        % above this value a policy is considered safe
    U_MAX = 1;                      % max input
    POINTS_IN_TRAJ = 100;           % points in trajectory
    DELTA_T = 0.01;                 % time step size
    STOMP_NOISE_MEAN = 0;           % mean of noise for trajectory
    STOMP_NOISE_STD = 0.1;         % noise std for trajectory

    
    
    %#######################################
    %############## VARIABLES ##############
    %#######################################
    cur_state = 0;                    % current state
    end_state = pi;                   % goal state
    u_inputs = [-U_MAX, U_MAX, 0];    % Possible inputs
    
    
    
    % Create Nominal Trajectory
    % build straight line trajectory
    u_traj = [];
    pos_traj = linspace(cur_state, end_state, POINTS_IN_TRAJ)';
    vel_traj = zeros(size(pos_traj));
    
    for i_traj = 2:length(pos_traj) - 1
        start_traj = pos_traj(i_traj);
        end_traj = pos_traj(i_traj + 1);
        vel_start = vel_traj(i_traj-1);
        vel_traj(i_traj) = (end_traj - start_traj) * 2 / DELTA_T - vel_start;
    end    
    % Trajectory is crazy (need to smooth it out)
%     plot(pos_traj, 'ro')
%     hold on
%     plot(vel_traj, 'bo')
%     hold off
    
    
    % Create K trajectories by adding Noise
    STOMP_traj = cell(K,1);
%     STOMP_traj{:} = zeros(shape(start_traj), 2); % K trajectories x pts x [pos, vel]
%     plot(pos_traj, 'ro')
%     hold on
%     plot(vel_traj, 'bo')
    for i_K = 1:K
       noise = normrnd(STOMP_NOISE_MEAN, STOMP_NOISE_STD, size([pos_traj, vel_traj]));
       STOMP_traj{i_K} = [pos_traj, vel_traj] + noise;
%        plot(STOMP_traj{i_K}(:,1), 'ro')
%        plot(STOMP_traj{i_K}(:,2), 'bo')
    end
%     hold off
    % Build Feedback Controller -- LQG
        % linear GP about steps
        
        % add points while building K matrices
        % other matrices
        
    % Imagination Rollouts
        % Sample at each time step
        % check if it is safe
        % cost of trajectory
        
    % Choose Policy
        % Weight all points sampled during STOMP
        % Build softmax and sample
        % add that to original trajectory
        % Smooth with M = smoothing factor
        
    % Evaluate Safety of POlicy
    
    % if above threshold -> return policy
    
    % else:
    
    % find point with highest info gain
    
    % create trajectory x0 -> xi -> x0 (probably just use STOMP again)
    
    % update f_cost based on objective (change it somehow)

end