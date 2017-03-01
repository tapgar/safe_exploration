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
    STOMP_NOISE_STD = 0.1;          % noise std for trajectory
    NUM_OF_STATES = 2;              % number of state variables
    H = 10;                         % STOMP regularizing term
    
    
    %#######################################
    %############## VARIABLES ##############
    %#######################################
    cur_state = 0;                    % current state
    end_state = pi;                   % goal state
    u_inputs = [-U_MAX, U_MAX, 0];    % Possible inputs
    M_smoothing = ones(POINTS_IN_TRAJ - 1, 2);                 % Smoothing array for STOMP
    
    
    
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
    
    for i = 1:10
        % Create K trajectories by adding Noise
        STOMP_traj = cell(K,1);
    %     STOMP_traj{:} = zeros(shape(start_traj), 2); % K trajectories x pts x [pos, vel]
    %     plot(pos_traj, 'ro')
    %     hold on
    %     plot(vel_traj, 'bo')
        noise = zeros(K, POINTS_IN_TRAJ, NUM_OF_STATES);
        for i_K = 1:K
            %update noise to be sampled from R^-1
           noise(i_K, :, :) = normrnd(STOMP_NOISE_MEAN, STOMP_NOISE_STD, size([pos_traj, vel_traj]));
           STOMP_traj{i_K} = [pos_traj, vel_traj] + squeeze(noise(i_K,:,:));
    %        plot(STOMP_traj{i_K}(:,1), 'ro')
    %        plot(STOMP_traj{i_K}(:,2), 'bo')
        end
    %     hold off
        % Build Feedback Controller -- LQG
            % linear GP about steps

            % add points while building K matrices
            % other matrices

        % Imagination Rollouts
            % Build matrices
            % Sample at each time step
                % check if it is safe - ?
                % cost of trajectory

        % Choose Policy
        cost = rand(K, POINTS_IN_TRAJ, NUM_OF_STATES);
        importance_weighting = zeros(K, POINTS_IN_TRAJ, NUM_OF_STATES);
        for i_step = 1:length(STOMP_traj{1}(:,1)) % each point in trajectory
            for i_K = 1:K % each trajectory
            % Weight all points sampled during STOMP 
                for i_state = 1:NUM_OF_STATES
                    num = cost(i_K, i_step, i_state) - min(cost(:, i_step, i_state));
                    den = max(cost(:, i_step, i_state)) - min(cost(:, i_step, i_state));
                    importance_weighting(i_K, i_step, i_state) = exp(-H * num/den); 
                end
            end
            
            %normalize for softmax
            for i_state = 1:NUM_OF_STATES
                importance_weighting(:, i_step, i_state) = importance_weighting(:, i_step, i_state) / sum(importance_weighting(:, i_step, i_state));
            end
        end

        % add that to original trajectory
        delta = zeros(POINTS_IN_TRAJ - 1, NUM_OF_STATES);
        for i_step = 1:length(delta)
            for i_states = 1:NUM_OF_STATES
                % delta = sum (prob * noise) for each variation at that point
                delta(i_step, i_states) = importance_weighting(:, i_step, i_states)' * noise(:, i_step, i_states);
            end
        end
        % Smooth with M = smoothing factor
        delta_smooth = M_smoothing .* delta;
        pos_traj_new = pos_traj(1:end-1) + delta_smooth(:,1);
        vel_traj_new = vel_traj(1:end-1) + delta_smooth(:,2);


        plot(pos_traj, 'ro')
        hold on
        plot(vel_traj, 'bo')
        plot(pos_traj_new, 'r*')
        plot(vel_traj_new, 'b*')
        hold off
    end
    % Evaluate Safety of POlicy
    
    % if above threshold -> return policy
    
    % else:
    
    % find point with highest info gain
    
    % create trajectory x0 -> xi -> x0 (probably just use STOMP again)
    
    % update f_cost based on objective (change it somehow)

end