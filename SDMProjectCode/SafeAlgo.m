function SafeAlgo()
    close all
    
    %#######################################
    %################ TODO #################
    %#######################################
    % Test out adding points to OGLP
    
    % Fix sampling from R_1 for STOMP
    
    % STOMP is definitely not working
        % need to change to u's from pos,vel ***P1***
    
    % change environment to class that has map as input
        % return things like 
            % Nominal Trajectory
            % points in trajectory
            %
            
    % roll out
        % make rollout a subfunction so can call it recursively
        % weird error as it gets closer to end.
    
    % post loop stuff
    
    
    
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
    ROLLOUTS = 10;                  % Number of imaginary rollouts
    SAFETY_THRESHOLD = 0.95;        % Threshold above which to accept trajectory
    
    
    %###########################################
    %############## VISUALIZATION ##############
    %###########################################
    NOMINAL_TRAJECTORY = false;
    STOMP_1            = false;
    STOMP_2            = false;
    
    
    %#######################################
    %############## VARIABLES ##############
    %#######################################
    cur_state = [0, 0];                              % current state
    end_state = [pi, 0];                             % goal state
%     u_inputs = [-U_MAX, U_MAX, 0];              % Possible inputs
    [M, A, R_1] = precompute(POINTS_IN_TRAJ);   % Smoothing array for STOMP
    max_vel = 4;
    min_vel = -4;
    
    
    
    % Create Nominal Trajectory
    % build straight line trajectory
    u_traj = [];
    pos_traj = linspace(cur_state(1), end_state(1), POINTS_IN_TRAJ)';
    vel_traj = linspace(cur_state(2), end_state(2), POINTS_IN_TRAJ)';
    time_array = DELTA_T * linspace(1, POINTS_IN_TRAJ);
    
    for i_traj = 2:length(pos_traj) - 1
        start_traj = pos_traj(i_traj);
        end_traj = pos_traj(i_traj + 1);
        vel_start = vel_traj(i_traj-1); 
        vel_traj(i_traj) = (end_traj - start_traj) * 2 / DELTA_T - vel_start;
        if i_traj == 2
            vel_traj(i_traj) = vel_traj(i_traj)/2; % cheating to make it not go full speed and then slow down
        end
    end    
    if NOMINAL_TRAJECTORY
        figure()
        plot(pos_traj, 'ro')
        hold on
        plot(vel_traj, 'bo')
        hold off
        title('Nominal Trajectory')
    end
    % ########## STOMP ############# %
    for i = 1:50 % iterations over STOMP
        % Create K trajectories by adding Noise
        STOMP_traj = cell(K,1);
%         noise = zeros(K, POINTS_I/N_TRAJ, NUM_OF_STATES);
% If A is a matrix whose columns represent random variables and whose rows 
% represent observations, C is the covariance matrix with the corresponding
% column variances along the diagonal.
%         R_1(R_1 < 10^-4) = 10^-4; %try to fix problem with matrix
%         R_1_pad = padarray(R_1, [2 2],0 , 'post');
        R_1 = (R_1 + R_1') / 2; %need this for mvnrnd to work - i have no idea why
        % https://www.mathworks.com/matlabcentral/answers/63168-error-message-in-using-mvnrnd-function
        noise = mvnrnd(zeros(1, POINTS_IN_TRAJ), R_1, K); % this should work but it doesn't?!
        for i_K = 1:K
            %update noise to be sampled from R^-1
%            noise(i_K, :, :) = normrnd(STOMP_NOISE_MEAN, STOMP_NOISE_STD, size([pos_traj, vel_traj]));
           % first and last points shouldn't change
%            noise(i_K, 1, :) = [0, 0]; noise(i_K, end, :) = [0, 0];
%            STOMP_traj{i_K} = [pos_traj, vel_traj] + squeeze(noise(i_K,:,:));
           STOMP_traj{i_K} = [pos_traj, vel_traj] + noise(i_K,:)';
           
           if STOMP_1
               plot(STOMP_traj{i_K}(:,1), 'ro')
               hold on
               plot(STOMP_traj{i_K}(:,2), 'bo')
               hold off
               title('STOMP Trajectories')
           end
        end

%         cost = zeros(K, POINTS_IN_TRAJ, NUM_OF_STATES);
        acc_traj = zeros(K, POINTS_IN_TRAJ);
        cost = zeros(K, POINTS_IN_TRAJ);
        for i_K = 1:K
            acc_traj(i_K,:) = [diff(STOMP_traj{i_K}(:,2)); 0];
            for i_PIJ = 1:POINTS_IN_TRAJ
                % should this be one cost for all state?
                cost(i_K, i_PIJ) = S(STOMP_traj{i_K}(i_PIJ, :)) - acc_traj(i_K, i_PIJ) ^ 2; 
                
            end
        end
        importance_weighting = zeros(K, POINTS_IN_TRAJ, NUM_OF_STATES);
        for i_step = 1:length(STOMP_traj{1}(:,1)) % each point in trajectory
            for i_K = 1:K % each trajectory
            % Weight all points sampled during STOMP 
                for i_state = 1:NUM_OF_STATES
                    num = cost(i_K, i_step) - min(cost(:, i_step));
                    den = max(cost(:, i_step)) - min(cost(:, i_step));
                    importance_weighting(i_K, i_step, i_state) = exp(-H * num/den); 
                end
            end
            
            %normalize for softmax
            for i_state = 1:NUM_OF_STATES
                importance_weighting(:, i_step, i_state) = importance_weighting(:, i_step, i_state) / sum(importance_weighting(:, i_step, i_state));
            end
        end

        % add that to original trajectory
        delta = zeros(POINTS_IN_TRAJ, NUM_OF_STATES);
        for i_step = 1:length(delta)
            for i_states = 1:NUM_OF_STATES
                % delta = sum (prob * noise) for each variation at that point
                delta(i_step, i_states) = importance_weighting(:, i_step, i_states)' * noise(i_step);
            end
        end
        % Smooth with M = smoothing factor
        delta_smooth = M * delta;
        pos_traj_new = pos_traj(1:end) + delta_smooth(:,1);
        vel_traj_new = vel_traj(1:end) + delta_smooth(:,2);
       % clip velocities
       vel_traj_new = max(min(vel_traj_new, max_vel), min_vel);

        if STOMP_2
            figure(1)
            hold on
            plot(pos_traj,vel_traj, 'bo')
            plot(pos_traj_new,vel_traj_new, 'b*')
            figure(2)
            hold on
            plot(time_array, pos_traj, 'ro', time_array, vel_traj, 'bo')
            plot(time_array, pos_traj_new, 'r*', time_array, vel_traj_new, 'b*')
            hold off
            title('Completed STOMP Trajectory')
            pause(.1)
        end

        pos_traj = pos_traj_new;
        vel_traj = vel_traj_new;
    end
    % ########## END STOMP ############# %
    
    
    
    
        % Build Feedback Controller -- LQG
            % linear GP about steps

            % add points while building K matrices
            % other matrices

    % ############# Imagination Rollouts ################ %
    %PD Controller
    Kp = 100; Kd = 1;
    controller = @(e, e_dot) Kp * e + Kd * e_dot;
    DELTA_T;
    % Build matrices - done as precompute step
    addpath('../OLGP')
    hp = struct('y_std',0.1027,'sig_std',3.9978,'W',eye(3));
    model = LocalGP(200, 30, 0.8, hp);
    p_total = 0;
    commanded = zeros(POINTS_IN_TRAJ, NUM_OF_STATES);
    actual = zeros(POINTS_IN_TRAJ, NUM_OF_STATES*3/2);
    e = commanded; e_dot = commanded;
    commanded = [pos_traj, vel_traj];
    commanded(1, 2) = vel_traj(1);
    commanded(2, 2) = vel_traj(2);
    u = zeros(POINTS_IN_TRAJ, 1);
    model = model.add_training_data([0,0,0], 0);
    actual(1,1:2) = commanded(1,:);
    for i_PIJ_real = 2:POINTS_IN_TRAJ
        model2 = model;
        p_safe = zeros(ROLLOUTS,1);
        fprintf('Point: %.f ', i_PIJ_real)
        for i_ro = 1:ROLLOUTS
            p_total = 0;
            for i_PIJ = i_PIJ_real:POINTS_IN_TRAJ
                e(i_PIJ) = [commanded(i_PIJ, 1) - actual(i_PIJ-1,1)];
                e_dot(i_PIJ) = [commanded(i_PIJ, 2) - actual(i_PIJ-1,2)];
                u(i_PIJ) = controller(e(i_PIJ), e_dot(i_PIJ));
                % Sample at each time step
                actual(i_PIJ,3) = model2.query_data_point([actual(i_PIJ-1,1:2), u(i_PIJ)]);
                % GP was only giving 0 so i am trying this.
                if abs(actual(i_PIJ,3)) < rand(1) 
                    actual(i_PIJ,3) = actual(i_PIJ,3) + rand(1);
                end
                actual(i_PIJ,2) = actual(i_PIJ-1,2) + DELTA_T * actual(i_PIJ,3); %velocity update
                actual(i_PIJ,1) = actual(i_PIJ-1,1) + DELTA_T * (actual(i_PIJ,2) + actual(i_PIJ-1,2))/2; % position update, velocity is taken as average of old and new
                model2 = model2.add_training_data([actual(i_PIJ-1,1:2), u(i_PIJ)], actual(i_PIJ,3));
                % check if it is safe
                p = env_map(actual(i_PIJ-1,1), actual(i_PIJ,1));
                if p == 0
%                     disp('Unsafe Point!')
                    break
                else
                    p_total = p_total + p;
                end
            end
        p_safe(i_ro) = p_total / POINTS_IN_TRAJ;
        fprintf('Rollout %.f, Steps %.f: %.3f \n', i_ro, i_PIJ, p_safe(i_ro))
        end
%         hold on
%         plot(actual(:,1),'color',rand(1,3),'marker','o')
%         plot(commanded(:,1),'bo')
%         plot(actual(:,2),'r*')
%         plot(commanded(:,2),'b*')
%         hello = 1;
    end
    
%     gp_x_1 = linspace(min(actual(:,1)), max(actual(:,1)), 100);
%     gp_x_2 = linspace(min(actual(:,2)), max(actual(:,2)), 100);
%     [gp_X_1, gp_X_2] = meshgrid(gp_x_1, gp_x_2);
%     ypred = zeros(100,100);
%     V = zeros(100,100);
%     for j = 1:100
%         for k = 1:100
%             [ypred(j,k), V(j,k)]  = model.query_data_point([gp_X_1(j,k), gp_X_2(j,k), 1]);
%         end
%     end
%     figure()
%     plot3(gp_x_1, gp_x_2, ypred)
    % cost of trajectory
    % ############# END Imagination Rollouts ################ %
        
        
    % Choose Policy - no choosing. policy is either good or not?
 

    % Evaluate Safety of POlicy
    % if above threshold -> return policy
    if  mean(p_safe(:)) > SAFETY_THRESHOLD
%         output = model;
          hello = 1;
    else
        hello = 1;
    end
        % find point with highest info gain
    
    % create trajectory x0 -> xi -> x0 (probably just use STOMP again)
    
    
    
    % update f_cost based on objective (change it somehow)



    function c = S(state) %this should be part of env?
        pos = state(1);
        vel = state(2);

        c = -(end_state(1) - pos)^2 - (end_state(2) - vel)^2; 
    end

%     function p_safe = rollout(s0, GP, k_ro, sE)
%         addpath('../OLGP')
%         p_total = 0;
%        for n = 1:POINTS_IN_TRAJ
%           sample = GP(x, controller(x));
%           [s_next, p] = env_map(sample);
%           p_total = p_total + p;
%        end
%        p_safe = p_total / POINTS_IN_TRAJ;
%     end


end