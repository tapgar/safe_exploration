function u = STOMP(env, K)
    % env should have initial trajectories.  Only using physical position
    % of points (q)
    
    %#######################################
    %################ TODO #################
    %#######################################
    % STOMP is sorta working
        % a little unclear on how to clip u's to stay in range
        % end point is all over the place
        % should cost of STOMP incorporate space being safe? - YES
        
    % should be able to do this easily with multiple input dimensions
        
        
        
    %#######################################
    %############## VARIABLES ##############
    %#######################################
    NUM_OF_INPUTS = env.NUM_OF_INPUTS;
    DELTA_T = env.DELTA_T;
    H = 10;                                             % STOMP regularizing term
    time_array = env.DELTA_T * linspace(1, env.POINTS_IN_TRAJ);
    nom_traj = env.U_NOM;                           % nominal initial trajectory
    noise_factor = 1;
    NOISE_COOLING = 0.99;                          % cooling rate for noise
    Q_diff = 10^10;                         % difference between successive trajectories
    Q_prev = 10^10;                         % cost of previous trajectory
    Q_prev_arr = 0;
    Q_conv_counter = 0;                     % Q convergence limit
    Q_conv = 5;
    Q_LIMIT = 0.001;                        % Percent change in Q to be considered converging
    

    %###########################################
    %############## VISUALIZATION ##############
    %###########################################
    STOMP_1            = false; 
    STOMP_2            = false; %visualize end trajectories
    STOMP_3            = true;  %viusalize Q over time
    
    
    
    % K is number of noisy trajectories to try
    % env is the enviornment that we test over
    
    % Create K trajectories by adding Noise
    STOMP_traj = cell(K,1);
    POINTS_IN_TRAJ = env.POINTS_IN_TRAJ;
    [M, A, R, R_1] = precompute(POINTS_IN_TRAJ);   % Smoothing array for STOMP
    

    while  Q_conv_counter < Q_conv % iterations over STOMP
        
        R_1 = (R_1 + R_1') / 2; %need this for mvnrnd to work - i have no idea why
        % https://www.mathworks.com/matlabcentral/answers/63168-error-message-in-using-mvnrnd-function
        noise = mvnrnd(zeros(1, POINTS_IN_TRAJ), R_1, K) * noise_factor;
        for i_K = 1:K
            STOMP_traj{i_K} = nom_traj + noise(i_K,:)';
%             z0 = [0,0]; %initial condition
%             [z_new, p_unsafe] = env.forward_traj(z0, STOMP_traj{i_K});
%             pos_traj_init = z_new(:,1);
%             vel_traj_init = z_new(:,2);
            vel_traj_nom = diff(STOMP_traj{i_K}) / DELTA_T; % differentiating to get velocity
            acc_traj_nom = diff(vel_traj_nom) / DELTA_T;    % differentiating to get acceleration
%             STOMP_traj{i_K} = [STOMP_traj{i_K}, z_new];
            STOMP_traj{i_K} = [STOMP_traj{i_K}, [vel_traj_nom; 0], [acc_traj_nom; 0; 0]]; %padding arrays to be same length

           if STOMP_1
               plot(STOMP_traj{i_K}(:,1), 'ro')
               hold on
               plot(STOMP_traj{i_K}(:,2), 'bo')
               plot(STOMP_traj{i_K}(:,3), 'go')
               hold off
               title('STOMP Trajectories')
               pause(0.1)
           end
        end
        
        cost = zeros(K, POINTS_IN_TRAJ);
        for i_K = 1:K
            for i_PIJ = 1:POINTS_IN_TRAJ
                % should this be one cost for all state?
                cost(i_K, i_PIJ) = S(STOMP_traj{i_K}(i_PIJ, :), env.END_STATE);
                
            end
        end
        
        importance_weighting = zeros(K, POINTS_IN_TRAJ, NUM_OF_INPUTS);
        for i_step = 1:length(STOMP_traj{1}(:,1)) % each point in trajectory
            for i_K = 1:K % each trajectory
            % Weight all points sampled during STOMP 
                for i_input = 1:NUM_OF_INPUTS
                    num = cost(i_K, i_step) - min(cost(:, i_step));
                    den = max(cost(:, i_step)) - min(cost(:, i_step));
                    importance_weighting(i_K, i_step, i_input) = exp(-H * num/den); 
                end
            end
            
            %normalize for softmax
            for i_input = 1:NUM_OF_INPUTS
                importance_weighting(:, i_step, i_input) = importance_weighting(:, i_step, i_input) / sum(importance_weighting(:, i_step, i_input));
            end
        end

        % add that to original trajectory
        % will need to do something if there are multiple inputs
        delta = zeros(POINTS_IN_TRAJ, 1);
        for i_step = 1:length(delta) % this can be done with a single matrix operation
            % delta = sum (prob * noise) for each variation at that point
            delta(i_step) = sum(importance_weighting(:, i_step) .* noise(i_step));

        end
        % Smooth with M = smoothing factor
        delta_smooth = M * delta;
        
        u_traj_new = env.U_NOM + delta_smooth;
%         u_traj_new = max(min(u_traj_new, env.U_MAX), env.U_MIN);
%         [z_new, p_new] = env.forward_traj(env.START_STATE, u_traj_new);
%         pos_traj_new = z_new(:,1);
%         vel_traj_new = z_new(:,2);
        vel_traj_new = [diff(u_traj_new); 0];
        acc_traj_new = [diff(vel_traj_new); 0];
        

        if STOMP_2
            figure(1)
            clf
            subplot(3,1,1) % plot all trajectories compared to original
            plot(env.U_NOM, 'ro')
            hold on
            for i_K = 1:K
                plot(STOMP_traj{i_K}(:,1))
            end
            title('Noisy Trajectories')
            hold off
            subplot(3,1,2) % plot all pos vs vel
            hold on
            for i_K = 1:K
                plot(STOMP_traj{i_K}(:, 1)) % pos
                plot(STOMP_traj{i_K}(:, 2)) % vel
            end
            title('Position & Velocity vs Step')
            subplot(3,1,3)
            hold on
%             plot(time_array, pos_traj, 'ro', time_array, vel_traj, 'bo')
            plot(time_array, u_traj_new, 'r*', time_array, vel_traj_new, 'b*', time_array, acc_traj_new, 'g*')
            hold off
            title('Completed STOMP Trajectory (Vel, Pos)')
            pause(.1)
        end

        nom_traj = u_traj_new;
        noise_factor = NOISE_COOLING * noise_factor;
        Q_traj = sum(S([u_traj_new, vel_traj_new], env.END_STATE)) + 0.5 * u_traj_new' * R * u_traj_new;
        Q_diff = abs(Q_traj - Q_prev)/Q_prev;
        if Q_diff < Q_LIMIT  % 1% decrease
            Q_conv_counter = Q_conv_counter + 1;
        else
            Q_conv_counter = 0;
        end
        Q_prev = Q_traj;
        if STOMP_3
            fprintf('Cost of Trajectory: %.3f, Difference: %.3f \n', Q_traj, Q_diff)
            Q_prev_arr = [Q_prev_arr, Q_prev];
            plot(Q_prev_arr)
            pause(0.1)
        end
    end
    
u = [u_traj_new, vel_traj_new, acc_traj_new];

    function c = S(state, end_state) %this should be part of env?
        pos = state(:,1);
        vel = state(:,2);
        % penalized for being far from goal and having high velocity
        c = (end_state(1) - pos).^2 ; %+ (end_state(2) - vel).^2; 
    end
end