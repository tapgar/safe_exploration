function u = STOMP(env, K, GP, invGP, icyGP, invicyGP, cost_func)
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
    time_array = env.DELTA_T * linspace(0, env.POINTS_IN_TRAJ, 2+env.POINTS_IN_TRAJ);
    nom_traj = reshape(env.U_NOM',env.POINTS_IN_TRAJ*3,1);                           % nominal initial trajectory
    nom_traj = nom_traj(1:env.POINTS_IN_TRAJ*2,1);
    noise_factor = 500/env.POINTS_IN_TRAJ;
    noise_cooling = 0.95;                          % cooling rate for noise
    Q_diff = 10^10;                         % difference between successive trajectories
    Q_prev = 10^10;                         % cost of previous trajectory
    Q_prev_arr = 0;
    Q_conv_counter = 0;                     % Q convergence limit
    Q_conv = 5;
    

    %###########################################
    %############## VISUALIZATION ##############
    %###########################################
    STOMP_1            = false; 
    STOMP_2            = true; %visualize end trajectories
    STOMP_3            = false;  %viusalize Q over time
    
    
    
    % K is number of noisy trajectories to try
    % env is the enviornment that we test over
    
    % Create K trajectories by adding Noise
    STOMP_traj = cell(K,1);
    POINTS_IN_TRAJ = env.POINTS_IN_TRAJ;
    [M, A, R, R_1] = precompute(POINTS_IN_TRAJ, env.DELTA_T);   % Smoothing array for STOMP  
    while  Q_conv_counter < Q_conv % iterations over STOMP
        %sample according to covariance matrix... R_1 issues fixed
        noise = [mvnrnd(zeros(1, POINTS_IN_TRAJ), R_1, K) ,mvnrnd(zeros(1, POINTS_IN_TRAJ), R_1, K)] .* noise_factor;
        
        %add noise to K trajectories
        for i_K = 1:K
            STOMP_traj{i_K} = nom_traj + noise(i_K,:)';
            
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
        cost = zeros(K, 2*POINTS_IN_TRAJ);
        for i_K = 1:K
            traj = [env.START_STATE(1:env.TRAJ_DIMS); reshape(STOMP_traj{i_K}(:, 1),env.POINTS_IN_TRAJ,2); env.END_STATE(1:env.TRAJ_DIMS)];
            %[policy, cov, ~, ~] = LQR_GP(traj,GP,10.*eye(2),1,[]);
            pcost = cost_func(2.2, traj, env, GP, invGP, icyGP, invicyGP);
            cost(i_K,:) = [pcost', pcost'];
        end
        importance_weighting = zeros(K, POINTS_IN_TRAJ, NUM_OF_INPUTS);
        for i_step = 1:length(STOMP_traj{1}(:,1)) % each point in trajectory
            for i_K = 1:K % each trajectory
            % Weight all points sampled during STOMP 
                for i_input = 1:NUM_OF_INPUTS
                    num = cost(i_K, i_step) - min(cost(:, i_step));
                    den = 0.00000000001 + max(cost(:, i_step)) - min(cost(:, i_step));
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
        delta = zeros(POINTS_IN_TRAJ*2, 1);
        for i_step = 1:length(delta) % this can be done with a single matrix operation
            delta(i_step) = sum(importance_weighting(:, i_step) .* noise(:,i_step));
        end

        % Smooth with M = smoothing factor
        delta_smooth = reshape((M * reshape(delta,env.POINTS_IN_TRAJ,2)),env.POINTS_IN_TRAJ*2,1);
        
        u_traj_new = nom_traj + delta_smooth;
        traj = [env.START_STATE(1:env.TRAJ_DIMS); reshape(u_traj_new,env.POINTS_IN_TRAJ,2); env.END_STATE(1:env.TRAJ_DIMS)];
%         qdd_targ = zeros(POINTS_IN_TRAJ+2,1);
%         qd_targ = qdd_targ;
%         for k = 2:1:POINTS_IN_TRAJ+1
%             qdd_targ(k,1) = (traj(k-1,1:env.TRAJ_DIMS)' - 2.*traj(k,1:env.TRAJ_DIMS)' + traj(k+1,1:env.TRAJ_DIMS)')./(env.DELTA_T^2);
%             qd_targ(k,1) = (traj(k+1,1:env.TRAJ_DIMS)' - traj(k-1,1:env.TRAJ_DIMS)')./(2.*env.DELTA_T);
%         end
%         vel_traj_new = [0; diff(u_traj_new)./DELTA_T];
%         acc_traj_new = [0; diff(vel_traj_new)./DELTA_T];
        

        if STOMP_2
            figure(1)
            clf
            env.f_map([0,0]);
%             subplot(3,1,1) % plot all trajectories compared to original
%             plot(time_array, traj, 'k', 'LineWidth', 3)
            hold on
            for i_K = 1:K
                traj2 = [env.START_STATE(1:env.TRAJ_DIMS); reshape(STOMP_traj{i_K},env.POINTS_IN_TRAJ,2); env.END_STATE(1:env.TRAJ_DIMS)];
                plot(traj2(:,1),traj2(:,2))
            end
%             title('Noisy Trajectories')
%             hold off
%                        
%             subplot(3,1,2) % plot all pos vs vel
%             hold on
%             for i_K = 1:K
%                 %plot(STOMP_traj{i_K}(:, 1)) % pos
%                 plot(STOMP_traj{i_K}(:, 2)) % vel
%             end
%             
%             plot(diff(nom_traj)./env.DELTA_T,'r-')
%             title('Position & Velocity vs Step')
%             subplot(3,1,3)
%             hold on
%             plot(time_array, traj, 'r*', time_array, qd_targ, 'b*', time_array, qdd_targ, 'g*')
            hold off
            title('Completed STOMP Trajectory (Vel, Pos)')
            pause(.1)
        end

        nom_traj = u_traj_new;
        noise_factor = noise_cooling * noise_factor;
        Q_traj = sum(collision_cost_function(2.2,traj,env, GP, invGP, icyGP, invicyGP)) + sum(0.5 * traj(2:end-1,:)' * R * traj(2:end-1,:));
        Q_diff = abs(Q_traj - Q_prev)/Q_prev;
        if Q_diff < 0.000005  %1% decrease
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
    
    u = [env.START_STATE(1:env.TRAJ_DIMS); reshape(u_traj_new,env.POINTS_IN_TRAJ,2); env.END_STATE(1:env.TRAJ_DIMS)];

end