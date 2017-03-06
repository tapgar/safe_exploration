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
    time_array = env.DELTA_T * linspace(1, 1, env.POINTS_IN_TRAJ);
    nom_traj = env.U_NOM;                           % nominal initial trajectory
    noise_factor = 1;
    noise_cooling = 0.99;                          % cooling rate for noise
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
    STOMP_2            = true; %visualize end trajectories
    STOMP_3            = false;  %viusalize Q over time
    
    
    
    % K is number of noisy trajectories to try
    % env is the enviornment that we test over
    
    % Create K trajectories by adding Noise
    STOMP_traj = cell(K,1);
    POINTS_IN_TRAJ = env.POINTS_IN_TRAJ;
    [M, A, R, R_1] = precompute(POINTS_IN_TRAJ);   % Smoothing array for STOMP
   
    %R_1 = R_1./max(max(R_1));
    
    while  Q_conv_counter < Q_conv % iterations over STOMP
        
        %R_1 = (R_1 + R_1') / 2; %need this for mvnrnd to work - i have no idea why
        % https://www.mathworks.com/matlabcentral/answers/63168-error-message-in-using-mvnrnd-function
        noise = mvnrnd(zeros(1, POINTS_IN_TRAJ), R_1, K) * noise_factor;
        for i_K = 1:K
            STOMP_traj{i_K} = nom_traj + noise(i_K,:)';
%             z0 = [0,0]; %initial condition
%             [z_new, p_unsafe] = env.forward_traj(z0, STOMP_traj{i_K});
%             pos_traj_init = z_new(:,1);
%             vel_traj_init = z_new(:,2);
%             vel_traj_nom = diff(STOMP_traj{i_K}) / DELTA_T; % differentiating to get velocity
%             acc_traj_nom = diff(vel_traj_nom) / DELTA_T;    % differentiating to get acceleration
            
            
            vel_traj_nom = cdiff(STOMP_traj{i_K}, DELTA_T); % differentiating to get velocity
            acc_traj_nom = cdiff(vel_traj_nom, DELTA_T);    % differentiating to get acceleration
            
            
%             STOMP_traj{i_K} = [STOMP_traj{i_K}, z_new];
%             STOMP_traj{i_K} = [STOMP_traj{i_K}, [vel_traj_nom; 0], [0; acc_traj_nom;0]];
            STOMP_traj{i_K} = [STOMP_traj{i_K}, vel_traj_nom, acc_traj_nom]; %padding arrays to be same length

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
                
                traj = STOMP_traj{i_K}(:, 1);
%                 cu = (diff([0;diff(traj)./DELTA_T;0])./DELTA_T).^2;
                cu = STOMP_traj{i_K}(:,3).^2;
%                 if i_K == 1
%                     cost(i_K,i_PIJ) = 0;
%                 else
%                     cost(i_K,i_PIJ) = 1000;
%                 end
%                 cost(i_K, i_PIJ) = sum(cu);
                cost(i_K, i_PIJ) = cu(i_PIJ);
                %cost(i_K, i_PIJ) = STOMP_COST(STOMP_traj{i_K}(i_PIJ, :), env.END_STATE) + 0.5 * STOMP_traj{i_K}(i_PIJ:end, 1)' * R(i_PIJ:end,i_PIJ:end) * STOMP_traj{i_K}(i_PIJ:end, 1);
                
            end
%             figure(1)
%             plot(traj)
%             hold on
%             figure(2)
%             plot(diff(traj))
%             hold on
%             
%             pos_traj = STOMP_traj{i_K}(:, 1);
            
        end
        importance_weighting = zeros(K, POINTS_IN_TRAJ, NUM_OF_INPUTS);
        for i_step = 1:length(STOMP_traj{1}(:,1)) % each point in trajectory
            for i_K = 1:K % each trajectory
            % Weight all points sampled during STOMP 
                for i_input = 1:NUM_OF_INPUTS
                    num = cost(i_K, i_step) - min(cost(:, i_step));
                    den = max(cost(:, i_step)) - min(cost(:, i_step));
                    if den == 0
                        den = den + 10^-8;
                    end
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
            delta(i_step) = sum(importance_weighting(:, i_step) .* noise(:,i_step));

        end
%         figure; plot(STOMP_traj{1,1}(:,1))
%         hold on; plot(nom_traj+delta)
        % Smooth with M = smoothing factor
        delta_smooth = M * delta;
        
        u_traj_new = nom_traj + delta_smooth;
%         plot(u_traj_new)
%         figure(1)
%         hold on
%         plot(u_traj_new,'k','LineWidth',5)
%         hold off
%         u_traj_new = max(min(u_traj_new, env.U_MAX), env.U_MIN);
%         [z_new, p_new] = env.forward_traj(env.START_STATE, u_traj_new);
%         pos_traj_new = z_new(:,1);
%         vel_traj_new = z_new(:,2);
%         vel_traj_new = [diff(u_traj_new); 0];
%         acc_traj_new = [diff(vel_traj_new); 0];
        vel_traj_new = cdiff(u_traj_new, DELTA_T);
        acc_traj_new = cdiff(vel_traj_new, DELTA_T);
        

        if STOMP_2
            figure(1)
            clf
            subplot(3,1,1) % plot all trajectories compared to original
            plot(nom_traj, 'ro')
            plot(u_traj_new, 'ko')
            hold on
            for i_K = 1:K
                plot(STOMP_traj{i_K}(:,1))
            end
            title('Noisy Trajectories')
            hold off
            
            test = A*nom_traj;
            
            subplot(3,1,2) % plot all pos vs vel
            hold on
            for i_K = 1:K
                %plot(STOMP_traj{i_K}(:, 1)) % pos
                plot(STOMP_traj{i_K}(:, 2)) % vel
            end
            plot(cdiff(u_traj_new))
%             plot(cdiff(nom_traj)./env.DELTA_T,'r-')
            title('Velocity')
            subplot(3,1,3)
            hold on
%             plot(time_array, pos_traj, 'ro', time_array, vel_traj, 'bo')
%             plot(time_array, u_traj_new, 'r*', time_array, STOMP_traj{i_K}(:,2), 'b*', time_array, STOMP_traj{i_K}(:,3), 'g*')
%             for i_K = 1:K
%                 plot(STOMP_traj{i_K}(:,3))
%             end
            plot(acc_traj_new)
            hold off
            title('Accelerations')
            pause(.1)
        end

        nom_traj = u_traj_new;
        noise_factor = noise_cooling * noise_factor;
        Q_traj = sum(STOMP_COST([u_traj_new, vel_traj_new], env.END_STATE)) + 0.5 * u_traj_new' * R * u_traj_new;
        Q_diff = abs(Q_traj - Q_prev)/Q_prev;
        if Q_diff < 0.000001  %1% decrease
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

end