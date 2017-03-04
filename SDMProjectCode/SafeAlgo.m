function SafeAlgo(env)
    close all
    
    if nargin < 1
        env = PendulumEnv(@env_map);
    end
    
    %#######################################
    %################ TODO #################
    %#######################################
    % change environment to class that has map as input
        % return things like 
            % Nominal Trajectory
            % points in trajectory
            %
            
    % roll out
        % make rollout a subfunction so can call it recursively
        % weird error as it gets closer to end.
    
    % post loop stuff
    
    % double check coordinate system for pendulum and signs in equations
    
    % how to get point of maximum info gain?
        % KL distance
    
    %#######################################
    %############## CONSTANTS ##############
    %#######################################
    K = 10;                         % noisy trajectories for STOMP
    STEPS = 10;                     % number of steps to linearize for LQG
    
    SAFETY_THRESHOLD = 0.95;        % above this value a policy is considered safe
    U_MAX = env.U_MAX;                      % max input
    POINTS_IN_TRAJ = env.POINTS_IN_TRAJ;           % points in trajectory
    DELTA_T = env.DELTA_T;                         % time step size
    NUM_OF_STATES = 2;              % number of state variables
    NUM_OF_INPUTS = 1;              % number of input variabls (u's)
    ROLLOUTS = 10;                  % Number of imaginary rollouts
    SAFETY_THRESHOLD = 0.95;        % Threshold above which to accept trajectory
    
    
    %###########################################
    %############## VISUALIZATION ##############
    %###########################################
    ROLLOUT_GRAPHS = true;
    
    %#######################################
    %############## VARIABLES ##############
    %#######################################
    cur_state = env.CUR_STATE;                              % current state
    end_state = env.END_STATE;                             % goal state
    max_vel = 4;
    min_vel = -4;
    
    
    
    
    
    
    
    
    % ########## NOMINAL TRAJECTORY ############# %
    time_array = DELTA_T * linspace(1, POINTS_IN_TRAJ);
    env = env.NominalTrajectory();
   
    
    % ########## STOMP ############# %
    u_stomp = STOMP(env, 10);
    
    % ########## END STOMP ############# %
    
    
    
    
    % ########## Build Feedback Controller -- LQG ################ %
            % linear GP about steps

            % add points while building K matrices
            % other matrices
    
    %PD Controller
    Kp = 100; Kd = 1;
    controller = @(e, e_dot) Kp * e + Kd * e_dot;
    
    % ############# Imagination Rollouts ################ %
    % this should be easily moved into a seperate function
    % should return a value of safety which corresponds to the given
    % trajectory??
    addpath('../OLGP')
    hp = struct('y_std',0.1027,'sig_std',3.9978,'W',eye(3));
    model = LocalGP(200, 30, 0.8, hp);
    p_total = 0;
    commanded = zeros(POINTS_IN_TRAJ, NUM_OF_STATES);
    actual = zeros(POINTS_IN_TRAJ, NUM_OF_STATES*3/2);
    GP_vals = zeros(POINTS_IN_TRAJ, 4); % [mean, Vbef, Vafter, Vgain]
    e = commanded; e_dot = commanded;
    commanded = u_stomp(:,2:3);
    u_rollout = zeros(POINTS_IN_TRAJ, 1);
    model = model.add_training_data([0,0,0], 0); % have to add a point before querrying
    actual(1,1:2) = commanded(1,:);
    for i_PIJ_real = 2:POINTS_IN_TRAJ
        p_safe = zeros(ROLLOUTS,1);
        fprintf('Point: %.f ', i_PIJ_real)
        for i_ro = 1:ROLLOUTS
            model2 = model;
            p_total = 0;
            for i_PIJ = i_PIJ_real:POINTS_IN_TRAJ
                e(i_PIJ) = [commanded(i_PIJ, 1) - actual(i_PIJ-1,1)];
                e_dot(i_PIJ) = [commanded(i_PIJ, 2) - actual(i_PIJ-1,2)];
                u_rollout(i_PIJ) = controller(e(i_PIJ), e_dot(i_PIJ));
                % Sample at each time step
                [a, b] = model2.query_data_point([actual(i_PIJ-1,1:2), u_rollout(i_PIJ)]);
                [GP_vals(i_PIJ,1), GP_vals(i_PIJ,2)] = model2.query_data_point([actual(i_PIJ-1,1:2), u_rollout(i_PIJ)]);
                actual(i_PIJ,3) = GP_vals(i_PIJ, 1) + randn(1) * GP_vals(i_PIJ, 2);
                
                actual(i_PIJ,2) = actual(i_PIJ-1,2) + DELTA_T * actual(i_PIJ,3); %velocity update
                actual(i_PIJ,1) = actual(i_PIJ-1,1) + DELTA_T * (actual(i_PIJ,2) + actual(i_PIJ-1,2))/2; % position update, velocity is taken as average of old and new
                model2 = model2.add_training_data([actual(i_PIJ-1,1:2), u_rollout(i_PIJ)], actual(i_PIJ,3));
                [~, GP_vals(i_PIJ, 3)] = model2.query_data_point([actual(i_PIJ-1,1:2), u_rollout(i_PIJ)]);
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
        fprintf('From Point %.f, Rollout %.f, Steps %.f: %.3f \n', i_PIJ_real, i_ro, i_PIJ-i_PIJ_real, p_safe(i_ro))
        end
        if ROLLOUT_GRAPHS
            hold on
            plot(actual(:,1),'color',rand(1,3),'marker','o')
            plot(commanded(:,1),'bo')
            plot(actual(:,2),'r*')
            plot(commanded(:,2),'b*')
            title(fprintf('Point in Trajectory: %f', i_PIJ_real))
            pause(0.1)
        end
    end
    
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

end