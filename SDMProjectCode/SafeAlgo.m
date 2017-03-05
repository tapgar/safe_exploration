function SafeAlgo(env)
    close all
    
    if nargin < 1
        env = PendulumEnv();
    end
    
    %#######################################
    %################ TODO #################
    %#######################################

    % STOMP is definitely not working
        % doesn't work with DI
            
    % roll out
        % make rollout a subfunction so can call it recursively
        % weird error as it gets closer to end.
    
    % post loop stuff
    
    % double check coordinate system for pendulum and signs in equations
    
    
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
    H = 10;                         % STOMP regularizing term
    ROLLOUTS = 10;                  % Number of imaginary rollouts
    SAFETY_THRESHOLD = 0.95;        % Threshold above which to accept trajectory
    
    
    %###########################################
    %############## VISUALIZATION ##############
    %###########################################
    VIS_ROLLOUTS = false;
    
    
    %#######################################
    %############## VARIABLES ##############
    %#######################################
    cur_state = [0, 0];                              % current state
    end_state = [pi, 0];                             % goal state
    

    time_array = DELTA_T * linspace(1, POINTS_IN_TRAJ);
   % ########## NOMINAL TRAJECTORY ########## %
   env = env.NominalTrajectory();
    
    % ########## STOMP ############# %
    u_stomp = STOMP(env, 10);
    
    % ######## CONTROLLER ######### %
        % Build Feedback Controller -- LQG
            % linear GP about steps

            % add points while building K matrices
            % other matrices
            
    %PD Controller
    Kp = 100; Kd = 1;
    controller = @(e, e_dot) Kp * e + Kd * e_dot;

    % ############# Imagination Rollouts ################ %
    DELTA_T;
    % Build matrices - done as precompute step
    addpath('../OLGP')
    hp = struct('y_std',0.1027,'sig_std',3.9978,'W',eye(3));
    model = LocalGP(200, 30, 0.8, hp);
    p_total = 0;
    commanded = zeros(POINTS_IN_TRAJ, NUM_OF_STATES);
    actual = zeros(POINTS_IN_TRAJ, NUM_OF_STATES*3/2);
    e = commanded; e_dot = commanded;
    commanded = u_stomp(:,2:3);
    u_rollout = u_stomp(:,1);
    model = model.add_training_data([0,0,0], 0);
    actual(1,1:2) = commanded(1,:);
    [p_safe] = imaginary_ro(controller, DELTA_T, model, commanded, actual, POINTS_IN_TRAJ, ROLLOUTS, u_rollout, VIS_ROLLOUTS);
    % cost of trajectory
    % ############# END Imagination Rollouts ################ %
        
        
    % Choose Policy - no choosing. policy is either good or not?
 

    % Evaluate Safety of POlicy
    % if above threshold -> return policy
%     if  mean(p_safe(:)) > SAFETY_THRESHOLD
% %         output = model;
%           hello = 1;
%     else
%         hello = 1;
%     end
    % find point with highest info gain
    
    % create trajectory x0 -> xi -> x0 (probably just use STOMP again)
    
    
    
    % update f_cost based on objective (change it somehow)



    function c = S(state) %this should be part of env?
        pos = state(1);
        vel = state(2);

        c = -(end_state(1) - pos)^2 - (end_state(2) - vel)^2; 
    end


end