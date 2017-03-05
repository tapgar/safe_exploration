classdef DIEnv
    
    properties
        ENV_NAME                        % environment name
        POINTS_IN_TRAJ                  % points in trajectory
        DELTA_T                         % time step size
        U_MAX                           % max input
        U_MIN                           % min input
        START_STATE                     % state state
        END_STATE                       % end state
        CUR_STATE                       % current state
        U_NOM                           % nominal trajectory
        NUM_OF_INPUTS                   % number of inputs (u's)
        
        f_map                           % function handle to map
        
        
        % ##### PLOTTING ########## %
        NOMINAL_TRAJECTORY
        
    end
    
    methods 
        function obj = DIEnv(f_map)
        % creates enviornment
            
            if nargin < 1
                f_map = @DI_map_1;
            end
            
            obj.f_map = f_map;   % Map is a function that returns safety in a map
            obj.POINTS_IN_TRAJ = 100;
            obj.DELTA_T = 0.1;
            obj.START_STATE = [0, 0];
            obj.END_STATE = [9, 0];
            obj.U_MAX = 4;
            obj.U_MIN = -4;
            obj.NUM_OF_INPUTS = 1;
            
            
            obj.NOMINAL_TRAJECTORY = false;
            obj.ENV_NAME = 'DoubleIntegrator';
        end
        
        function obj = NominalTrajectory(obj) % Create Nominal Trajectory
            % build straight line trajectory
            u_traj = [];
            pos_traj = linspace(obj.START_STATE(1), obj.END_STATE(1), obj.POINTS_IN_TRAJ)';
%             vel_traj = linspace(obj.START_STATE(2), obj.END_STATE(2), obj.POINTS_IN_TRAJ)';
%             for i_traj = 2:length(pos_traj) - 1
%                 start_traj = pos_traj(i_traj);
%                 end_traj = pos_traj(i_traj + 1);
%                 vel_start = vel_traj(i_traj); 
%                 vel_traj(i_traj) = (end_traj - start_traj) * 2 / obj.DELTA_T - vel_start;
%                 if i_traj == 2
%                     vel_traj(i_traj) = vel_traj(i_traj)/2; % cheating to make it not go full speed and then slow down
%                 end
%             end  
%             obj.U_NOM = obj.u_trans(pos_traj, vel_traj);
            obj.U_NOM = pos_traj;
%             
            
            if obj.NOMINAL_TRAJECTORY
                figure()
                plot(pos_traj, 'ro')
                hold on
                plot(vel_traj, 'bo')
                hold off
                title('Nominal Trajectory')
            end
        end
        
        function [z_new, unsafe] = forward(obj, z, u)
            b = 0.01; % [s*Nm/rad] friction coefficient
            unsafe = 1; % assume unsafe

            % z = [theta, theta_dot]

            dz = zeros(size(z));
            dz(1) = z(2);
            dz(2) = u - b*z(2);

            z_new = z + dz * obj.DELTA_T;

            % map check - map can be passed as argument to env_pendulum function
            if ~obj.f_map(z(1), z_new(1))
                unsafe = 0; % safe zone!
            end
        end
        
        function [z_new, unsafe] = forward_traj(obj, z0, u)
            % given a set of inputs, finding the output physical positions
            unsafe = zeros(obj.POINTS_IN_TRAJ, 1);
            z_new = zeros(obj.POINTS_IN_TRAJ, 2);
            z_new(1,:) = z0;
            for i_PIJ = 2:obj.POINTS_IN_TRAJ
                [z, unsafe(i_PIJ)] = obj.forward(z_new(i_PIJ-1,:), u(i_PIJ, :));
                z_new(i_PIJ,:) = z;
            end
        end
            
        function u = u_trans(obj, xs, x_dots) %input needed to transition between states
            b = 0.01; % [s*Nm/rad] friction coefficient
            
            u = zeros(length(xs), 1);
            d_x = circshift(xs,1) - xs;
            avg_x = (xs + circshift(xs,1)) ./ 2;
            avg_x_dots = (x_dots + circshift(x_dots,1)) ./2;
            for i = 2:length(xs)
                u(i) = -u(i-1) + 2/obj.DELTA_T^2 * (d_x(i) - x_dots(i) * obj.DELTA_T);
            end
        end
            
            
    end
end