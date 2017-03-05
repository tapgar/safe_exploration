classdef PendulumEnv
    
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
        
        
        m
        l
        g
        b
        dt
        
        maxQdd
        
        % ##### PLOTTING ########## %
        NOMINAL_TRAJECTORY = false;
        
    end
    
    methods 
        function obj = PendulumEnv(f_map)
        % creates enviornment
            
            if nargin < 1
                f_map = @env_map;
            end
            
            obj.f_map = f_map;   % Map is a function that returns safety in a map
            obj.POINTS_IN_TRAJ = 100;
            obj.DELTA_T = 0.1;
            obj.START_STATE = [0, 0];
            obj.END_STATE = [pi, 0];
            obj.U_MAX = 2.5;
            obj.U_MIN = -2.5;
            obj.NUM_OF_INPUTS = 1;
            
            obj.l = 1;
            obj.m = 1;
            obj.g = 9.806;
            obj.b = 0.01;
            obj.dt = 0.01;
            obj.maxQdd = 1;
            
            
            obj.NOMINAL_TRAJECTORY = false;
            obj.ENV_NAME = 'Pendulum';
        end
        
        function obj = NominalTrajectory(obj) % Create Nominal Trajectory
            % build straight line trajectory
            pos_traj = linspace(obj.START_STATE(1), obj.END_STATE(1), obj.POINTS_IN_TRAJ)';
            vel_traj = linspace(obj.START_STATE(2), obj.END_STATE(2), obj.POINTS_IN_TRAJ)';
            for i_traj = 2:length(pos_traj) - 1
                start_traj = pos_traj(i_traj);
                end_traj = pos_traj(i_traj + 1);
                vel_start = vel_traj(i_traj-1); 
                vel_traj(i_traj) = (end_traj - start_traj) * 2 / obj.DELTA_T - vel_start;
                if i_traj == 2
                    vel_traj(i_traj) = vel_traj(i_traj)/2; % cheating to make it not go full speed and then slow down
                end
            end  
            obj.U_NOM = obj.u_trans(pos_traj, vel_traj);
            
            
            if obj.NOMINAL_TRAJECTORY
                figure()
                plot(pos_traj, 'ro')
                hold on
                plot(vel_traj, 'bo')
                hold off
                title('Nominal Trajectory')
            end
        end
        
        function [dz] = getAccel(obj, z, u)
           dz = ( u - obj.b*z(2) - obj.m*obj.g*obj.l*sin(z(1))/2 ) / (obj.m*obj.l^2/3); 
        end
        
        function [z_new, unsafe] = forward(obj, z, u)
            unsafe = 1; % assume unsafe

            % z = [theta, theta_dot]

            dz = zeros(size(z));
            dz(1) = z(2);
            dz(2) = obj.getAccel(z,u);
%             dz(2) = ( u - m*g*l*sin(z(1))/2 ) / (m*l^2/3);

            z_new = z + dz * obj.dt;
            %z_new(1) = mod(z_new(1), 2*pi);

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
        
        function [X, y] = generateLocalData(obj, x0, K)
           
            X = randn(K,length(x0)).*0.05;
            y = zeros(K,1);
            for i = 1:1:length(X(:,1))
                dz = obj.getAccel(X(i,1:2),X(i,3));
                y(i,1) = dz(end,1);
            end
            
        end
            
            
        function [X, y] = plot(obj, traj, cov, x, policy, invGP)
           
            
            
            pts = [-0.1, 0;...
                   -0.1, obj.l;...
                   0.1, obj.l;...
                   0.1, 0;...
                   -0.1, 0];

            iters = 5;
            X = zeros((length(traj(:,1))-1)*iters,3);
            y = zeros(length(X),1);
            c=1;
            for i = 1:1:length(traj(:,1))-1
                limiting = false;
                K = reshape(policy(i,:),1,2);
                for j = 1:1:iters
                    xe = (traj(i,1:2) - x)';
                    qd = xe(1)/(iters*obj.dt);
                    qdd = (qd - x(2))/(iters*obj.dt);
                    qdd = min(obj.maxQdd,max(-obj.maxQdd,qdd));
                    [u, ~] = invGP.query_data_point([x,qdd]);
                    u = -K*xe + u;
                    unew = min(obj.U_MAX,max(obj.U_MIN,u));
                    if unew ~= u
                        u = unew;
                        limiting = true;
                    end
                    y(c,1) = obj.getAccel(x,u);
                    if abs(y(c,1)) > obj.maxQdd
                        obj.maxQdd = y(c,1);
                    end
                    X(c,:) = [x, u];
                    
                    [x, ~] = obj.forward(x,u);
                    c = c + 1;
                end
                th = traj(i,1)+pi;
                R = [cos(th), -sin(th); sin(th), cos(th)];
                npts = pts*R;

                s = sqrt(cov(i,1))*1.96;
                ths = linspace(-s+th,s+th,100)';
                cpts = zeros(length(ths),2);
                for p = 1:1:length(ths)
                    R = [cos(ths(p)), -sin(ths(p)); sin(ths(p)), cos(ths(p))];
                    r = [0; obj.l];
                    cpts(p,:) = (r'*R);
                end
                plot(npts(:,1),npts(:,2));
                hold on
                plot(cpts(:,1),cpts(:,2),'r');
                
                s = sqrt(cov(i,2))*1.96;
                ths = [0, -s + obj.l; 0, s + obj.l];
                R = [cos(th), -sin(th); sin(th), cos(th)];
                cpts = ths*R;
                plot(cpts(:,1),cpts(:,2),'r');
                
                
                th = x(1,1)+pi;
                R = [cos(th), -sin(th); sin(th), cos(th)];
                npts = pts*R;
                if ~limiting
                    plot(npts(:,1),npts(:,2),'g');
                else
                    plot(npts(:,1),npts(:,2),'r');
                end
                xlim([-2.5, 2.5])
                ylim([-1.2, 1.2])
                pause(0.1)
                hold off

            end
            
        end
        
        function u = u_trans(obj, thetas, theta_dots) %input needed to transition between states           
%             diff_thetas = diff(theats);
%             diff_theta_dots = diff(theta
            u = zeros(length(thetas), 1);
            for i = 2:length(thetas)
                d_theta = thetas(i) - thetas(i-1);
                avg_theta = thetas(i) + thetas(i-1);
                avg_theta_dot = theta_dots(i) + theta_dots(i-1);
                u(i) = -u(i-1) + 2/obj.DELTA_T^2 * (d_theta - 0.5 * avg_theta_dot * obj.DELTA_T) + obj.g * sin(avg_theta);
            end
        end
            
            
    end
end