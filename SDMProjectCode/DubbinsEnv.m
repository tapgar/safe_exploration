classdef DubbinsEnv
    
    properties
        ENV_NAME                        % environment name
        POINTS_IN_TRAJ                  % points in trajectory
        DELTA_T                         % time step size
        U_MAX                           % max input
        U_MIN                           % min input
        R_MIN                           % min turning radius
        START_STATE                     % state state
        END_STATE                       % end state
        CUR_STATE                       % current state
        U_NOM                           % nominal trajectory
        NUM_OF_INPUTS                   % number of inputs (u's)
        TRAJ_DIMS                       % number of dimensions
        
        f_map                           % function handle to map
        
        NOMINAL_TRAJECTORY
    end
        
    
    methods
        function obj = DubbinsEnv(f_map)
        % creates enviornment
            
            if nargin < 1
%                 f_map = @env_map;
                f_map = 0;
            end
            
            obj.f_map = f_map;   % Map is a function that returns safety in a map
            obj.POINTS_IN_TRAJ = 50;
            obj.TRAJ_DIMS = 2;
            obj.DELTA_T = 0.1;
            obj.START_STATE = [0, 0, 0, 0, 0];
            obj.END_STATE = [10, 10, pi, 0, 0];
            obj.U_MAX = [3.0, 0.1];
            obj.U_MIN = [-3.0, -0.1];
            obj.R_MIN = 0.5;
            obj.NUM_OF_INPUTS = 2;
            
            obj.ENV_NAME = 'DubbinsSimple';
            obj.NOMINAL_TRAJECTORY = false;
            
%             addpath('../Dubins-Curve-For-MATLAB-master/Dubins')
            
        end
        
        
        function obj = NominalTrajectory(obj) % Create Nominal Trajectory
%             build straight line trajectory
              path = dubins_curve(obj.START_STATE, obj.END_STATE, obj.R_MIN, 0);
              obj.U_NOM = path;
        end
        
        function [dz] = getAccel(obj, u)
%            dz = ( u - obj.b*z(2) - obj.m*obj.g*obj.l*sin(z(1))/2 ) / (obj.m*obj.l^2/3);
             dz = u(1);
        end
        
        function [z_new, unsafe] = forward(obj, z0, u)
            [z_new, unsafe] = obj.forward_tra(obj, z0, u);
            z_new = z_new(end,:); unsafe = unsafe(end);
            
        end
        
        function [z_new, fric, unsafe] = forward_traj(obj, z0, u)
        % given an inputs, initial state
        % z = [x0, y0, theta0, v0, thetad0]
        % u = [vd, thetadd]
    
            unsafe = ones(size(u, 1) + 1); % assume unsafe
            q_array = zeros(size(u, 1) + 1, 3);
            qd_array = zeros(size(u, 1) + 1, 2);
            q_array(1,:) = z0(1:3);
            qd_array(1,:) = z0(4:5);
            
            ydd_max = 3; % some arbitrary constant above which slipping occurs
            fric_S = 0.1; fric_D = 0.1;
            ydd = 0; yd = 0;
            % for each input
            for i = 2:size(u,1)+1 % initial position to one beyond end of array
                %update state
                % x_new = x_old + v_old * cos(theta_old) * dt;
                % y_new = y_old + v_old * sin(theta_old) * dt;
                % theta_new = theta_old + thetad * dt;
                
                % get forward and sideways velocity in bodyframe coordinates
                % rho = v/alpha_dot
                rho = qd_array(i - 1, 1) / qd_array(i -1, 2);
                % centripetal acceleration = v^2 / rho;
                ydd_limit = qd_array(i - 1, 1) ^ 2 / rho - sign(rho)*(fric_S + fric_D * abs(yd));
                yd  = yd + ydd * obj.DELTA_T;
                if abs(ydd_limit) > ydd_max % if above some value then slipping occurs
                    ydd = ydd_limit;
                    unsafe(i) = 0;
                elseif yd > 0.001 % if going counter clockwise around circle
                    ydd = - fric_S - fric_D * yd; % ydd should always be negative
                elseif yd < -0.001 % if going clockwise around circle
                    ydd = fric_S - fric_D * yd; % ydd shold always be positive
                else
                    ydd = 0;
                    yd = 0;
                end
                
                % DYNAMIC UPDATE %
                q_array(i, 1) = q_array(i - 1, 1) + ...
                    qd_array(i - 1, 1) * cos(q_array(i-1,3)) * obj.DELTA_T + ...
                    abs(yd) * cos(q_array(i-1,3) + sign(qd_array(i-1, 2)) * pi/2) * obj.DELTA_T;
                q_array(i, 2) = q_array(i - 1, 2) + ...
                    qd_array(i - 1, 1) * sin(q_array(i-1,3)) * obj.DELTA_T + ...
                    abs(yd) * sin(q_array(i-1,3) + sign(qd_array(i-1, 2)) * pi/2) * obj.DELTA_T;
                
                % behavior for reversing
                if qd_array(i-1, 1) >= 0
                    q_array(i, 3) = q_array(i - 1, 3) + qd_array(i - 1, 2) * obj.DELTA_T;
                else
                    q_array(i, 3) = q_array(i - 1, 3) - qd_array(i - 1, 2) * obj.DELTA_T;
                end

                % v_new = v_old + u1 * dt
                % theta_turn = thetad_old + u2 * dt
                qd_array(i, 1) = qd_array(i - 1, 1) + u(i - 1, 1) * obj.DELTA_T;
                qd_array(i, 2) = qd_array(i - 1, 2) + u(i - 1, 2) * obj.DELTA_T;
                
                % updating friction
                ydd_max = 3;
            end
            
            z_new = [q_array, qd_array];
            
            % map check
            [fric, unsafe] = obj.f_map(z_new);
%             if ~any(safe)
%                 unsafe = 0; % safe zone!
%             end
            
        end
        
        function [X, y] = generateLocalData(obj, x0, K)
           
            X = randn(K,length(x0)).*0.05;
            y = zeros(K,1);
            for i = 1:1:length(X(:,1))
                dz = obj.getAccel(X(i,1:2),X(i,3));
                y(i,1) = dz(end,1);
            end
            
        end
        
        function qdd = getTargetAccel(obj, targ, xe)
           
            qd = xe(1)/obj.DELTA_T;
            qdd = (qd - targ(2))./obj.DELTA_T;
            qdd = min(obj.maxQdd,max(-obj.maxQdd,qdd));
            
        end
            
        function [X, y] = plot(obj, traj, cov, x, policy, invGP)
           
            
            
            pts = [-0.1, 0;...
                   -0.1, obj.l;...
                   0.1, obj.l;...
                   0.1, 0;...
                   -0.1, 0];

            %%%NOTE iters=5 was because of the trajectory i was using to
            %%%test this really should be DT/dt
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
        
        function [X, y, obj] = run_sim(obj, traj, cov, x, policy, invGP, SHOWPLOT)
            
           
            iters = obj.DELTA_T/obj.dt;
            X = zeros((length(traj(:,1))-2)*iters,3);
            y = zeros(length(X),1);
            c=1;
            for n = 2:1:length(traj(:,1))-1
                
                
                p_idx = n-1;
                
                
                limiting = false;
                K = reshape(policy(p_idx,:),1,2);
                for j = 1:1:iters
                    
                    xe = (traj(n,1:2) - x)';
                    qd = xe(1)/(iters*obj.dt);
                    qdd = (qd - x(2))/(iters*obj.dt);
                    qdd = min(obj.maxQdd,max(-obj.maxQdd,qdd));
                    
                    [u, ~] = invGP.query_data_point([x,qdd]);
                    u = -K*xe + u;
                    unew = min(obj.U_MAX,max(obj.U_MIN,u));
                    if unew ~= u
                        
                        limiting = true;
                    end
                    u = unew;
                    
                    y(c,1) = obj.getAccel(x,u);
                    if abs(y(c,1)) > obj.maxQdd
                        obj.maxQdd = y(c,1);
                    end
                    X(c,:) = [x, u];
                    
                    [x, ~] = obj.forward(x,u);
                    c = c + 1;
                end
                
                if SHOWPLOT
                    figure(4)
                    clf;
                    pts = [-0.1, 0;...
                           -0.1, obj.l;...
                           0.1, obj.l;...
                           0.1, 0;...
                           -0.1, 0];
                    
                    th = traj(n,1)+pi;
                    R = [cos(th), -sin(th); sin(th), cos(th)];
                    npts = pts*R;

                    s = sqrt(cov(p_idx,1))*1.96;
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

                    s = sqrt(cov(p_idx,2))*1.96;
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

        end
        
    end
        
        
        
        
        
        
        
        
        
        
        
        
end