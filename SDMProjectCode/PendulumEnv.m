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
        V_NOM
        t_NOM
        NUM_OF_INPUTS                   % number of inputs (u's)
        TRAJ_DIMS                       % number of dimensions
        
        f_map                           % function handle to map
        
        
        m
        l
        g
        b
        dt
        
        maxQdd
        
        dircolPts
        
        x_start
        v_start
        u_start
        
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
            obj.POINTS_IN_TRAJ = 60;
            obj.TRAJ_DIMS = 1;
            obj.DELTA_T = 0.1;
            obj.START_STATE = [0, 0];
            obj.END_STATE = [pi, 0];
            obj.U_MAX = 2.0;
            obj.U_MIN = -2.0;
            obj.NUM_OF_INPUTS = 1;
            
            obj.l = 1;
            obj.m = 1;
            obj.g = 9.806;
            obj.b = 0.01;
            obj.dt = 0.01;
            obj.maxQdd = 1;
            
            
            
            obj.dircolPts = 20;
            obj.x_start = linspace(obj.START_STATE(1),obj.END_STATE(1),obj.dircolPts)';
            obj.v_start = linspace(obj.START_STATE(2),obj.END_STATE(2),obj.dircolPts)';
            obj.u_start = ones(obj.dircolPts, 1)*obj.U_MAX;
            
            
            
            obj.NOMINAL_TRAJECTORY = false;
            obj.ENV_NAME = 'Pendulum';
        end
        
        function obj = NominalTrajectory_old(obj) % Create Nominal Trajectory
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
        
        function obj = NominalTrajectory(obj, GP) % Create Nominal Trajectory
            % build straight line trajectory
%             pos_traj = linspace(obj.START_STATE(1), obj.END_STATE(1), obj.POINTS_IN_TRAJ)';
%             obj.U_NOM = pos_traj;
%             load pendulumData
%             obj.U_NOM = X(1:8:end-1,1);

              traj = obj.DirColTrajectory(GP);
              obj.U_NOM = interp(traj(2:1+obj.dircolPts),obj.POINTS_IN_TRAJ/obj.dircolPts);
              obj.V_NOM = interp(traj(2+obj.dircolPts:1+obj.dircolPts*2),obj.POINTS_IN_TRAJ/obj.dircolPts);
              obj.t_NOM = interp(traj(2+obj.dircolPts*2:1+obj.dircolPts*3),obj.POINTS_IN_TRAJ/obj.dircolPts);
              
              obj.DELTA_T = traj(1)/obj.POINTS_IN_TRAJ;
              obj.x_start = traj(2:obj.dircolPts+1);
              obj.v_start = traj(obj.dircolPts+2:1+obj.dircolPts*2);
              obj.u_start = traj(obj.dircolPts*2+2:1+obj.dircolPts*3);
        end
        
        function optimal = DirColTrajectory(obj, GP)
           
            gridN = obj.dircolPts;

            time_min = @(x) (x(1) + 2.*sum(x(2 + gridN * 3 : end).^2));

            % The initial parameter guess; 1 second, gridN positions, gridN velocities,
            % gridN accelerations
            x0 = [10;...
                  obj.x_start;...
                  obj.v_start;...
                  obj.u_start;...
                  zeros(gridN, 1)];

            % No linear inequality or equality constraints
            A = [];
            b = [];
            Aeq = [];
            Beq = [];

            maxU = obj.U_MAX;

            f = @(x)pendulum_constraints(x, GP, gridN);


            lb = [0;    ones(gridN * 2, 1) * -Inf;  ones(gridN, 1) * -maxU;    ones(gridN * 1, 1) * -Inf];
            ub = [Inf;  ones(gridN * 2, 1) * Inf;   ones(gridN, 1) * maxU;    ones(gridN * 1, 1) * Inf];
            % Options for fmincon
            options = optimoptions(@fmincon, 'TolFun', 0.00000001, 'MaxIter', 10000, ...
                                   'MaxFunEvals', 100000, 'Display', 'iter', ...
                                   'DiffMinChange', 0.001, 'Algorithm', 'sqp');
            % Solve for the best simulation time + control input
            optimal = fmincon(time_min, x0, A, b, Aeq, Beq, lb, ub, ...
                          f, options);
            
            
            
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
        
        function [GP, invGP, obj] = run_sim(obj, traj, cov, x, policy, GP, invGP, SHOWPLOT)
            
           
            iters = round(obj.DELTA_T/obj.dt);
            obj.dt = obj.DELTA_T/iters;
            X = zeros((length(traj(:,1))-2)*iters,3);
            y = zeros(length(X),1);
            c=1;
            for n = 2:1:length(traj(:,1))-1
                
                
                p_idx = n-1;
                
                
                limiting = false;
                K = reshape(policy(p_idx,:),1,2);
                for j = 1:1:iters
                    
                    xe = (traj(n,1:2) - x)';
                    qd = xe(1)/(obj.DELTA_T);
                    qdd = (qd - x(2))/(obj.DELTA_T);
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
                    
                    GP = GP.add_training_data(X(c,:),y(c,1));
                    invGP = invGP.add_training_data([X(c,1:2), y(c,1)], X(c,3));
                    
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