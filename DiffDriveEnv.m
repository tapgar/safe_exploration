classdef DiffDriveEnv
    
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
        NUM_OF_INPUTS                   % number of inputs (u's)
        TRAJ_DIMS                       % number of dimensions       
        QDD_MAX                         % max accelerations
        QDD_MIN                         % min accelerations
        QD_MAX                          % max velocities
        QD_MIN                          % min velocities
        f_map                           % function handle to map
        
        U_NOM                           % nominal trajectory
        V_NOM
        t_NOM
        
        dircolPts
        
        x_start
        v_start
        u_start
        
        dt
        
        bot
        
        NOMINAL_TRAJECTORY
    end
        
    
    methods
        function obj = DiffDriveEnv(f_map)
        % creates enviornment
            
            if nargin < 1
%                 f_map = @env_map;
                f_map = 0;
            end
            
            obj.f_map = f_map;   % Map is a function that returns safety in a map
            obj.POINTS_IN_TRAJ = 50;
            obj.TRAJ_DIMS = 2;
            obj.DELTA_T = 0.1;
            obj.START_STATE = [0.5, 0.5, 0, 0, 0, 0];
            obj.END_STATE = [6.5, 9, -pi, 0, 0, 0];
            obj.U_MAX = [150, 150];
            obj.U_MIN = [-150, -150];
            obj.QDD_MAX = [2.5, 1.0, 3];
            obj.QDD_MIN = [-2.5, -1.0, -3];
            obj.QD_MAX = [5, 1.5];
            obj.QD_MIN = [-5, -1.5];
            obj.R_MIN = 0.5;
            obj.NUM_OF_INPUTS = 2;
            
            obj.dircolPts = 25;
            
            obj.x_start = [linspace(obj.START_STATE(1),obj.END_STATE(1),obj.dircolPts)';...
                           linspace(obj.START_STATE(2),obj.END_STATE(2),obj.dircolPts)';...
                           linspace(obj.START_STATE(3),obj.END_STATE(3),obj.dircolPts)'];
            obj.v_start = [linspace(obj.START_STATE(4),obj.END_STATE(4),obj.dircolPts)';...
                           linspace(obj.START_STATE(5),obj.END_STATE(5),obj.dircolPts)';...
                           linspace(obj.START_STATE(6),obj.END_STATE(6),obj.dircolPts)'];
            
            obj.u_start = zeros(obj.dircolPts*2,1);
            
            traj = [0; obj.x_start; obj.v_start; obj.u_start]';
            obj.U_NOM = [interp(traj(2:1+obj.dircolPts),obj.POINTS_IN_TRAJ/obj.dircolPts)';...
                         interp(traj(2+obj.dircolPts:1+obj.dircolPts*2),obj.POINTS_IN_TRAJ/obj.dircolPts)';...
                         interp(traj(2+obj.dircolPts*2:1+obj.dircolPts*3),obj.POINTS_IN_TRAJ/obj.dircolPts)'];
            obj.V_NOM = [interp(traj(2+obj.dircolPts*3:1+obj.dircolPts*4),obj.POINTS_IN_TRAJ/obj.dircolPts)';...
                         interp(traj(2+obj.dircolPts*4:1+obj.dircolPts*5),obj.POINTS_IN_TRAJ/obj.dircolPts)';...
                         interp(traj(2+obj.dircolPts*5:1+obj.dircolPts*6),obj.POINTS_IN_TRAJ/obj.dircolPts)'];
            
            obj.t_NOM = [interp(traj(2+obj.dircolPts*6:1+obj.dircolPts*7),obj.POINTS_IN_TRAJ/obj.dircolPts)';...
                         interp(traj(2+obj.dircolPts*7:1+obj.dircolPts*8),obj.POINTS_IN_TRAJ/obj.dircolPts)'];
                       
            
            obj.ENV_NAME = 'DiffDrive';
            obj.NOMINAL_TRAJECTORY = false;
            
            obj.bot = load_bot_parameters();
            
            obj.dt = 0.01;
%             addpath('../Dubins-Curve-For-MATLAB-master/Dubins')
            
        end
        
        
        function obj = NominalTrajectory_old(obj) % Create Nominal Trajectory
%             build straight line trajectory
              path = dubins_curve(obj.START_STATE, obj.END_STATE, obj.R_MIN, 0);
              obj.U_NOM = path;
        end
        
        function obj = NominalTrajectory(obj, GP, icyGP) % Create Nominal Trajectory
            % build straight line trajectory
%             pos_traj = linspace(obj.START_STATE(1), obj.END_STATE(1), obj.POINTS_IN_TRAJ)';
%             obj.U_NOM = pos_traj;
%             load pendulumData
%             obj.U_NOM = X(1:8:end-1,1);

              traj = obj.DirColTrajectory(GP, icyGP);
              obj.U_NOM = [interp(traj(2:1+obj.dircolPts),obj.POINTS_IN_TRAJ/obj.dircolPts)';...
                           interp(traj(2+obj.dircolPts:1+obj.dircolPts*2),obj.POINTS_IN_TRAJ/obj.dircolPts)';...
                           interp(traj(2+obj.dircolPts*2:1+obj.dircolPts*3),obj.POINTS_IN_TRAJ/obj.dircolPts)'];
              obj.V_NOM = [interp(traj(2+obj.dircolPts*3:1+obj.dircolPts*4),obj.POINTS_IN_TRAJ/obj.dircolPts)';...
                           interp(traj(2+obj.dircolPts*4:1+obj.dircolPts*5),obj.POINTS_IN_TRAJ/obj.dircolPts)';...
                           interp(traj(2+obj.dircolPts*5:1+obj.dircolPts*6),obj.POINTS_IN_TRAJ/obj.dircolPts)'];
            
              obj.t_NOM = [interp(traj(2+obj.dircolPts*6:1+obj.dircolPts*7),obj.POINTS_IN_TRAJ/obj.dircolPts)';...
                           interp(traj(2+obj.dircolPts*7:1+obj.dircolPts*8),obj.POINTS_IN_TRAJ/obj.dircolPts)'];
              
              obj.DELTA_T = traj(1)/obj.POINTS_IN_TRAJ;
              obj.x_start = reshape(traj(2:1+obj.dircolPts*3),obj.dircolPts*3,1);
              obj.v_start = reshape(traj(obj.dircolPts*3+2:1+obj.dircolPts*6),obj.dircolPts*3,1);
              obj.u_start = reshape(traj(obj.dircolPts*6+2:1+obj.dircolPts*8),obj.dircolPts*2,1);
        end
        
        function obj = trajOptfromSTOMP(obj)
           
           obj.x_start(1:obj.dircolPts*2,1) = [decimate(obj.U_NOM(1,:),obj.POINTS_IN_TRAJ/obj.dircolPts)';...
                                               decimate(obj.U_NOM(2,:),obj.POINTS_IN_TRAJ/obj.dircolPts)'];
        end
        
        function optimal = DirColTrajectory(obj, GP, icyGP)
           
            gridN = obj.dircolPts;

            time_min = @(x) (x(1) + 0.1.*sum(sum(x(2 + gridN * 8 : end).^2)));

            % The initial parameter guess; 1 second, gridN positions, gridN velocities,
            % gridN accelerations
            x0 = [10;...
                  obj.x_start;...
                  obj.v_start;...
                  obj.u_start;...
                  zeros(gridN*2,1)];

            % No linear inequality or equality constraints
            A = [];
            b = [];
            Aeq = [];
            Beq = [];


            f = @(x)diff_drive_constraints(x, GP, icyGP, gridN, @check_surf_type);

            lb = [0;...
                  zeros(gridN, 1);... %x
                  zeros(gridN, 1);... %y
                  -ones(gridN,1)*Inf;... %yaw
                  ones(gridN,1)*obj.QD_MIN(1);...
                  ones(gridN,1)*obj.QD_MIN(1);...
                  ones(gridN,1)*obj.QD_MIN(2);...
                  ones(gridN, 1) * obj.U_MIN(1);...
                  ones(gridN, 1) * obj.U_MIN(2);...
                  ones(gridN,1)*obj.U_MIN(1);...
                  ones(gridN,1)*obj.U_MIN(2)];
              
            ub = [Inf;...
                ones(gridN, 1).*10;...
                ones(gridN, 1).*10;...
                ones(gridN,1)*Inf;...
                ones(gridN,1)*obj.QD_MAX(1);...
                ones(gridN,1)*obj.QD_MAX(1);...
                ones(gridN,1)*obj.QD_MAX(2);...
                ones(gridN, 1) * obj.U_MAX(1);...
                ones(gridN, 1) * obj.U_MAX(2);...
                ones(gridN,1)*obj.U_MAX(1);...
                ones(gridN,1)*obj.U_MAX(2)];
            
            % Options for fmincon
            options = optimoptions(@fmincon, 'TolFun', 0.0001, 'MaxIter', 10000, ...
                                   'MaxFunEvals', 100000, 'Display', 'iter', ...
                                   'DiffMinChange', 0.01, 'Algorithm', 'sqp');
            % Solve for the best simulation time + control input
            optimal = fmincon(time_min, x0, A, b, Aeq, Beq, lb, ub, ...
                          f, options);
            
            
        end
        
        function [dz] = getAccel(obj,z,u)
                           
            xd = z(4);  %linear velocity
            yd = z(5);  %slip velocity
            w = z(6); %angular velocity
            
            if (check_surf_type(z(1:2)) == 0) %ice
                static = 0.01;
                dynamic = 0.01;
                max_force = 10;
            else
                static = obj.bot.TracStaticMu;
                dynamic = obj.bot.TracDynamicMu;
                max_force = 150;
            end
            
            [xdd,ydd,tdd] = diff_drive_dynamics(obj.bot, xd, yd, w, u(1), u(2), dynamic, static, max_force);            
            dz = [xdd,ydd,tdd];
        end
        
        function [z_new, unsafe] = forward(obj, z0, u)
            [z_new, unsafe] = obj.forward_tra(obj, z0, u);
            z_new = z_new(end,:); unsafe = unsafe(end);
            
        end
        
        function [z_new] = forward_dyn(obj, z, u)
           
            x = z(1);   %global x
            y = z(2);   %global y
            psi = z(3); %angle
            xd = z(4);  %linear velocity
            yd = z(5);  %slip velocity
            w = z(6); %angular velocity
            
            accel = obj.getAccel(z,u);            
            tdd = accel(3);
            xdd = accel(1);
            ydd = accel(2);
            
            w = w + tdd*obj.dt;
            xd = xd + xdd*obj.dt;
            yd = yd + ydd*obj.dt;
            
            psi = psi + w*obj.dt;
            
            vx = xd*cos(psi) - yd*sin(psi);
            vy = xd*sin(psi) + yd*cos(psi);
            
            x = x + vx*obj.dt;
            y = y + vy*obj.dt;
            
            z_new = [x;y;psi;xd;yd;w];
            
        end
        
        function [z_new, fric, unsafe] = forward_traj(obj, z0, u)
            clip = @(in, ub, lb) min(max(in, lb), ub);

        % given an inputs, initial state
        % z = [x0, y0, theta0, v0, thetad0]
        % u = [vd, thetadd]
    
            unsafe = ones(size(u, 1) + 1); % assume unsafe
            q_array = zeros(size(u, 1) + 1, 3);
            qd_array = zeros(size(u, 1) + 1, 2);
            q_array(1,:) = z0(1:3);
            qd_array(1,:) = z0(4:5);
            
            ydd_max = 3; % some arbitrary constant above which slipping occurs
            ydd = 0; yd = 0;
            
            
            % for each input
            for i = 2:size(u,1)+1 % initial position to one beyond end of array
                %update friction - both friction values can come from env
                %if we want
                [s, ~] = obj.f_map(q_array(i-1,1:2));
                fric_S = s; fric_D = 0.1 * fric_S;
                %update state
                % x_new = x_old + v_old * cos(theta_old) * dt;
                % y_new = y_old + v_old * sin(theta_old) * dt;
                % theta_new = theta_old + thetad * dt;
                
                
                % SLIPPING DYNAMICS
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
                % clipping accelerations
                eff_acc = clip((u(i - 1, 1) - sign(qd_array(i-1,1)) * fric_S), obj.QDD_MAX(1), obj.QDD_MIN(1));
                eff_turn = clip(u(i - 1, 2), obj.QDD_MAX(2), obj.QDD_MIN(2));
                
                qd_array(i, 1) = qd_array(i - 1, 1) + eff_acc * obj.DELTA_T;
                qd_array(i, 2) = qd_array(i - 1, 2) + eff_turn * obj.DELTA_T;
                
                
                % clip some velocity - if on ice, then can go as fast as
                % you want ?? -- at max accerlation should the damping
                % coefficient begin to dominate?
                if fric_S > 0.1
                    qd_array(i,1) = clip(qd_array(i,1), obj.QD_MAX(1), obj.QD_MIN(1));
                end
                qd_array(i,2) = clip(qd_array(i,2), obj.QD_MAX(2), obj.QD_MIN(2));
                
                
            end
            
            z_new = [q_array, qd_array];
            
            % map check
            [fric, unsafe] = obj.f_map(z_new);

            
        end
        
        function [X, y] = generateLocalData(obj, x, u, K)
           
            X = zeros(K,length(x));
            X(:,[4,6]) = randn(K,2).*0.05;
            U = randn(K,length(u)).*10;
            y = zeros(K,3);
            for i = 1:1:length(X(:,1))
                dz = obj.getAccel(X(i,:),U(i,:));
                y(i,:) = dz';
            end
            X = [X,U];
        end
        
        function qdd = getTargetAccel(obj, targ, xe)
           
            qd = xe(1)/obj.DELTA_T;
            qdd = (qd - targ(2))./obj.DELTA_T;
            qdd = min(obj.maxQdd,max(-obj.maxQdd,qdd));
            
        end
        
        function col = collision(obj, p1, p2)
           
            col = 0;
            for i = 1:1:10
                x = p1(1) + (p2(1) - p1(1))*i/10;
                y = p1(2) + (p2(2) - p1(2))*i/10;
                surfType = check_surf_type([x,y]);
                if (surfType == 2)
                    col = 1;
                    return;
                end
            end
            
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
        
        function [GP, invGP, iceGP, inviceGP, obj] = run_sim(obj, traj, cov, x, policy, GP, invGP, iceGP, inviceGP, SHOWPLOT, run_ff)
            
           
            iters = round(obj.DELTA_T/obj.dt);
            obj.dt = obj.DELTA_T/iters;
            X = zeros((length(traj(:,1))-2)*iters,8);
            y = zeros(length(X),3);
            
            
            c=obj.POINTS_IN_TRAJ+1;

            psi_targ = zeros(obj.POINTS_IN_TRAJ+2,1);
            psi_targ(end,1) = pi;
            for n = length(traj(:,1))-1:-1:2
                %global vels
                qd_targ = (traj(n+1,1:obj.TRAJ_DIMS)' - traj(n-1,1:obj.TRAJ_DIMS)')./(2.*obj.DELTA_T);
                psi_targ(c,1) = atan2(qd_targ(2),qd_targ(1));
                c=c-1;
            end
            
            c=1;
            for n = 2:1:length(traj(:,1))-1
                
                
                %global accels
                qdd_targ = ([traj(n-1,1:obj.TRAJ_DIMS),psi_targ(n-1,1)]' - 2.*[traj(n,1:obj.TRAJ_DIMS),psi_targ(n,1)]' + [traj(n+1,1:obj.TRAJ_DIMS),psi_targ(n+1,1)]')./(obj.DELTA_T^2);
                %global vels
                qd_targ = ([traj(n+1,1:obj.TRAJ_DIMS),psi_targ(n+1,1)]' - [traj(n-1,1:obj.TRAJ_DIMS),psi_targ(n-1,1)]')./(2.*obj.DELTA_T);
                psi = atan2(qd_targ(2),qd_targ(1));
                if qd_targ(3,1) < -pi
                    qd_targ(3,1) = qd_targ(3,1) + 2*pi;
                elseif (qd_targ(3,1) > pi)
                    qd_targ(3,1) = qd_targ(3,1) - 2*pi;
                end

                R = [cos(psi), sin(psi), 0; -sin(psi) cos(psi), 0; 0, 0, 1];

                qdd = R*qdd_targ;
                qd = R*qd_targ;
                
                if (qdd(3) > obj.QDD_MAX(3))
                    qdd(3) = obj.QDD_MAX(3);
                elseif (qdd(3) < obj.QDD_MIN(3))
                    qdd(3) = obj.QDD_MIN(3);
                end

                q_targ = [traj(n,1:obj.TRAJ_DIMS)'; psi];
                
                p_idx = n-1;
                
                
                limiting = false;
                K = reshape(policy(p_idx,:),2,6);
                for j = 1:1:iters
                    
                    xe = ([q_targ; qd]' - x)';
%                     qd = xe(1)/(obj.DELTA_T);
%                     qdd = (qd - x(2))/(obj.DELTA_T);
%                     qdd = min(obj.maxQdd,max(-obj.maxQdd,qdd));
                    u = zeros(1,2);
                    if (run_ff)
                        [u, ~] = invGP.query_data_point([x(4:6),qdd']);
                    end
                    u = -K*xe + u';
                    u(1) = min(obj.U_MAX(1),max(obj.U_MIN(1),u(1)));
                    u(2) = min(obj.U_MAX(2),max(obj.U_MIN(2),u(2)));

                    
                    y(c,:) = obj.getAccel(x,u);
                    %if abs(y(c,1)) > obj.maxQdd
                    %    obj.maxQdd = y(c,1);
                    %end
                    X(c,:) = [x, u'];
                    
                    if (check_surf_type([x(1),x(2)])==0)
                        iceGP = iceGP.add_training_data(X(c,4:8),y(c,:));
                        inviceGP = inviceGP.add_training_data([X(c,4:6), y(c,:)], X(c,7:8));
                    else
                        GP = GP.add_training_data(X(c,4:8),y(c,:));
                        invGP = invGP.add_training_data([X(c,4:6), y(c,:)], X(c,7:8));  
                    end
                    x = obj.forward_dyn(x,u')';
                    c = c + 1;
                end
                
                if SHOWPLOT
                    figure(4)
                    clf;
                    obj.f_map([0,0]);
                    hold on
                    PlotBot(x(1),x(2),x(3))
                    PlotBot(q_targ(1),q_targ(2),q_targ(3))
                    pause(0.1)
                    hold off
                end

            end

        end
        
    end
        
        
        
        
        
        
        
        
        
        
        
        
end