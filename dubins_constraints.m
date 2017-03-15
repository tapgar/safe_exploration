function [ c, ceq ] = dubins_constraints( traj_pts, GP, iceGP, gridN, check_surf_type )
    
    % No nonlinear inequality constraint needed
    c = [];
    % Calculate the timestep
    sim_time = traj_pts(1);
    delta_time = sim_time / gridN;
    % Get the states / inputs out of the vector
    x = traj_pts(2:1+gridN);
    y = traj_pts(2+gridN:1+gridN*2);
    yaw = traj_pts(2+gridN*2:1+gridN*3);
    xd = traj_pts(2+gridN*3:1+gridN*4);
    yd = traj_pts(2+gridN*4:1+gridN*5);
    u = traj_pts(2+gridN*5:1+gridN*6);
    psd = traj_pts(2+gridN*6:1+gridN*7);
    
    delxdd = traj_pts(2+gridN*7:1+gridN*8);
    delydd = traj_pts(2+gridN*8:1+gridN*9);
    
    % Constrain initial position and velocity to be zero
    ceq = [x(1) - 0.5; y(1)-0.5; yaw(1); xd(1); yd(1)];
    for i = 1 : length(x) - 1
        % The state at the beginning of the time interval
        x_i = [x(i); y(i); yaw(i); xd(i); yd(i)];
        % What the state should be at the start of the next time interval
        x_n = [x(i+1); y(i+1); yaw(i+1); xd(i+1); yd(i+1)];
        % The time derivative of the state at the beginning of the time
        % interval
        if (check_surf_type([x(i),y(i)])==0)
            [qdd, vr] = GP.query_data_point([xd(i),yd(i),u(i),psd(i)]);
        else
            [qdd, vr] = iceGP.query_data_point([xd(i),yd(i),u(i),psd(i)]);
        end
        
        qdd = qdd + [delxdd(i),delydd(i)].*sqrt(vr);
        xdot_i = [xd(i); yd(i); qdd'];
        
        if (check_surf_type([x(i+1),y(i+1)])==0)
            [qdd, vr] = GP.query_data_point([xd(i+1),yd(i+1),u(i+1),psd(i+1)]);
        else
            [qdd, vr] = iceGP.query_data_point([xd(i+1),yd(i+1),u(i+1),psd(i+1)]);
        end
        
        qdd = qdd + [delxdd(i+1),delydd(i+1)].*sqrt(vr);
        xdot_n = [xd(i+1); yd(i+1); qdd'];
        
        vx = xdot_i(1) + delta_time*(xdot_i(3) + xdot_n(3))/2;
        vy = xdot_i(2) + delta_time*(xdot_i(4) + xdot_n(4))/2;
        
        ceq = [ceq; x_n(3) - x_i(3) - delta_time*psd(i); x_n(4)-vx; x_n(5)-vy];
        
        vx = delta_time*((xdot_i(1) + xdot_n(1))/2);
        vy = delta_time*((xdot_i(2) + xdot_n(2))/2);
        dx = vx*cos(x_i(3)) - vy*sin(x_i(3));
        dy = vx*sin(x_i(3)) + vy*cos(x_i(3));
        
        ceq = [ceq; x_n(1) - x_i(1) - dx; x_n(2) - x_i(2) - dy];
        
    end
    % Constrain end position to 1 and end velocity to 0
    ceq = [ceq ; x(end)-5; y(end)-9.5; yaw(end)+pi/2; xd(end); yd(end)];
end