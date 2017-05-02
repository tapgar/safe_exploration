function [ c, ceq ] = diff_drive_constraints( traj_pts, GP, iceGP, gridN, env, check_surf_type )
    
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
    psd = traj_pts(2+gridN*5:1+gridN*6);
    
    uL = traj_pts(2+gridN*6:1+gridN*7);
    uR = traj_pts(2+gridN*7:1+gridN*8);
    
    
    delxdd = traj_pts(2+gridN*8:1+gridN*9);
    delydd = traj_pts(2+gridN*9:1+gridN*10);
    deltdd = traj_pts(2+gridN*10:1+gridN*11);
    
    % Constrain initial position and velocity to be zero
    ceq = [x(1)-env.START_STATE(1); y(1)-env.START_STATE(2); yaw(1)-env.START_STATE(3); xd(1); yd(1); psd(1)];
    for i = 1 : length(x) - 1
        % The state at the beginning of the time interval
        x_i = [x(i); y(i); yaw(i); xd(i); yd(i); psd(i)];
        % What the state should be at the start of the next time interval
        x_n = [x(i+1); y(i+1); yaw(i+1); xd(i+1); yd(i+1); psd(i+1)];
        % The time derivative of the state at the beginning of the time
        % interval
        if (check_surf_type([x(i),y(i)])==0)
            [qdd, vr] = GP.query_data_point([xd(i),yd(i),psd(i),uL(i),uR(i)]);
        else
            [qdd, vr] = iceGP.query_data_point([xd(i),yd(i),psd(i),uL(i),uR(i)]);
        end
        
        
        qdd = qdd + [delxdd(i)/sqrt(vr*100),delydd(i)/sqrt(vr*25),deltdd(i)/sqrt(vr*225)];
        xdot_i = [xd(i); yd(i); psd(i); qdd'];
        
        if (check_surf_type([x(i+1),y(i+1)])==0)
            [qdd, vr] = GP.query_data_point([xd(i+1),yd(i+1),psd(i+1),uL(i+1),uR(i+1)]);
        else
            [qdd, vr] = iceGP.query_data_point([xd(i+1),yd(i+1),psd(i+1),uL(i+1),uR(i+1)]);
        end
        
        qdd = qdd + [delxdd(i+1)/sqrt(vr*100),delydd(i+1)/sqrt(vr*25),deltdd(i+1)/sqrt(vr*225)];
        xdot_n = [xd(i+1); yd(i+1); psd(i+1); qdd'];
        
        vx = xdot_i(1) + delta_time*(xdot_i(4) + xdot_n(4))/2;
        vy = xdot_i(2) + delta_time*(xdot_i(5) + xdot_n(5))/2;
        w = xdot_i(3) + delta_time*(xdot_i(6) + xdot_n(6))/2;
        
        ceq = [ceq; x_n(4)-vx; x_n(5)-vy; x_n(6)-w];
        
        vx = delta_time*((xdot_i(1) + xdot_n(1))/2);
        vy = delta_time*((xdot_i(2) + xdot_n(2))/2);
        dx = vx*cos(x_i(3)) - vy*sin(x_i(3));
        dy = vx*sin(x_i(3)) + vy*cos(x_i(3));
        dpsi = delta_time*(xdot_i(3) + xdot_n(3))/2;
        
        ceq = [ceq; x_n(1) - x_i(1) - dx; x_n(2) - x_i(2) - dy; x_n(3)-x_i(3)-dpsi];
        
    end
    % Constrain end position to 1 and end velocity to 0
    ceq = [ceq ; x(end)-6.5; y(end)-9; yaw(end)-pi; xd(end); yd(end); psd(1)];
end