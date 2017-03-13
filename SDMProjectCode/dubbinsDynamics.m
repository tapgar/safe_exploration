function out = dubbinsDynamics(u, q0, dt)
    % given an inputs, initial state, and time step
    % u is [acceleration, thetadd]
    % q0 is [x, y, theta]
    % q0d is [v, thetad]
    
    % return position velocity along the way (and acceleration)
    
    % no constraints on dynamics!
    thetad_min = -1.5; thetad_max = 1.5;
    a_min = -5; a_max = 5;
    
    q_array = zeros(size(u, 1) + 1, 3);
    qd_array = zeros(size(u, 1) + 1, 2);
    q_array(1,:) = q0(1:3);
    qd_array(1,:) = q0(4:5);
    % for each input
    for i = 2:size(u,1)+1 % initial position to one beyond end of array
        %update state
        % x_new = x_old + v_old * cos(theta_old) * dt;
        % y_new = y_old + v_old * sin(theta_old) * dt;
        % theta_new = theta_old + thetad * dt;
        
        
        q_array(i, 1) = q_array(i - 1, 1) + qd_array(i - 1, 1) * cos(q_array(i-1,3)) * dt;
        q_array(i, 2) = q_array(i - 1, 2) + qd_array(i - 1, 1) * sin(q_array(i-1,3)) * dt;
        % behavior for reversing
        if qd_array(i-1, 1) >= 0
            q_array(i, 3) = q_array(i - 1, 3) + qd_array(i - 1, 2) * dt;
        else
            q_array(i, 3) = q_array(i - 1, 3) - qd_array(i - 1, 2) * dt;
        end
            
        
        % v_new = v_old + u1 * dt
        % theta_turn = thetad_old + u2 * dt
        qd_array(i, 1) = qd_array(i - 1, 1) + u(i - 1, 1) * dt;
        qd_array(i, 2) = qd_array(i - 1, 2) + u(i - 1, 2) * dt;
    end
    
    out = [q_array, qd_array];
end