function [ c, ceq ] = pendulum_constraints( x, GP, gridN )
    
    % No nonlinear inequality constraint needed
    c = [];
    % Calculate the timestep
    sim_time = x(1);
    delta_time = sim_time / gridN;
    % Get the states / inputs out of the vector
    positions = x(2:1+gridN);
    vels      = x(2+gridN:1+gridN*2);
    u      = x(2+gridN*2:1+gridN*3);
    del      = x(2+gridN*3:end);
    
    % Constrain initial position and velocity to be zero
    ceq = [positions(1); vels(1)];
    for i = 1 : length(positions) - 1
        % The state at the beginning of the time interval
        x_i = [positions(i); vels(i)];
        % What the state should be at the start of the next time interval
        x_n = [positions(i+1); vels(i+1)];
        % The time derivative of the state at the beginning of the time
        % interval
        [qdd, vr] = GP.query_data_point([positions(i), vels(i), u(i)]);
        qdd = qdd + del(i)*sqrt(vr);
        
        xdot_i = [vels(i); qdd];
        
        [qdd, vr] = GP.query_data_point([positions(i+1), vels(i+1), u(i+1)]);
        qdd = qdd + del(i+1)*sqrt(vr);

        % The time derivative of the state at the end of the time interval
        xdot_n = [vels(i+1); qdd];
        
        % The end state of the time interval calculated using quadrature
        xend = x_i + delta_time * (xdot_i + xdot_n) / 2;
        % Constrain the end state of the current time interval to be
        % equal to the starting state of the next time interval
        ceq = [ceq ; x_n - xend];
    end
    % Constrain end position to 1 and end velocity to 0
    ceq = [ceq ; positions(end) - 3.1415; vels(end)];
end