function [ c, ceq ] = double_integrator_constraints( x )
    global gridN
    % No nonlinear inequality constraint needed
    c = [];
    % Calculate the timestep
    sim_time = x(1);
    delta_time = sim_time / gridN;
    % Get the states / inputs out of the vector
    positions = x(2:1+gridN);
    vels      = x(2+gridN:1+gridN*2);
    accs      = x(2+gridN*2:end);
    
    mu = 0.1;
    % Constrain initial position and velocity to be zero
    ceq = [positions(1); vels(1)];
    for i = 1 : length(positions) - 1
        % The state at the beginning of the time interval
        x_i = [positions(i); vels(i)];
        % What the state should be at the start of the next time interval
        x_n = [positions(i+1); vels(i+1)];
        % The time derivative of the state at the beginning of the time
        % interval
        a_i = acc_f(accs(i), mu, vels(i));
        xdot_i = [vels(i); a_i];
        % The time derivative of the state at the end of the time interval
        a_i_1 = acc_f(accs(i+1), mu, vels(i+1));
        xdot_n = [vels(i+1); a_i_1];
        
        % The end state of the time interval calculated using quadrature
        xend = x_i + delta_time * (xdot_i + xdot_n) / 2;
        % Constrain the end state of the current time interval to be
        % equal to the starting state of the next time interval
        ceq = [ceq ; x_n - xend];
    end
    % Constrain end position to 1 and end velocity to 0
    ceq = [ceq ; positions(end) - 100; vels(end)];
end


function a_f = acc_f(acc_input, mu, vel)
    if vel == 0 % if your not moving, no friction
        a_mu = 0;
        sign_mu = 0;
    else
        sign_mu = -1 * sign(vel); %friction opposes motion
        a_mu = sign_mu * mu;
    end
    
    %accelerating opposing friction then
    if sign(acc_input) == sign_mu
        a_f = acc_input + a_mu; %same direction just add
    else %acting in opposite directions
        if abs(acc_input) < abs(a_mu)
            a_f = 0; % friction is larger
        else
            a_f = acc_input + a_mu; % acceleration is larger
        end
    end
    
%     a_f = acc_input; % no friction!
        
end