function [z_new, unsafe] = env_pendulum(z, u)
    l = 1;    % [m]        length of pendulum
    m = 1;    % [kg]       mass of pendulum
    g = 9.82; % [m/s^2]    acceleration of gravity
    b = 0.01; % [s*Nm/rad] friction coefficient
    d_time = 10^-2;
    unsafe = 1; % assume unsafe
    
    % z = [theta, theta_dot]
    
    dz = zeros(size(z));
    dz(1) = z(2);
%     dz(2) = ( u - b*z(2) - m*g*l*sin(z(1))/2 ) / (m*l^2/3);
    dz(2) = ( u - m*g*l*sin(z(1))/2 ) / (m*l^2/3);
    
    z_new = z + dz * d_time;
    z_new(1) = mod(z_new(1), 2*pi);
    
    % map check - map can be passed as argument to env_pendulum function
    if ~env_map(z(1), z_new(1))
        unsafe = 0; % safe zone!
    end

end