function [ c, ceq ] = flight_constraint( x )

c=[];
% Unpack the parameters
time = x(1);
vx_0 = x(2);
vy_0 = x(3);

% Calculate the travel distances
x_travel = time * vx_0;
y_travel = time * vy_0 - 0.5*9.806*time^2;

ceq = [x_travel - 5; y_travel - 10];

end

