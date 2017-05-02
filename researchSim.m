function [ x, invGP, u_log, jump ] = researchSim( x, invGP, first )

u_log = zeros(1,2);
u_max = 2.0;
x_max = 10;

xdd_max = 1;
xdd_min = -1;

xd = 1.5;
P = 2;

% d = -2;
d= 0;%randn*0.05;
if (x(1) > 1 && x(1) < 2)
    d = 1;
elseif (x(1) > 5 && x(1) < 7)
    d = -1;
end


qdd = (xd - x(2));

% if (qdd > xdd_max)
%     qdd = xdd_max;
% end
% 
% if (qdd < xdd_min)
%     qdd = xdd_min;
% end

[u, ~] = invGP.query_data_point([x(1), xd - x(2), qdd]);

if (~first)
    u = 0;
end

u_log(1) = P*(xd - x(2));
u_log(2) = u;
u = P*(xd - x(2)) + u;

% if (u > u_max)
%     u = u_max;
% elseif (u < -u_max)
%     u = -u_max;
% end 

xdd = researchSim_accel(x, u, d);

% if (xdd > xdd_max)
%     xdd_max = xdd;
% end
% 
% if (xdd < xdd_min)
%     xdd_min = xdd;
% end

invGP = invGP.add_training_data([x(1), xd-x(2), xdd], u);

x(2) = x(2) + xdd*0.01;
x(1) = x(1) + x(2)*0.01;

jump = false;
if (x(1) > x_max)
    x(1) = x(1) - x_max;
    jump = true;
elseif (x(1) < 0)
    x(1) = x(1) + x_max;
    jump = true;
end

end

