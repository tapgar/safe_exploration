function [ s ] = SimpleSkid( s, u )


%inputs
%   x, xdot, y, ydot, theta, omega, steering angle 


m = 1460;
cf = 17000;
cr = 20000;

a = 1.2;
b = 1.5;
I = 2170;

dt = 0.01;

s(7) = s(7) + u(2)*dt;


if (abs(s(2)) < 0.00001)
    ff = 0;
    fr = 0;
else
    ff = cf*((s(4) + a*s(6))/s(2) + s(7));
    fr = cr*(s(4) - b*s(6))/s(2);
end

xdd = u(1);
alpha = (a*ff - b*fr)/I;
ydd = -s(2)*s(6) + (ff + fr)/m;
ydd = 0;

s(2) = s(2) + xdd*dt;
s(6) = s(6) + alpha*dt;
s(4) = s(4) + dt*ydd;

s(5) = s(5) + s(6)*dt;
s(1) = s(1) + dt*(s(2)*cos(s(5)) - s(4)*sin(s(5)));
s(3) = s(3) + dt*(s(2)*sin(s(5)) + s(4)*cos(s(5)));

end

