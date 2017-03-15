function [ s, fw, fyp, xdd, psdd ] = AckermanDynamics( s, u )

%input state x
%   xpos, ypos, yaw, vx, vy, w, steering angle
%input action u
%   forward F, steering vel

%%%% First we have to solve assuming velocity constraints are held

max_vel = 1000000;
dt = 0.01;
rx = 0.25;
cx = 0.5;
m = 5;
Izz = m*(.25^2 + .5^2)/12;

x = s(1);
xd = s(2);

if (xd > max_vel)
    xd = max_vel;
end

y = s(3);
yd = s(4);
ps = s(5);
psd = s(6);
a = s(7);

% [static_force_x, static_force_y, kinetic_force_x, kinetic_force_y] = get_friction(map, x, y
% 
static_force_x = 7;
% if (abs(u(1)) < static_force_x)
%     fx = u(1);
% else
%     fx = kinetic_force_x;
% end

fx = u(1);% - 0.05 - 0.1*xd ;
da = u(2);
ad = da;

fxp = 0;% -sign(xd)*(0.05 + 0.1*xd);

fw=(-1).*(Izz+m.*rx.^2+(-1).*(Izz+m.*((-1).*cx.^2+rx.^2)).*cos(a).^2)...
  .^(-1).*(m.^2.*psd.*rx.^2.*xd+(-1).*m.^2.*psd.*rx.*((-1).*cx+rx).* ...
  xd.*cos(a).^2+fxp.*(Izz+m.*rx.*((-1).*cx+rx)).*sin(a)+(fx+m.*(ad+( ...
  -1).*psd).*psd.*(cx+(-1).*rx)).*(Izz+m.*rx.*((-1).*cx+rx)).*cos(a) ...
  .*sin(a));

fyp=(Izz+m.*rx.^2+(-1).*(Izz+m.*((-1).*cx.^2+rx.^2)).*cos(a).^2).^(-1)...
  .*((fx+m.*(ad+(-1).*psd).*psd.*(cx+(-1).*rx)).*(Izz+m.*rx.^2).* ...
  sin(a)+cos(a).*(cx.*m.^2.*psd.*rx.*xd+fxp.*(Izz+m.*((-1).*cx.^2+ ...
  rx.^2)).*sin(a)));

xdd = (fx + (fxp*cos(a) - fyp*sin(a)) - m*rx*psd*psd)/m;

% psdd = ((fxp*sin(a) + fyp*cos(a))*rx - m*rx*psd*xd)/(Izz + m*rx*rx);

psdd = (fw + (fxp*sin(a) + fyp*cos(a)))/m*rx;

% 
ypdd = (xdd + m*rx*psd*psd)*sin(a) + (psdd*cx)*cos(a);


xd = xd + xdd*dt;
psd = psd + psdd*dt;

ps = ps + psd*dt;
x = x + xd*cos(ps)*dt;
y = y + xd*sin(ps)*dt;

a = a + da*dt;

s = [x, xd, y, yd, ps, psd, a];
end

