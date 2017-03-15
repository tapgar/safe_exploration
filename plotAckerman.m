function plotAckerman( s )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here


rec = [0, 0.25; 0, -0.25; 0.5, -0.25; 0.5, 0.25; 0,0.25];

R = [cos(s(5)), -sin(s(5)); sin(s(5)), cos(s(5))];

rotRec = (R*rec')';

c = [s(1); s(3)];

for i = 1:1:5
    rotRec(i,:) = (c + (R*rec(i,:)'))';
end

plot(rotRec(:,1),rotRec(:,2))

wheel_width = 0.075;
wheel_length = 0.025;
rec = [-wheel_width/2, -wheel_length/2; wheel_width/2, -wheel_length/2;...
       wheel_width/2, wheel_length/2; -wheel_width/2, wheel_length/2;...
       -wheel_width/2, -wheel_length/2];
c = c + R*[.5; 0];

R = [cos(s(5)+s(7)), -sin(s(5)+s(7)); sin(s(5)+s(7)), cos(s(5)+s(7))];

for i = 1:1:5
    rotRec(i,:) = (c + (R*rec(i,:)'))';
end

hold on
plot(rotRec(:,1),rotRec(:,2))




xd = s(2)*cos(s(7)) - (s(4) + s(6)*0.5).*sin(s(7));
yd = s(2).*sin(s(7)) + (s(4) + s(6)*0.5).*cos(s(7));

R = [cos(s(5)), -sin(s(5)); sin(s(5)), cos(s(5))];

vec = R*[xd;yd];
ivec = R*[cos(s(7)); sin(s(7))];

quiver(c(1), c(2), vec(1), vec(2))
quiver(c(1), c(2), ivec(1), ivec(2))

xlim([s(1)-1, s(1)+1])
ylim([s(3)-1, s(3)+1])
grid on
end

