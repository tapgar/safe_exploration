function check = env_map(s1, s2)

pendulum_topx = 0;
pendulum_topy = 1;

%pendulum_top = [0, 1];

r = 1;

t = linspace(s1, s2);

x = r * cos(t) + pendulum_topx;
y = r * sin(t) + pendulum_topy;

b1 = 0;
b2 = (23*pi)/12;

unsafe = linspace(b1, b2);

bx = r * cos(unsafe) + pendulum_topx;
by = r * sin(unsafe) + pendulum_topy;

if size(intersect(t, unsafe)) > 0
    check = 1;
else
    check = 0;
end

%if start1 <= end2 && start2 <= end1
%    check = 1;
%else
%    check = 0;
%end

plot(x, y, 'o');
axis([-2, 2, -1, 3]); 
hold on
plot(bx, by, 'o');
hold on

%cart_x1 = 1 * sin(s1);
%cart_y1 = 1 - 1 * cos(s1);

%cart_x2 = 1 * sin(s2);
%cart_x2 = 1 - 1 * cos(s2);

%figure();
%plot(pendulum_topx, pendulum_topy, cart_x1, cart_y1, '-');
%hold on
%plot(pendulum_topx, pendulum_topy, cart_x2, cart_y2, '-');

end