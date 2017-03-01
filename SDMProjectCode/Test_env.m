close all
iter = 1000;

pos = zeros(1,iter);
vel = zeros(1,iter);
u = 0;
z_cur = [3*pi/4,0];
unsafe_array = [];
for i = 1:iter
    [z, unsafe] = env_pendulum(z_cur, u);
    pos(i) = z(1);
    vel(i) = z(2);
    z_cur = z;
    if unsafe
%         fprintf('UNSAFE!')
        unsafe_array = [unsafe_array; i, pos(i), vel(i)];
    end
    
end

plot(pos, 'ro')
hold on
plot(vel, 'bo')
plot(unsafe_array(:,1), unsafe_array(:,2), 'g*')
plot(unsafe_array(:,1), unsafe_array(:,3), 'k*')
hold off