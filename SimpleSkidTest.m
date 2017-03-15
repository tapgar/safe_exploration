clear
close all

s = [0, 0, 0, 0, 0, 0.0, pi/4];
u = [1, 0];

for i = 1:1:10000

    s = SimpleSkid(s, u);
    
    hist(i,:) = s;
    
%     if (i == 5000)
%         s(end) = pi/2;
%     end
%     
    plotAckerman(s);
    pause(0.00001)
    hold off
end

plot(hist(:,1), hist(:,3))
figure;
plot(hist(:,1), hist(:,2))

