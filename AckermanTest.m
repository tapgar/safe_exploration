clear
close all

s = [0, 0, 0, 0, 0, 0.0, pi/2.05];
u = [1, 0];

for i = 1:1:10000

    [s, fw, fyp, xdd, psdd] = AckermanDynamics(s, u);
    
    hist(i,:) = [s, fw, fyp, xdd, psdd];
    
%     if (i == 5000)
%         s(end) = pi/2;
%     end
%     
%     plotAckerman(s);
%     pause(0.00001)
%     hold off
end

plot(hist(:,1), hist(:,3))
figure;
plot(hist(:,1), hist(:,2))

figure;
plot(hist(:,8))
hold on
plot(hist(:,9))

yd = -hist(:,2).*sin(hist(:,7)) + (hist(:,4) + hist(:,6)*0.5).*cos(hist(:,7));

figure; 
plot(yd)

% hold on
% plot(hist(:,2))

