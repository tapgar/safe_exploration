close all
clear
    
Load_Bot_Params
bot1818_20150513_092613_timed_jose

xd = dVelocity_mps(1,1);
yd = 0;
w = dRotationalVelocity_rps(1,1);
psi = dYaw_rad(1,1);
x = dXPosition_m(1,1);
y = dYPosition_m(1,1);

for i = 1:1:length(SysTime)-1

    if i == 7237
       ofuck = true; 
    end
    
    [xdd, ydd, yawdd] = diff_drive_dynamics(Bot,xd,yd,w,TracMotor_current_left(i,1),TracMotor_current_right(i,1),0.001,0.002,10);
    dt = (SysTime(i+1,1) - SysTime(i,1));
    xd = xd + xdd*dt;
    yd = yd + ydd*dt;
    w = w + yawdd*dt;
    
    psi = psi + w*dt;
    x = x + xd*cos(psi)*dt - yd*sin(psi)*dt;
    y = y + xd*sin(psi)*dt + yd*cos(psi)*dt;
    
    hist(i,:) = [x,y,psi,xd,yd,w,xdd,ydd,yawdd];
end

plot(hist(:,4:6))
% hold on
% plot(dVelocity_mps)
% plot(dRotationalVelocity_rps)