clear
close all

hp = struct('y_std',0.01,'sig_std',1.39978,'W',0.1.*eye(3));
GP = LocalGP(20, 50, 0.5, hp);
hp = struct('y_std',0.01,'sig_std',1.39978,'W',0.05.*eye(3));
invGP = LocalGP(20, 50, 0.5, hp);


env = PendulumEnv();

start_pts = 10;
[x,y] = env.generateLocalData([0,0,0],start_pts);

for i = 1:1:start_pts
    GP = GP.add_training_data(x(i,:),y(i,:));
    invGP = invGP.add_training_data([x(i,1:2),y(i,:)],x(i,end));
end


env = env.NominalTrajectory(GP);

for k = 1:1:10

    figure(6)
    plot(env.U_NOM);
    
    u = STOMP(env, 30, GP, invGP, @collision_cost_function);
    
    [ L_rec, cov, ~, ~, IG ] = LQR_GP( env, u, GP, invGP, 100.*eye(2),1);
    
    
    %env.U_NOM = traj(2:end-1,1);
% 
%     qd = zeros(length(u),1);
%     qdd = zeros(length(u),1);
% 
%     for i = 2:1:length(traj)-1
%         qd(i-1,1) = (traj(i+1,1) - traj(i-1,1))./(2*env.DELTA_T);
%         qdd(i-1,1) = (traj(i+1,1) + traj(i-1,1) - 2.*traj(i,1))/(env.DELTA_T^2);
%     end
    
    x = [traj(1,1),0];
    [GP, invGP, env] = env.run_sim(traj(:,1:2), cov, x, L_rec, GP, invGP, true);

    
%     for i = 1:1:length(Xr(:,1))
%        GP = GP.add_training_data(Xr(i,:),yr(i,:)); 
%        invGP = invGP.add_training_data([Xr(i,1:2),yr(i,:)],Xr(i,end));
%     end
    
    
    figure(3);
    plot(u(:,1),u(:,2))
    hold on;
    
    env = env.NominalTrajectory(GP);

end

figure;
plot(u)