clear
close all

for_hp = struct('y_std',0.001,'sig_std',0.05,'W',50.*eye(5),'SF',[20,1,4,400,400,10,5,15]);
inv_hp = struct('y_std',0.001,'sig_std',0.05,'W',50.*eye(6),'SF',[20,1,4,10,5,15,400,400]);

GP = LocalGP(200, 50, 0.5, for_hp);
icyGP = LocalGP(200, 50, 0.5, for_hp);


invGP = LocalGP(200, 50, 0.5, inv_hp);
invicyGP = LocalGP(200, 50, 0.5, inv_hp);

env = DiffDriveEnv(@roundabout_map);

start_pts = 10;
[x,y] = env.generateLocalData([0.5,0.5,0,0,0,0],[0,0],start_pts);

for i = 1:1:start_pts
    GP = GP.add_training_data(x(i,4:8),y(i,:));
    icyGP = icyGP.add_training_data(x(i,4:8),y(i,:));
    invGP = invGP.add_training_data([x(i,[4,5,6]),y(i,:)],x(i,[7,8]));
    invicyGP = invicyGP.add_training_data([x(i,[4,5,6]),y(i,:)],x(i,[7,8]));
end
% 
env = env.NominalTrajectory(GP,icyGP);
% hold on
% plot(env.U_NOM(1,:),env.U_NOM(2,:));

u = STOMP(env, 20, GP, invGP, icyGP, invicyGP, @collision_cost_function);
% env.U_NOM = [u(2:end-1,:),zeros(length(u)-2,1)]';
    
for k = 1:1:10
    
    [ L_rec, cov, ~, ~, IG ] = LQR_GP( env, u, GP, invGP, icyGP, invicyGP, 100.*diag([100, 100, 100, 2, 2, 2]), eye(2));
    IG(find(IG < 0)) = 0;
    
    figure(k+1)
    clf
    plot_cov(env, u, cov, IG);
    
    [GP, invGP, icyGP, invicyGP, env] = env.run_sim(u, cov, [0.5,0.5,0,0,0,0], L_rec, GP, invGP, icyGP, invicyGP, true, k>5);
    
%     env = env.trajOptfromSTOMP();
%     env = env.NominalTrajectory(GP,icyGP);
    
    cov_log(k,:) = [cov(:,1);cov(:,2)]'; 
end