clear
close all

hp = struct('y_std',0.01,'sig_std',1.39978,'W',0.1.*eye(4));
GP = LocalGP(20, 50, 0.5, hp);
icyGP = LocalGP(20, 50, 0.5, hp);
invGP = LocalGP(20, 50, 0.5, hp);
invicyGP = LocalGP(20, 50, 0.5, hp);

env = DubbinsEnv(@roundabout_map);

start_pts = 10;
[x,y] = env.generateLocalData([0.5,0.5,0,0,0,0],[0,0],start_pts);

for i = 1:1:start_pts
    GP = GP.add_training_data(x(i,[4,5,7,8]),y(i,:));
    icyGP = icyGP.add_training_data(x(i,[4,5,7,8]),y(i,:));
    invGP = invGP.add_training_data([x(i,[4,5]),y(i,:)],x(i,[7,8]));
    invicyGP = invicyGP.add_training_data([x(i,[4,5]),y(i,:)],x(i,[7,8]));
end

env = env.NominalTrajectory(GP,icyGP);
hold on
%plot(env.U_NOM(1,:),env.U_NOM(2,:));

for k = 1:1:10
    
    u = STOMP(env, 40, GP, invGP, @collision_cost_function);
    
    [ L_rec, cov, ~, ~, IG ] = LQR_GP( env, u, GP, invGP, icyGP, invicyGP, 0.5.*diag([10, 10, 10, 2, 2, 2]), eye(2));
    
    figure(2)
    clf
    plot_cov(env, u, cov, IG);
    
end