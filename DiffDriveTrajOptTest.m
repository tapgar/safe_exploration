clear
close all

% for_hp = struct('y_std',0.001,'sig_std',0.05,'W',100.*eye(5),'SF',[20,1,4,400,400,10,5,15]);
% inv_hp = struct('y_std',0.001,'sig_std',0.05,'W',100.*eye(6),'SF',[20,1,4,10,5,15,400,400]);
% 
% GP = LocalGP(200, 50, 0.5, for_hp);
% icyGP = LocalGP(200, 50, 0.5, for_hp);
% 
% 
% invGP = LocalGP(200, 50, 0.5, inv_hp);
% invicyGP = LocalGP(200, 50, 0.5, inv_hp);
% 
% env = DiffDriveEnv(@roundabout_map);
% 
% start_pts = 10;
% [x,y] = env.generateLocalData([0.5,0.5,0,0,0,0],[0,0],start_pts);
% 
% for i = 1:1:start_pts
%     GP = GP.add_training_data(x(i,4:8),y(i,:));
%     icyGP = icyGP.add_training_data(x(i,4:8),y(i,:));
%     invGP = invGP.add_training_data([x(i,[4,5,6]),y(i,:)],x(i,[7,8]));
%     invicyGP = invicyGP.add_training_data([x(i,[4,5,6]),y(i,:)],x(i,[7,8]));
% end
% % 
% env = env.NominalTrajectory(GP,icyGP);
% hold on
% plot(env.U_NOM(1,:),env.U_NOM(2,:));

% u = STOMP(env, 20, GP, invGP, icyGP, invicyGP, @collision_cost_function);
% env.U_NOM = [u(2:end-1,:),zeros(length(u)-2,1)]';

 load('ext_run_test_params')
env = DiffDriveEnv(@roundabout_map);
env = env.NominalTrajectory(GP,icyGP);
figure(1);
ftemp = getframe;
k=1;
while true
%     figure(1);
%     plot(env.U_NOM(1,:),env.U_NOM(2,:),'LineWidth',10)
    frames = 1;
    x = env.START_STATE;
    
    sto_noise = env.POINTS_IN_TRAJ*100/50;
    
    [ L_rec, cov, IG, u, F1, fnum ] = STO_LQG_MP(1, env, GP, invGP, icyGP, invicyGP, 100.*diag([100, 100, 100, 2, 2, 2]), eye(2), 0, 0, @collision_cost_function,100, ftemp, frames);
    
    [GP, invGP, icyGP, invicyGP, env, x, F] = env.run_sim(u, cov, x, L_rec, GP, invGP, icyGP, invicyGP, true, k>1, F1, fnum);
    k = k + 1;
    
    if (abs(x(1) - env.END_STATE(1)) + abs(x(2) - env.END_STATE(2)) < 0.5)
        env = DiffDriveEnv(@roundabout_map);
        env = env.NominalTrajectory(GP,icyGP);
    else
        [mx,my] = env.get_nearest_obstacle(x(1), x(2), 0.1, 0.1);
        if sqrt((x(1) - mx)^2 + (x(2)-my)^2) < 0.5
            display('Entered unsafe condition... resetting')
            env = DiffDriveEnv(@roundabout_map);
            env = env.NominalTrajectory(GP,icyGP);
        else
            env.START_STATE = [x(1), x(2), x(3), 0, 0, 0];
            
            env = env.trajOptfromSTOMP(x(3));
            env = env.NominalTrajectory(GP,icyGP);
        end
    end
    
    v = VideoWriter(sprintf('RealExperience_%d.avi',k-1));
    v.FrameRate = 5;
    open(v)
    writeVideo(v,F);
    close(v);
    
    %cov_log(k,:) = [cov(:,1);cov(:,2)]'; 
end