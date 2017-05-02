function [ L_rec, cov, IG, u, F, fnum] = STO_LQG_MP( sf, env, GP, invGP, icyGP, invicyGP, Q, R, IG, s, cost_func, stomp_noise, F, fnum )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

u = STOMP(env, 20, GP, invGP, icyGP, invicyGP, IG, s, cost_func, stomp_noise, sf);

[ L_rec, cov, ~, ~, IG, s ] = LQR_GP( env, u, GP, invGP, icyGP, invicyGP, Q, R);
IG(find(IG < 0)) = 0;

max_plan_idx = length(u(:,1));
for t = 2:1:length(u(:,1))-1
    [x,y] = env.get_nearest_obstacle(u(t,1),u(t,2),cov(t-1,1),cov(t-1,2));
%     if (sqrt((x - u(t,1))^2 + (y - u(t,2))^2) < sf*sqrt(cov(t-1,1)^2 + cov(t-1,2)^2))
    if (abs(x - u(t,1)) < sf*cov(t-1,1) && abs(y - u(t,2)) < sf*cov(t-1,2))
       figure(5)
       clf
       plot_cov(env,u,cov,IG,sf)
       F(fnum) = getframe;
       fnum = fnum+1;
       display(sprintf('Uncertain Collision Detected: (%f, %f)', u(t,1), u(t,2)))
       display('Planning Intermediate Trajectory')
       max_plan_idx = t-1;
       
       safe_env = env;
       safe_env.POINTS_IN_TRAJ = max(10,max_plan_idx);
       safe_env.DELTA_T = safe_env.DELTA_T*(max_plan_idx)/safe_env.POINTS_IN_TRAJ;
       safe_env.END_STATE = [u(max_plan_idx,1), u(max_plan_idx,2), s(max_plan_idx,4), 0, 0, 0];
       
       safe_env.U_NOM = [linspace(env.START_STATE(1),safe_env.END_STATE(1),safe_env.POINTS_IN_TRAJ);...
               linspace(env.START_STATE(2),safe_env.END_STATE(2),safe_env.POINTS_IN_TRAJ);...
               linspace(env.START_STATE(3),safe_env.END_STATE(3),safe_env.POINTS_IN_TRAJ)];
       [L_rec, cov, IG, u, F, fnum] = STO_LQG_MP(sf, safe_env, GP, invGP, icyGP, invicyGP, Q, R, IG(1:max_plan_idx,:), s(1:max_plan_idx,:), @safe_cost_func, stomp_noise/2, F, fnum);
       return;
    end
end

figure(5)
clf
plot_cov(env,u,cov,IG,sf)
F(fnum) = getframe;
fnum = fnum+1;


end

