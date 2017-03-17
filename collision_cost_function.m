function [ cost ] = collision_cost_function( sf, traj, env, GP, invGP, icyGP, invicyGP )

[ ~, cov, ~, ~, IG ] = LQR_GP( env, traj, GP, invGP, icyGP, invicyGP, 100.*diag([100, 100, 100, 2, 2, 2]), eye(2));

cost = zeros(env.POINTS_IN_TRAJ,1);
for i = 2:1:length(traj(:,1))-1
%     if (env.collision(traj(i-1,1:2),traj(i,1:2)) || traj(i,1) < 0 || traj(i,2) < 0 || traj(i,1) > 10 || traj(i,2) > 10)
%         cost(i-1,1) = 10;
%     end
    
    [mx,my] = env.get_nearest_obstacle(traj(i,1), traj(i,2), cov(1), cov(2));
    dx = [traj(i,1) - mx, traj(i,2) - my]*inv([cov(1), 0; 0, cov(2)])*[traj(i,1) - mx, traj(i,2) - my]';
    cost(i-1,1) = 10.*exp(-dx);
end
% cost = flipud(cumsum(flipud(cost)));


% c=env.POINTS_IN_TRAJ+1;
% psi_targ = zeros(env.POINTS_IN_TRAJ+2,1);
% psi_targ(end,1) = pi;
% for n = 2:1:length(traj(:,1))-1
%     %global vels
%     qd_targ = (traj(n+1,1:env.TRAJ_DIMS)' - traj(n-1,1:env.TRAJ_DIMS)')./(2.*env.DELTA_T);
%     psi_targ(c,1) = atan2(qd_targ(2),qd_targ(1));
%     c=c-1;
% end
% 
% for n = 2:1:length(traj(:,1))-1
%     %global accels
%     qdd_targ = ([traj(n-1,1:env.TRAJ_DIMS),psi_targ(n-1,1)]' - 2.*[traj(n,1:env.TRAJ_DIMS),psi_targ(n,1)]' + [traj(n+1,1:env.TRAJ_DIMS),psi_targ(n+1,1)]')./(env.DELTA_T^2);
%     cost(n-1,1) = cost(n-1,1) + 0.1*(qdd_targ(1)^2 + qdd_targ(2)^2) + qdd_targ(3)^2;
% end



end

