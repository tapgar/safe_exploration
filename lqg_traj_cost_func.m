function [ cost ] = lqg_traj_cost_func( policy, sf, forGP, invGP, traj, env )

%inputs
%   policy - feedback gain matrix
%   sf - std dev scaled by safety factor
%   forGP - forward dynamics
%   invGP - inverse dynamics
%   traj - trajectory from 0 to T+1
%   env - environment class

qdd_sq_err = zeros(env.POINTS_IN_TRAJ,1);
u_cost = zeros(env.POINTS_IN_TRAJ,1);

%sf = 0;

for k = 2:1:length(traj(:,1))-1
   
    p_idx = k-1;
    
    %central finite difference method
    qdd_targ = (traj(k-1,1:env.TRAJ_DIMS)' - 2.*traj(k,1:env.TRAJ_DIMS)' + traj(k+1,1:env.TRAJ_DIMS)')./(env.DELTA_T^2);
    qd_targ = (traj(k+1,1:env.TRAJ_DIMS)' - traj(k-1,1:env.TRAJ_DIMS)')./(2.*env.DELTA_T);
    q_targ = traj(k,1:env.TRAJ_DIMS)';
    
    [u, invCov] = invGP.query_data_point([q_targ', qd_targ', qdd_targ']);
    
    if (u + sf*sqrt(invCov) > env.U_MAX)
        u_cost(p_idx,1) = ((u + sf*invCov) - env.U_MAX)^2;
    elseif (u - sf*sqrt(invCov) < env.U_MIN)
        u_cost(p_idx,1) = ((u - sf*invCov) - env.U_MIN)^2;
    end
    
    [qdd_actual, ~] = forGP.query_data_point([q_targ', qd_targ', u']);
    qdd_sq_err(p_idx,1) = (qdd_targ - qdd_actual)'*(qdd_targ - qdd_actual);
end

cost = 0.*u_cost + 1.0.*qdd_sq_err + 0.0000001.*randn(env.POINTS_IN_TRAJ,1);

end

