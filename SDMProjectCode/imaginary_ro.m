function [p_safe] = imaginary_ro(env, forward_GP, inverse_GP, K, commanded, POINTS_IN_TRAJ, ROLLOUTS)

p_safe = zeros(POINTS_IN_TRAJ,1);
for i_PIJ_real = 2:POINTS_IN_TRAJ
    
    for i_ro = 1:ROLLOUTS
        actual = commanded(i_PIJ_real,:);
        safe = single_noisy_rollout( env, commanded, actual, forward_GP, inverse_GP, K, start_idx, POINTS_IN_TRAJ );
        p_safe(i_PIJ_real,1) = p_safe(i_PIJ_real,1) + safe;
    end
    p_safe(i_PIJ_real,1) = p_safe(i_PIJ_real,1) / ROLLOUTS;
    
    actual = commanded(i_PIJ_real,:);
    
    %go to next point
    [ ~, forward_GP, inverse_GP, env, safe ] = run_fake_controller( env, commanded(i_PIJ_real+1,:), actual, forward_GP, inverse_GP, K );
    
    if ~safe
       p_safe(i_PIJ_real+1:end,1) = 0;
       break;
    end
    
end
