function [ safe ] = single_noisy_rollout( env, commanded, actual, forward_GP, inverse_GP, K, start_idx, POINTS_IN_TRAJ )

for i_PIJ = start_idx:POINTS_IN_TRAJ
    [ actual, forward_GP, inverse_GP, ~, safe ] = run_controller( env, commanded(start_idx,:), actual, forward_GP, inverse_GP, K, true );
    if safe == 0
        break;
    end
end

end

