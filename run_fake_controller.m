function [ actual, forward_GP, inverse_GP, env, safe ] = run_fake_controller( env, commanded, actual, forward_GP, inverse_GP, K, RUN_NOISY )

%self reinforced inverse and forward model predictors

prev = actual;
for c=1:1:env.DELTA_T/env.dt
    %Traj Error
    e = (commanded(1, 1:2) - actual(1,1:2))';
    %Desired Accel
    qdd = env.getTargetAccel(commanded(1, 1:2), e);
    %inverse dynamic model
    [u, ~] = inverse_GP.query_data_point([actual(1,1:2),qdd]);
    u = -K*e + u;
    u = min(env.U_MAX,max(env.U_MIN,u));

    %given pos, velocity, and control input predict
    %acceleration
    [qdd,  v] = forward_GP.query_data_point([actual(1,1:2), u]);

    env = env.updateMaxAccel(qdd);

    %update GPs record information gain
    inverse_GP = inverse_GP.add_training_data([actual,qdd],u);
    forward_GP = forward_GP.add_training_data([actual,u],qdd);


    %given acceleration update actual
    actual(1,2) = actual(1,2) + env.dt * qdd; %velocity update
    actual(1,1) = actual(1,1) + env.dt * actual(1,2) + 1/2 * qdd * env.dt^2; % position update

    p = env_map(prev, actual);
    if p == 0
        safe = false;
        break
    end
    
    prev = actual;
end

end

