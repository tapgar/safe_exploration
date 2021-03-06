function [traj] = PI2_rollout( GP, x0, k, T )

traj = zeros(T+1,3);
traj(1,:) = [x0,0];
dt = 0.2;
u_max = 1.0;

for t = 1:1:T
    
    if (traj(t,1) > 0.95 && traj(t,1) < 1.05 && abs(traj(t,2)) < 0.01)
        
        traj(t+1,1) = traj(t,1);
        traj(t+1,2) = 0;
        traj(t+1,3) = 0;
        continue;
    end
    u = 0.5*(1-traj(t,1)) + 0.2*(0 - traj(t,2)) + k(t);
    if (u > u_max)
        u = u_max;
    elseif u < -u_max
        u = -u_max;
    end
    traj(t,end) = u;
    [mu, s] = GP.query_data_point(traj(t,2:end));
    qdd = mu + randn(size(s))*sqrt(s);
    qd = traj(t,2) + qdd*dt;
    q = traj(t,1) + qd*dt;
    traj(t+1,1) = q;
    traj(t+1,2) = qd;
    traj(t+1,3) = 0;
    GP = GP.add_training_data(traj(t,2:end),qdd);
end


end

