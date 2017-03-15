function [ cost ] = collision_cost_function( sf, traj, env )


cost = zeros(env.POINTS_IN_TRAJ,1);
for i = 2:1:length(traj(:,1))-1
    if (env.collision(traj(i-1,1:2),traj(i,1:2)) || traj(i,1) < 0 || traj(i,2) < 0 || traj(i,1) > 10 || traj(i,2) > 10)
        cost(i-1,1) = 1;
    end
end
cost = cumsum(cost);

end

