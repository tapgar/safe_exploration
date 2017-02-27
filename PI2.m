function [ theta ] = PI2( GP, x0, T, K, best_k, cost_func, safety_func )

traj = zeros(T+1,3,K);
traj_cost = zeros(T+1,K);
traj_safe = zeros(K,1);
all_theta = zeros(T,K);

max_opt = 30;

if isempty(best_k)
   theta = randn(T,1); 
end

num_opt = 0;

eta = 1.5;
% kl_div = 0.1;

while num_opt < max_opt
    eta = eta*(((max_opt-num_opt)/max_opt)^1);
    for k = 1:1:K
        all_theta(:,k) = theta + randn(T,1).*eta;
        traj(:,:,k) = PI2_rollout(GP, x0, all_theta(:,k), T);
        traj_cost(:,k) = cost_func(reshape(traj(:,:,k),T+1,3));
        traj_safe(k,1) = safety_func(reshape(traj(:,:,k),T+1,3));
    end
    
    xs = reshape(traj(:,1,:),T+1,K);
    vs = reshape(traj(:,2,:),T+1,K);
    figure(2)
    plot(xs)
    ylim([-1, 5])
    figure(3)
    plot(vs)
    
    pause(0.01)
    for t = 1:1:T
       
        c_to_go = sum(traj_cost(t:end,:));
        c_to_go = (c_to_go - min(c_to_go))/(max(c_to_go)-min(c_to_go)+0.00001);
        
%         min_func = @(x)kl_dual(x, kl_div, c_to_go);
%         eta = fminsearch(min_func, eta);
%         
        S = exp(-5.*c_to_go);
        P = S./sum(S);
        
        theta(t,1) = P*all_theta(t,:)';
    end
    
    num_opt = num_opt + 1;
end

end

