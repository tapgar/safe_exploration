function [ costs ] = double_integrator_cost_func( traj )

costs = zeros(length(traj(:,1)),1);
% costs(end,1) = 0.5*(1.0 - traj(end,1))^2 + 1.0*(traj(end,2))^2;
for i = 1:1:length(traj(:,1))
    
    costs(i,1) = 0.5*(1.0 - traj(i,1))^2;
%    costs(i,1) = costs(i,1) + 1.0*(traj(i,2))^2;
%     if (traj(i,1) > 0.95 && traj(i,1) < 1.05 && abs(traj(i,2)) < 0.1)
%         costs(i,1) = -1;
%     end
%     
%     if (traj(i,1) > 1.05)
%        costs(i,1) = 1; 
%     end
%     
%     if (abs(traj(i,1)) > 0.05)
%        costs(i,1) = 1; 
%     end
% costs(i,1) = costs(i,1) + 0.1*traj(i,3)^2; 
%     if i > 1
%        costs(i,1) = costs(i,1) + 0.2*(traj(i,3)-traj(i-1,3))^2; 
%        
%     end
costs(i,1) = costs(i,1) + 0.1*traj(i,3)^2;
end
costs(end,1) = costs(end,1) + 2.0*(traj(end,2))^2;

end

