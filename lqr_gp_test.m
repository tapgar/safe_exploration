clear
close all

hp = struct('y_std',0.001,'sig_std',0.39978,'W',1.*eye(3));
GP = LocalGP(200, 600, -1, hp);

load pendulumData

idx = 1:5:length(X);
traj = X(idx,:);

env = PendulumEnv();
start_pts = 10;
[x,y] = env.generateLocalData([0,0,0],start_pts);

%traj = zeros(size(traj));

% start_pts = 1;
% idx = randi(length(X),start_pts,1);
% x = X(idx,:);
% y = y(idx,:);

for i = 1:1:start_pts
    GP = GP.add_training_data(x(i,:),y(i,:));
end

for t = 1:1:5
    [pi, cov, ~, IG] = LQR_GP(traj,GP,100.*eye(2),1,X);
    
    figure(1)
    [Xr, yr] = env.plot(traj,cov,traj(1,1:2),pi);

    for i = 1:1:length(Xr(:,1))
       GP = GP.add_training_data(Xr(i,:),yr(i,:)); 
    end
    
    figure(2)
    subplot(3,1,1)
    plot(cov(:,1))
    hold on
    subplot(3,1,2)
    plot(cov(:,2))
    hold on
    subplot(3,1,3)
    plot(IG)
    hold on
    
    pause(1)
    
    
end