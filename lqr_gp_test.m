clear
close all

hp = struct('y_std',0.01,'sig_std',1.39978,'W',0.1.*eye(3));
GP = LocalGP(200, 60, 0.5, hp);
invGP = LocalGP(200, 60, 0.5, hp);

load pendulumData

idx = 1:5:length(X);
traj = X(idx,:);

env = PendulumEnv();
start_pts = 1;
[x,y] = env.generateLocalData([0,0,0],start_pts);

%traj = zeros(size(traj));

% start_pts = 1;
% idx = randi(length(X),start_pts,1);
% x = X(idx,:);
% y = y(idx,:);

for i = 1:1:start_pts
    GP = GP.add_training_data(x(i,:),y(i,:));
    invGP = invGP.add_training_data([x(i,1:2),y(i,:)],x(i,end));
end

for t = 1:1:10
    [pi, cov, ~, IG] = LQR_GP(traj,GP,10.*eye(2),1,X);
    
    figure(1)
    [Xr, yr] = env.plot(traj,cov.^2,traj(1,1:2),pi,invGP);

    for i = 1:1:length(Xr(:,1))
       GP = GP.add_training_data(Xr(i,:),yr(i,:)); 
       invGP = invGP.add_training_data([Xr(i,1:2),yr(i,:)],Xr(i,end));
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