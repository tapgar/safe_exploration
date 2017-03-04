clear
close all

hp = struct('y_std',0.001027,'sig_std',0.39978,'W',1.*eye(3));
GP = LocalGP(200, 60, 0.8, hp);

load pendulumData

idx = 1:5:length(X);
traj = X(idx,:);

traj = zeros(size(traj));

start_pts = 1;
idx = 1;%randi(length(X),start_pts,1);
x = X(idx,:);
y = y(idx,:);

for i = 1:1:start_pts
    GP = GP.add_training_data(x(i,:),y(i,:));
end

for t = 1:1:5
    [pi, cov, GP] = LQR_GP(traj,GP,100.*eye(2),1,X);

    plot(cov(:,1))
    hold on
    pause(0.01)
end