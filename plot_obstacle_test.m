
close all
env = DiffDriveEnv(@roundabout_map);

plot_cov(env,u,cov,IG)
hold on

for i = 2:1:length(u(:,1))-1
   
    x = u(i,1);
    y = u(i,2);
    dx = cov(i-1,1);
    dy = cov(i-1,2);
    
    [xo, yo] = env.get_nearest_obstacle(x,y,dx,dy);
    
    plot([x,xo],[y,yo],'r')
    
end