function  plotPendulum( traj, cov )


l = 1;

pts = [-0.1, 0;...
       -0.1, l;...
       0.1, l;...
       0.1, 0;...
       -0.1, 0];

for i = 1:1:length(traj(:,1))-1
   
    th = traj(i,1)+pi;
    R = [cos(th), -sin(th); sin(th), cos(th)];
    npts = pts*R;
    
    s = sqrt(cov(i,1))*1.96;
    cpts = [-s/2, 1; s/2, 1]*R;
    
    plot(npts(:,1),npts(:,2));
    hold on
    plot(cpts(:,1),cpts(:,2),'r');
    xlim([-2.5, 2.5])
    ylim([-1.2, 1.2])
    pause(0.1)
    hold off
    
end


end

