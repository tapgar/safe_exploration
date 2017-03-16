function plot_cov(env, u, cov, IG)

env.f_map([0,0]);
hold on
plot(u(:,1),u(:,2))

for i = 1:1:length(cov)
   plot([u(i,1)-cov(i,1)*2.2, u(i,1)+cov(i,1)*2.2],[u(i,2), u(i,2)])
   hold on
   plot([u(i,1), u(i,1)],[u(i,2)-cov(i,2)*2.2, u(i,2)+cov(i,2)*2.2])
   if i > 1
        plot(u(i,1),u(i,2),'g.','MarkerSize',0.01+IG(i-1)*10)
   end
end

plot([0,0,10,10,0],[0,10,10,0,0],'k')
xlim([-1,11])
ylim([-1,11])

end

