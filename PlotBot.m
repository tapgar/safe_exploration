function [  ] = PlotBot( x,y,yaw,color)
zoom=0;
window = 2;
wheelsize=[.1,.05];
framesize=[1.222,.75];
bot.frame(1,:) = [-.142,1.114,1.114,-.142];
bot.frame(2,:)=[-.384,-.384,.384,.384];
bot.rwheel(1,:)=[-wheelsize(1),wheelsize(1),wheelsize(1),-wheelsize(1)];
bot.rwheel(2,:)=[-framesize(2)/2-wheelsize(2),-framesize(2)/2-wheelsize(2),-framesize(2)/2,-framesize(2)/2];
bot.lwheel(1,:)=bot.rwheel(1,:);
bot.lwheel(2,:)=bot.rwheel(2,:)+framesize(2)+wheelsize(2);

    
bot.xpos=x;
bot.ypos=y;
bot.theta=yaw;
R=[cos(bot.theta),-sin(bot.theta);sin(bot.theta),cos(bot.theta)];

rframe=R*bot.frame;
rrwheel=R*bot.rwheel;
rlwheel=R*bot.lwheel;
fill(rframe(1,:)+bot.xpos,rframe(2,:)+bot.ypos,color);    
fill(rlwheel(1,:)+bot.xpos,rlwheel(2,:)+bot.ypos,[.8 .8 .8]);
fill(rrwheel(1,:)+bot.xpos,rrwheel(2,:)+bot.ypos,[.8 .8 .8]);

end

