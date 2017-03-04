clear
close all

num_pts = 100;
num_basis = 500;

w = 1;
sig = 0.5;
sig_n = 5;

X = linspace(-10,10,num_basis)';

GP = RGP(X,sig,sig_n,w);

all_pts = zeros(num_pts,1);

for t = 1:1:num_pts
    x = (rand(1)-0.5)*20;
    y = RGP_testFunc(x);
    
    GP = GP.update(x,y);
   
    all_pts(t,1:2) = [x,y];
      
    v = GP.getBasisVar();

    plot(GP.X,GP.Y,'.b')
    hold on
    plot(GP.X,GP.mu)
    plot(GP.X,GP.mu+2.*sqrt(v))
    plot(GP.X,GP.mu-2.*sqrt(v))
    plot(all_pts(1:t,1),all_pts(1:t,2),'*k')
    hold off
    pause(0.1)
end

