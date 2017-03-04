clear
close all

num_pts = 100;
num_basis = 50;

w = 1;
sig = 0.2;
sig_n = 1;

X = linspace(-10,10,num_basis)';
Y = zeros(num_basis,1);
cY = 100000.*ones(num_basis,1);
active = zeros(num_basis,1);

% for i = 1:1:num_basis
%     x=X(i,1);
%     Y(i,1) = RGP_testFunc(x);
% end

C = 1000.*SE_kernel(sig,w,X,X);
mu = zeros(num_basis,1);
all_pts = zeros(num_pts,1);

for t = 1:1:num_pts
    x = (rand(1)-0.5)*20;
    y = RGP_testFunc(x);
    
    [m, n] = min(sqrt((X-x).^2));
    if (m < cY(n,1))
        cY(n,1) = m;
        Y(n,1) = y;
        active(n,1) = 1;
    end
    
    all_pts(t,1:2) = [x,y];
    
    %%inference
    
    J = SE_kernel(sig,w,x,X)*inv(SE_kernel(sig,w,X,X));
    %J(1,find(active == 0)) = 0;
    mxs = SE_kernel(sig,w,X,x)'*inv(SE_kernel(sig,w,X,X) + sig_n^2.*eye(num_basis))*Y;
    mXs = zeros(num_basis,1);
    for i = 1:1:num_basis
        mXs(i,1) = SE_kernel(sig,w,X,X(i,1))'*inv(SE_kernel(sig,w,X,X) + sig_n^2.*eye(num_basis))*Y;
    end
    
    mp = mxs + J*(mu - mXs);
    Cp = (sig^2 - J*SE_kernel(sig,w,X,x)) + J*C*J';
%     imagesc(C)
    %%Update
    G = C*J'*inv(Cp + (sig_n^2));
    %G(find(active == 0),1) = 0;
    mu = mu + G*(y - mp);
    C = (eye(num_basis) - G*J)*C;
%     imagesc(C)
%     blah=2;
    
    v = zeros(num_basis,1);
for i = 1:1:num_basis
    J = SE_kernel(sig,w,X(i,1),X)*inv(SE_kernel(sig,w,X,X));
   v(i,1) = (sig^2 - J*SE_kernel(sig,w,X,X(i,1))) + J*C*J'; 
end

% figure; 
plot(X(find(active==1),1),Y(find(active==1),1),'xb')
hold on
plot(X,mu)
plot(X,mu+2.*sqrt(v))
plot(X,mu-2.*sqrt(v))
plot(all_pts(1:t,1),all_pts(1:t,2),'*k')
hold off
pause(0.1)

    
end

