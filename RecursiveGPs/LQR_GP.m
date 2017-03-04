function [ L_rec, cov, GP ] = LQR_GP( traj, GP, C, D, ALL_DATA_X )

% Inputs
%   traj - [x,u] per row... N rows
%   GP - gaussian process class
%   C - quadratic state cost
%   D - quadratic action cost

% Outputs
%   pi - time varying feedback gain
%   cov - covariance ellipse at each step

N = length(traj(:,1))-1; %exclude goal point at end
nX = length(C);
nU = length(D);

L_rec = zeros(N,nX*nU);

cov=zeros(N,nX+nU);

dt = 0.1;

S = C;
for n = N:-1:1
    xn = traj(n+1,1:nX);
    un = traj(n+1,nX+1:end);
    [a, b] = GP.linearize(xn,un);
    A = [1, dt; dt*a(1,1), 1+dt*a(2,1)];
    B = [0; b*dt];
    L = -((B'*S*B + D)^-1)*B'*S*A;
    S = C + A'*S*A + A'*S*B*L;
    L_rec(n,:) = reshape(L,1,nX*nU);
end

R = 0.00000000000001.*eye(nX);
COV = zeros(nX);
for n = 1:1:N
   [ypred, SIG] = GP.query_data_point(traj(n,:));
   
   xn = traj(n,1:nX);
   un = traj(n,nX+1:end);
   [a, b] = GP.linearize(xn,un);
   A = [1, dt; dt*a(1,1), 1+dt*a(2,1)];
   B = [0; b*dt];
   L = reshape(L_rec(n,:),nU,nX);
   COV = A*COV*A' + [0, 0; 0, SIG*dt];
   R = (A + B*L)*R*(A + B*L)' + COV;
   GP = GP.add_training_data(traj(n,:),ypred);
   [ypred, SIG2] = GP.query_data_point(traj(n,:));
   if (SIG2 > SIG)
      wtf=true; 
   end
%    GP.plot(ALL_DATA_X,[1,3],traj(n,2));
%    plot3(traj(n,1),traj(n,3),ypred,'.')
%    pause(0.01)
%    hold off
   
   if n > 40
      ohno=true; 
   end
   
   if n == 1
       F = (A + B*L)';
       Rn = R;
       Ln = L;
       delN = [eye(nX); Ln];
   else
       F = (A + B*L)'*F;
       
       Lj = L;
       delj = [eye(nX); Lj];
%        cov_temp = delN*Rn*F*delj';
       cov_temp = delj*R*delj';
       [sig, corr] = cov2corr(cov_temp);
       %imagesc(cov_temp)
       cov(n,:) = sig;
   end

end


end

