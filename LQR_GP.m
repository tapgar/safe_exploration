function [ L_rec, cov, GP, invGP, IG ] = LQR_GP( env, traj, GP, invGP, C, D )

% Inputs
%   traj - [q,qd,qdd] per row... N + 2 rows
%   GP - gaussian process class
%   C - quadratic state cost
%   D - quadratic action cost

% Outputs
%   pi - time varying feedback gain
%   cov - covariance ellipse at each step

N = length(traj(:,1))-2; %exclude goal point at end
nX = length(C);
nU = length(D);

L_rec = zeros(N,nX*nU);

cov=zeros(N,nX+nU);

dt = env.DELTA_T;

S = C;
for n = N+1:-1:2
    
    qdd_targ = (traj(n-1,1:env.TRAJ_DIMS)' - 2.*traj(n,1:env.TRAJ_DIMS)' + traj(n+1,1:env.TRAJ_DIMS)')./(env.DELTA_T^2);
    qd_targ = (traj(n+1,1:env.TRAJ_DIMS)' - traj(n-1,1:env.TRAJ_DIMS)')./(2.*env.DELTA_T);
    q_targ = traj(n,1:env.TRAJ_DIMS)';
    
    [u, ~] = invGP.query_data_point([q_targ', qd_targ', qdd_targ']);
    
    [a, b] = GP.linearize([q_targ', qd_targ'],u);
    A = [1, dt; dt*a(1,1), 1+dt*a(2,1)];
    B = [0; b*dt];
    L = -((B'*S*B + D)^-1)*B'*S*A;
    S = C + A'*S*A + A'*S*B*L;
    L_rec(n-1,:) = reshape(L,1,nX*nU);
end

R = 0.000001.*eye(nX);
IG = zeros(N,1);

for n = 2:1:N+1
    
    p_idx = n-1;
    
    
    qdd_targ = (traj(n-1,1:env.TRAJ_DIMS)' - 2.*traj(n,1:env.TRAJ_DIMS)' + traj(n+1,1:env.TRAJ_DIMS)')./(env.DELTA_T^2);
    qd_targ = (traj(n+1,1:env.TRAJ_DIMS)' - traj(n-1,1:env.TRAJ_DIMS)')./(2.*env.DELTA_T);
    q_targ = traj(n,1:env.TRAJ_DIMS)';
    
    [u, ~] = invGP.query_data_point([q_targ', qd_targ', qdd_targ']);
    
    
   [qdd_actual, SIG] = GP.query_data_point([q_targ', qd_targ', u']);

   [a, b] = GP.linearize([q_targ', qd_targ'],u);
   A = [1, dt; dt*a(1,1), 1+dt*a(2,1)];
   B = [0; b*dt];
   L = reshape(L_rec(p_idx,:),nU,nX);
   COV = [0, 0; 0, SIG*dt];
   R = (A + B*L)*R*(A + B*L)' + COV;
   
   GP = GP.add_training_data([q_targ', qd_targ', u'],qdd_actual');
   invGP = invGP.add_training_data([q_targ', qd_targ',qdd_actual'],u);

   [~, SIG2] = GP.query_data_point([q_targ', qd_targ', u']);
   
   if (SIG2 > SIG)
      wtf=true; 
   end
   
   IG(p_idx,1) = 0.5*log2(SIG^2/SIG2^2);
       
       
   Lj = L;
   delj = [eye(nX); Lj];
   cov_temp = delj*R*delj';
   [sig, ~] = cov2corr(cov_temp);
   cov(p_idx,:) = sig;

end


end








