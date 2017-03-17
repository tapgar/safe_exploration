function [ L_rec, cov, GP, invGP, IG ] = LQR_GP( env, traj, GP, invGP, iceGP, inviceGP, C, D )

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

c=env.POINTS_IN_TRAJ+1;
psi_targ = zeros(env.POINTS_IN_TRAJ+2,1);
psi_targ(end,1) = pi;
for n = N+1:-1:2
    %global accels
    qdd_targ = (traj(n-1,1:env.TRAJ_DIMS)' - 2.*traj(n,1:env.TRAJ_DIMS)' + traj(n+1,1:env.TRAJ_DIMS)')./(env.DELTA_T^2);
    %global vels
    qd_targ = (traj(n+1,1:env.TRAJ_DIMS)' - traj(n-1,1:env.TRAJ_DIMS)')./(2.*env.DELTA_T);
    psi_targ(c,1) = atan2(qd_targ(2),qd_targ(1));
    c=c-1;
end


S = C;
for n = N+1:-1:2
    
    %global accels
    qdd_targ = ([traj(n-1,1:env.TRAJ_DIMS),psi_targ(n-1,1)]' - 2.*[traj(n,1:env.TRAJ_DIMS),psi_targ(n,1)]' + [traj(n+1,1:env.TRAJ_DIMS),psi_targ(n+1,1)]')./(env.DELTA_T^2);
    %global vels
    qd_targ = ([traj(n+1,1:env.TRAJ_DIMS),psi_targ(n+1,1)]' - [traj(n-1,1:env.TRAJ_DIMS),psi_targ(n-1,1)]')./(2.*env.DELTA_T);
    psi = atan2(qd_targ(2),qd_targ(1));
    if qd_targ(3,1) < -pi
        qd_targ(3,1) = qd_targ(3,1) + 2*pi;
    elseif (qd_targ(3,1) > pi)
        qd_targ(3,1) = qd_targ(3,1) - 2*pi;
    end
    
    R = [cos(psi), sin(psi), 0; -sin(psi) cos(psi), 0; 0, 0, 1];
    
    qdd = R*qdd_targ;
    qd = R*qd_targ;
    
    q_targ = [traj(n,1:env.TRAJ_DIMS)'; psi];
    
    if (check_surf_type([q_targ(1),q_targ(2)])==0)
        [u, ~] = inviceGP.query_data_point([qd', qdd']);
        [a, b] = iceGP.linearize(qd',u);
    else
        [u, ~] = invGP.query_data_point([qd', qdd']);
        [a, b] = GP.linearize(qd',u);
    end
    
    
    A = eye(6);
    B = zeros(6,2);
    A(1,4) = cos(psi)*dt;
    A(1,5) = -sin(psi)*dt;
    A(2,4) = sin(psi)*dt;
    A(2,5) = cos(psi)*dt;
    A(3,6) = dt;
    A(4,4:end) = A(4,4:end) + a(1,1:3).*dt;
    A(5,4:end) = A(5,4:end) + a(2,1:3).*dt;
    A(6,4:end) = A(6,4:end) + a(3,1:3).*dt;
    
    B(4:6,:) = b.*dt;
    
    L = -((B'*S*B + D)^-1)*B'*S*A;
    S = C + A'*S*A + A'*S*B*L;
    L_rec(n-1,:) = reshape(L,1,nX*nU);
end

R = 0.000001.*eye(nX);
IG = zeros(N,1);

for n = 2:1:N+1
    
    p_idx = n-1;
    
    %global accels
    qdd_targ = ([traj(n-1,1:env.TRAJ_DIMS),psi_targ(n-1,1)]' - 2.*[traj(n,1:env.TRAJ_DIMS),psi_targ(n,1)]' + [traj(n+1,1:env.TRAJ_DIMS),psi_targ(n+1,1)]')./(env.DELTA_T^2);
    %global vels
    qd_targ = ([traj(n+1,1:env.TRAJ_DIMS),psi_targ(n+1,1)]' - [traj(n-1,1:env.TRAJ_DIMS),psi_targ(n-1,1)]')./(2.*env.DELTA_T);
    psi = atan2(qd_targ(2),qd_targ(1));
    if qd_targ(3,1) < -pi
        qd_targ(3,1) = qd_targ(3,1) + 2*pi;
    elseif (qd_targ(3,1) > pi)
        qd_targ(3,1) = qd_targ(3,1) - 2*pi;
    end
    
    rot = [cos(psi), sin(psi), 0; -sin(psi) cos(psi), 0; 0, 0, 1];
    
    qdd = rot*qdd_targ;
    qd = rot*qd_targ;
    
    if (qdd(3) > env.QDD_MAX(3))
        qdd(3) = env.QDD_MAX(3);
    elseif (qdd(3) < env.QDD_MIN(3))
        qdd(3) = env.QDD_MIN(3);
    end
    
    q_targ = [traj(n,1:env.TRAJ_DIMS)'; psi];
    
    if (check_surf_type([q_targ(1),q_targ(2)])==0)
        [u, ~] = inviceGP.query_data_point([qd', qdd']);
        [qdd_actual, SIG] = iceGP.query_data_point([qd', u]);
        [a, b] = iceGP.linearize(qd',u);
    else
        [u, ~] = invGP.query_data_point([qd', qdd']);
        [qdd_actual, SIG] = GP.query_data_point([qd', u]);
        [a, b] = GP.linearize(qd',u);
    end
    
    A = eye(6);
    B = zeros(6,2);

    A(1,4) = cos(psi)*dt;
    A(1,5) = -sin(psi)*dt;
    A(2,4) = sin(psi)*dt;
    A(2,5) = cos(psi)*dt;
    A(3,6) = dt;
    A(4,4:end) = A(4,4:end) + a(1,1:3).*dt;
    A(5,4:end) = A(5,4:end) + a(2,1:3).*dt;
    A(6,4:end) = A(6,4:end) + a(3,1:3).*dt;
    
    B(4:6,:) = b.*dt;
       
   L = reshape(L_rec(p_idx,:),nU,nX);
   COV = zeros(6);
   COV(4,4) = SIG*dt;
   COV(5,5) = SIG*dt;
   COV(6,6) = SIG*dt;
   R = (A + B*L)*R*(A + B*L)' + COV;
   
   if (check_surf_type([q_targ(1),q_targ(2)])==0)
       iceGP = iceGP.add_training_data([qd', u],qdd_actual);
       inviceGP = inviceGP.add_training_data([qd',qdd_actual],u);
       [~, SIG2] = iceGP.query_data_point([qd', u]);
   else
       GP = GP.add_training_data([qd', u],qdd_actual);
       invGP = invGP.add_training_data([qd',qdd_actual],u);
       [~, SIG2] = GP.query_data_point([qd', u]);
   end
   
   if (SIG2 > SIG)
      wtf=true; 
   end
   
   IG(p_idx,1) = 0.5*log2(SIG^2/SIG2^2);
       
       
   Lj = L;
   delj = [eye(nX); Lj];
   cov_temp = delj*R*delj';
   [sig, ~] = cov2corr(cov_temp);
   if (sig(1) > 2)
      somethingfishy = true; 
   end
   cov(p_idx,:) = sig;

end


end








