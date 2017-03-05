% precompute matrix M for STOMP
% input v is equal to the number of rows for matrix M output

function [M, A, R, R_1] = precompute(v)
% diag = -2 * ones(1,v);

B = zeros(v + 2, v);
C = zeros(v + 2, v);
D = zeros(v + 2, v);

dt = .1;

B(1:end-2, :) = eye(v)./(dt^2);
C(2:end-1, :) = -2 * eye(v)./(dt^2);
D(3:end, :) = eye(v)./(dt^2);

A = B + C + D;

%A(1,1) = 0;
%A(2,1) = -1/(dt^2);
%A(end,end) = 0;
%A(end-1,end) = -1/(dt^2);

R = A' * A;

R_1 = inv(R);

M = R_1./(ones(v,1)*max(R_1,[],1))./v; %scale so that max value is 1/N per column
%M = M.*v;
end
