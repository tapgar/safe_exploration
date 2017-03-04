% precompute matrix M for STOMP
% input v is equal to the number of rows for matrix M output

function [M, A, R_1] = precompute(v)
% diag = -2 * ones(1,v);

A = zeros(v + 2, v);
B = zeros(v + 2, v);
C = zeros(v + 2, v);
D = zeros(v + 2, v);

B(1:end-2, :) = eye(v);
C(2:end-1, :) = -2 * eye(v);
D(3:end, :) = eye(v);

A = B + C + D;

R_1 = inv(A' * A);

M = R_1./(ones(v,1)*max(R_1,[],1))./v; %scale so that max value is 1/N per column

end
