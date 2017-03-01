function M = precompute(v)

diag = -2 * ones(1,v);

A = zeros(v + 2, v);
B = zeros(v + 2, v);
C = zeros(v + 2, v);
D = zeros(v + 2, v);

B(1:end-2, :) = eye(v);
C(2:end-1, :) = -2 * eye(v);
D(3:end, :) = eye(v);

A = B + C + D;

R_1 = (transpose(A) * A)';

M = R_1;

end
