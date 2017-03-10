function p = SimpleCovarianceChecker(C, M, point)
% finds largest eigenvalue and makes sure no points are within hyper sphere
% true if outside covariance
% false if inside covariance
    [eigenvec, eigenval] = eig(C);
    
    [max_r, max_ind] = max(diag(eigenval));
    
    dims = length(point); %number of dimensions searching over
    p_all = zeros(dims,1);
%     for i = 1:dims
%         if (abs(point(i) - M(i)) - max_r) > 0
%             p_all(i) = true;
%         else
%             p_all(i) = false;
%         end
%     end
    if (norm(point - M) - max_r) > 0
        p_all = true;
    else
        p_all = false;
    end
        
    p = all(p_all);

end