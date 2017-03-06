function [X_cdiff, X_ind] = cdiff(X, h)
    if nargin < 2
        h = 1;
    end
    
    X_ind = h .* [1:1:(length(X))] - h;
    X_cdiff = (X(3:end) - X(1:end-2)) / 2 / h;
    if size(X,2) == 1
        X_cdiff = [0; X_cdiff; 0];
    elseif size(X,1) == 1
        X_cdiff = [0, X_cdiff, 0];
    end
    
    

end