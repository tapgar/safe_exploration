function [ K ] = SE_kernel( sig, width, X, x )


%%%broken dont use!!! takes way too long... see RGP
numX = length(X);
numx = length(x);
xdiff = zeros(numX,numx,length(width));

for k = 1:1:length(width)
    for i = 1:1:numX
        for j = 1:1:numx
            xdiff(i,j,k) = X(i,k) - x(j,k);
        end
    end
    xd2d = reshape(xdiff(:,:,k),numX,numx);
    
end


K = (sig^2).*exp(-0.5*(1/width)*xdiff.^2);

end

