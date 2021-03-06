function p = CovarianceChecker(C, M)
% from:
% http://www.visiondummy.com/2014/04/draw-error-ellipse-representing-covariance-matrix/
% https://www.mathworks.com/matlabcentral/newsreader/view_thread/42966
% http://www.visiondummy.com/2014/04/geometric-interpretation-covariance-matrix/
    
    T = sqrt(C);
    [eigenvec, eigenval] = eig(C);
    
    radii = abs(diag(eigenval));
    r_dims = length(radii);
    r_coords = zeros(r_dims, r_dims);
    for r_ind = 1:r_dims
        
        % equation for ellipse is SUM{ x-xc/a } = Constant
        % can use the less than equation to see if it inside
        r_coords(r_ind, r_ind) = radii(r_ind); 
        
        x = [0, r_coords(r_ind,1)] + M(1);
        y = [0, r_coords(r_ind,2)] + M(2);
        plot(x, y, 'k-')
        eigvec_cur = abs(eigenvec(:,r_ind));
        quiver(M(1), M(2), eigvec_cur(1), eigvec_cur(2))
        hold on
    end
    hold off
    xlim([-3, 3])
    ylim([-3, 3])
    
    
%     % Get the index of the largest eigenvector
%     [largest_eigenvec_ind_c, r] = find(eigenval == max(max(eigenval)));
%     largest_eigenvec = eigenvec(:, largest_eigenvec_ind_c);
% 
%     % Get the largest eigenvalue
%     largest_eigenval = max(max(eigenval));
% 
%     % Get the smallest eigenvector and eigenvalue
%     if(largest_eigenvec_ind_c == 1)
%         smallest_eigenval = max(eigenval(:,2))
%         smallest_eigenvec = eigenvec(:,2);
%     else
%         smallest_eigenval = max(eigenval(:,1))
%         smallest_eigenvec = eigenvec(1,:);
%     end
% 
%     % Calculate the angle between the x-axis and the largest eigenvector
%     angle = atan2(largest_eigenvec(2), largest_eigenvec(1));
% 
%     % This angle is between -pi and pi.
%     % Let's shift it such that the angle is between 0 and 2pi
%     if(angle < 0)
%         angle = angle + 2*pi;
%     end
% 
%     % Get the coordinates of the data mean
%     avg = M;
% 
%     % Get the 95% confidence interval error ellipse
%     chisquare_val = 2.4477;
%     theta_grid = linspace(0,2*pi);
%     phi = angle;
%     X0=avg(1);
%     Y0=avg(2);
%     a=chisquare_val*sqrt(largest_eigenval);
%     b=chisquare_val*sqrt(smallest_eigenval);
% 
%     % the ellipse in x and y coordinates 
%     ellipse_x_r  = a*cos( theta_grid );
%     ellipse_y_r  = b*sin( theta_grid );
% 
%     %Define a rotation matrix
%     R = [ cos(phi) sin(phi); -sin(phi) cos(phi) ];
% 
%     %let's rotate the ellipse to some angle phi
%     r_ellipse = [ellipse_x_r;ellipse_y_r]' * R;
% 
%     % Draw the error ellipse
%     plot(r_ellipse(:,1) + X0,r_ellipse(:,2) + Y0,'-')
%     hold on;
% 
%     % Plot the original data
% %     plot(data(:,1), data(:,2), '.');
% %     mindata = min(min(data));
% %     maxdata = max(max(data));
% %     xlim([mindata-3, maxdata+3]);
% %     ylim([mindata-3, maxdata+3]);
% %     hold on;
% 
%     % Plot the eigenvectors
%     quiver(X0, Y0, largest_eigenvec(1)*sqrt(largest_eigenval), largest_eigenvec(2)*sqrt(largest_eigenval), '-m', 'LineWidth',2);
%     quiver(X0, Y0, smallest_eigenvec(1)*sqrt(smallest_eigenval), smallest_eigenvec(2)*sqrt(smallest_eigenval), '-g', 'LineWidth',2);
%     hold on;
% 
%     % Set the axis labels
%     hXLabel = xlabel('x');
%     hYLabel = ylabel('y');










%     [U,L] = eig(C);
% 
%     % For N standard deviations spread of data, the radii of the eliipsoid will
%     % be given by N*SQRT(eigenvalues).
% 
%     N = 1; % choose your own N
%     radii = N*sqrt(diag(L));
% 
%     % generate data for "unrotated" ellipsoid
%     [xc,yc,zc] = ellipsoid(0,0,0,radii(1),radii(2),radii(3));
% 
%     % rotate data with orientation matrix U and center M
%     a = kron(U(:,1),xc); b = kron(U(:,2),yc); c = kron(U(:,3),zc);
%     data = a+b+c;
%     n = size(data,2);
%     x = data(1:n,:)+M(1);
%     y = data(n+1:2*n,:)+M(2);
%     z = data(2*n+1:end,:)+M(3);
%     
%     % eigenvec plots
%     L_diag = diag(L);
%     x_e = zeros(size(L,1),1);
%     x_e(1) = M(1);
%     x_e(end) = x_e(1) + L_diag(1);
%     
%     % now plot the rotated ellipse
%     sc = surf(x,y,z);
% %     plot3(x_e, [0, 0], [0, 0])
%     shading interp
%     title('actual ellipsoid represented by data: C and M')
%     axis equal
%     alpha(0.5)
%     
%     
%     p = 0;

end