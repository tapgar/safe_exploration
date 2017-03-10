% from:
% http://www.visiondummy.com/2014/04/draw-error-ellipse-representing-covariance-matrix/

% Create some random data
s = [2 2];
x = randn(500,1);
% x2 = randn(500, 1);
% data = [x, x2];
y1 = normrnd(s(1).*x,1);
y2 = normrnd(s(2).*x,1);
data = [y1 y2];
m = mean(data);
% Calculate the eigenvectors and eigenvalues
c = cov(data);

% C = [4.6845, -1.8587, 1.6523;
%     -1.8587, 1.3192, -0.7436;
%     1.6523, -0.7436, 1.2799];
% 
% M = [-9.7275; 4.5526; -5.6775];

p = CovarianceChecker(c, m);