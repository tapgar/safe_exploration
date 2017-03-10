% from:
% http://www.visiondummy.com/2014/04/draw-error-ellipse-representing-covariance-matrix/
%% Simple Covariance Checker - Simple Circle
m = [0, 0];
c = [1, 0;
    0, 1];
point = [0.5, 0.5];
actual = false;
p = SimpleCovarianceChecker(c, m, point);
fprintf('Actual: %d, Predicted: %d \n', actual, p)

point = [1.5, 1.5];
actual = true;
p = SimpleCovarianceChecker(c, m, point);
fprintf('Actual: %d, Predicted: %d \n', actual, p)

%% Simple Covariance Checker - Simple Circle with Mean Moved
m = [3, 4];
c = [1, 0;
    0, 1];
point = [0.5, 0.5];
actual = true;
p = SimpleCovarianceChecker(c, m, point);
fprintf('Actual: %d, Predicted: %d \n', actual, p)

point = [3.5, 1.5];
actual = false;
p = SimpleCovarianceChecker(c, m, point);
fprintf('Actual: %d, Predicted: %d \n', actual, p)

%% Simple Covariance Checker - Simple Circle in 3D
m = [0, 0, 0];
c = [1, 0, 0;
    0, 1, 0;
    0, 0, 1];
point = [0.5, 0.5, 0.5];
actual = false;
p = SimpleCovarianceChecker(c, m, point);
fprintf('Actual: %d, Predicted: %d \n', actual, p)

point = [3.5, 1.5, 1.5];
actual = true;
p = SimpleCovarianceChecker(c, m, point);
fprintf('Actual: %d, Predicted: %d \n', actual, p)

%% Simple Covariance Checker - Simple Circle with Mean moved in 3D
m = [3, 3, 3];
c = [1, 0, 0;
    0, 1, 0;
    0, 0, 1];
point = [0.5, 0.5, 0.5];
actual = true;
p = SimpleCovarianceChecker(c, m, point);
fprintf('Actual: %d, Predicted: %d \n', actual, p)

point = [3.5, 1.5, 1.5];
actual = false;
p = SimpleCovarianceChecker(c, m, point);
fprintf('Actual: %d, Predicted: %d \n', actual, p)

%% Simple Covariance Checker - Simple Circle with mean moved with rotation in 3D
m = [3, 3, 3];
c = [1, 2, 3;
    -1, 1, 3;
    1, 1, 1];
point1 = [0.5, 0.5, 0.5];
actual = true;
p = SimpleCovarianceChecker(c, m, point1);
fprintf('Actual: %d, Predicted: %d \n', actual, p)

point2 = [3.5, 1.5, 1.5];
actual = false;
p = SimpleCovarianceChecker(c, m, point2);
fprintf('Actual: %d, Predicted: %d \n', actual, p)

[evec, eval] = eig(c);
r = diag(eval);
[x, y, z] = ellipsoid(m(1), m(2), m(3), r(1), r(2), r(3));
s = surf(x, y, z);
s.FaceColor = 'r';
hold on
[x, y, z] = ellipsoid(m(1), m(2), m(3), max(r), max(r), max(r));
t = surf(x, y, z);
t.FaceColor = 'y';
t.FaceAlpha = 0.2;
plot3(point1(1), point1(2), point1(3),'ro')
plot3(point2(1), point2(2), point2(3),'bo')
hold off

%% H3 Dimensional Sampling Test
m = [0, 0,0];
sample_size = 100;
samples = randn(sample_size, 3) + m;

C = cov(samples);
M = mean(samples, 1);

[evec, eval] = eig(C);
r = diag(eval);
[x, y, z] = ellipsoid(M(1), M(2), M(3), max(r), max(r), max(r));
t = surf(x, y, z);
hold on;
t.FaceColor = 'y';
t.FaceAlpha = 0.2;

for i = 1:length(samples)
    p = SimpleCovarianceChecker(C, M, samples(i,:));
    if p == 1
        plot3(samples(i,1), samples(i,2), samples(i,3),'ro')
    elseif p == 0
        plot3(samples(i,1), samples(i,2), samples(i,3),'bo')
    end
end
hold off
%% Simple circle
m = [0, 0];
c = [1, 0;
    0, 1];

CovarianceChecker(c, m)

%% Translated circle
m = [1, -1];
c = [1, 0;
    0, 1];

CovarianceChecker(c, m)
%% Simple rotated circle
m = [0, 0];
theta = pi/8;
c = [cos(theta), sin(theta);
    -sin(theta), cos(theta)];

CovarianceChecker(c, m)

%%
% Create some random data
s = [2 2];
x = randn(500,1);
% x2 = randn(500, 1);
% data = [x, x2];
y1 = normrnd(s(1).*x,1);
y2 = normrnd(s(2).*x,1);
data = [y1 y2];
m = m(data);
% Calculate the eigenvectors and eigenvalues
c = cov(data);

% C = [4.6845, -1.8587, 1.6523;
%     -1.8587, 1.3192, -0.7436;
%     1.6523, -0.7436, 1.2799];
% 
% M = [-9.7275; 4.5526; -5.6775];

p = CovarianceChecker(c, m);