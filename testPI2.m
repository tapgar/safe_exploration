clear
close all


[X, y] = generate_fake_DI_data(100);

hp = struct('y_std',0.0104,'sig_std',0.39978,'W',0.2.*eye(2));

GP = LocalGP(200, 30, 0.5, hp);
for i = 1:1:length(X(:,1))
   GP = GP.add_training_data(X(i,:),y(i,1)); 
end

num_cells = 100;
x1 = linspace(-2,4,num_cells);
x2 = linspace(-3,3,num_cells);
[X1, X2] = meshgrid(x1,x2);

ypred = zeros(size(X1));
V = zeros(size(X1));

for j = 1:1:num_cells
    for k = 1:1:num_cells
        [ypred(j,k), V(j,k)] = GP.query_data_point([X1(j,k),X2(j,k)]);
    end
end

mesh(X1,X2,ypred)
hold on
mesh(X1,X2,ypred + 2.*sqrt(V))
mesh(X1,X2,ypred - 2.*sqrt(V))

plot3(X(1:i,1),X(1:i,2),y(1:i,1),'.')


figure(2);
theta = PI2(GP, [0,0], 20, 60, [], @double_integrator_cost_func, @double_integrator_safety_func);
