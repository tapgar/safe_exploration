

clear all
close all

% Load example data: 
% This is the well-known "motor cycle data" featuring the different input dependent noise.

load pendulumData
%load MotorCycle_Data
x = X(:,[1,3]);% + 0.01.*randn(401,2);

% x = [x(1:50); x(end-50:end)];
% y = [x(1:50); y(end-50:end)];

%hp = struct('y_std',511,'sig_std',2050,'W',0.0362);

hp = struct('y_std',20,'sig_std',200,'W',eye(2));

model = LocalGP(200, 60, 0.5, hp);


num_cells = 100;
x1 = linspace(min(x(:,1)),max(x(:,1)),num_cells);
x2 = linspace(min(x(:,2)),max(x(:,2)),num_cells);
[X1, X2] = meshgrid(x1,x2);
ypred2 = zeros(size(X1));
V = zeros(size(X1));

last_mod_num = 1;

%i=shuffleRows((1:1:length(y(:,1)))')';
for i=1:1:length(y(:,1))
   
    model = model.add_training_data(x(i,:),y(i,:));
    [ypred(i) v(i)] = model.query_data_point(x(i,:));
    
    if model.num_models ~= last_mod_num
        display('New model!')
    end
    

    
    last_mod_num = model.num_models;

    
end

% figure; 
% plot(x,y,'*'); 
% hold on; 
% plot(x,ypred,'r')
% xi = 1:1:60;
% for i =1:1:60
%     [ypred2(i) V(i)] = model.query_data_point(xi(i));
% end
% plot(xi,ypred2+2.*sqrt(V),'g-')
% plot(xi,ypred2-2.*sqrt(V),'g-')
% 
% for k=1:1:model.num_models
%     plot(model.model_list{k}.c,model.model_list{k}.get_prediction(model.model_list{k}.c),'kx') 
% end

       % if mod(i,10)
        for j = 1:1:num_cells
            for k = 1:1:num_cells
                [ypred2(j,k) V(j,k)] = model.query_data_point([X1(j,k),X2(j,k)]);
            end
        end
    %     xq = 1:1:60;
    %     for k=1:1:length(xq)
    %         [ypred2(k), v(k)] = model.query_data_point(xq(k));
    %     end


        mesh(X1,X2,ypred2)
        hold on
        mesh(X1,X2,ypred2 + 2.*sqrt(V))
        mesh(X1,X2,ypred2 - 2.*sqrt(V))

        plot3(x(1:i,1),x(1:i,2),y(1:i,1),'.')
        plot3(x(1:i,1),x(1:i,2),ypred,'r-')
    %     plot(xq,ypred2,'g-')
    %     plot(xq,ypred2+2*sqrt(v))
    %     plot(xq,ypred2-2*sqrt(v))
    % 
    %     for k=1:1:model.num_models
    %        plot(model.model_list{k}.c,model.model_list{k}.get_prediction(model.model_list{k}.c),'kx') 
    %     end
    %     
        pause(0.01);
        hold off
    %end


% [2561.09029232627, 2048.53263189192, 2026.41443591064;
%  2048.53263189192, 2561.09029232627, 2036.70638392414;
%  2026.41443591064, 2036.70638392414, 2561.09029232627]