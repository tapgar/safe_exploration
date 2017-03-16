clear
close all

x = [0, 0];

W = diag([5, 5, 5]);

hp = struct('y_std',0.01,'sig_std',1.39978,'W',W);
invGP = LocalGP(50, 60, 0.5, hp);
xdd = researchSim_accel([0.01, 0.01], 0, 0);
invGP = invGP.add_training_data([0.01, 0.01, xdd], 0);

hist = zeros(10000,4);
c = 0;
for i = 1:1:10000
    c=c+1;
    [x, invGP, ulog, jump] = researchSim(x, invGP, c<i);
    hist(c,:) = [x, ulog];
    if jump
        c  =c - 1;
        figure(1);
        hold on
        plot(hist(1:c,1), hist(1:c,2));
        figure(2);
        plot(hist(1:c,1), hist(1:c,3))
        hold on
        figure(3)
        hold on
        plot(hist(1:c,1), hist(1:c,4))
        
        
        
        c = 0;
    end
end

for i = 1:1:invGP.num_models
   
    figure(4)
    plot(invGP.model_list{i}.c(1),invGP.model_list{i}.c(2),'.')   
    hold on
    figure(5)
     plot(invGP.model_list{i}.c(1),invGP.model_list{i}.c(3),'.') 
     hold on
end