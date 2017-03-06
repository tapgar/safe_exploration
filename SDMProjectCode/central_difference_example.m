%
Fun = @(x) exp(-x).*sin(3*x); 
dFun = @(x) -exp(-x).*sin(3*x)+ 3*exp(-x).*cos(3*x);
%
x=linspace(0,4,101);
F=Fun(x);
h=x(2)-x(1);
xCentral=x(2:end-1);
dFCenteral=(F(3:end)-F(1:end-2))/(2*h);

[F_cdiff, F_ind] = cdiff(F, h);

%
plot(x,dFun(x));
hold on
plot(xCentral,dFCenteral,'r*')
plot(F_ind,F_cdiff,'ko')
legend('Analytic','Central')