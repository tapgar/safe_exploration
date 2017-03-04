function [ y ] = RGP_testFunc( x )


y = (x/2) + ((25*x)/(1+x^2))*cos(x) + randn(1)*0.1;

end

