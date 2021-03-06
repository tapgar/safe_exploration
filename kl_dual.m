function [ dual_val ] = kl_dual( eta, kl_threshold, costs )
dual_val = eta*kl_threshold + eta*log((1.0/length(costs)))*sum(exp(costs./eta));
end

