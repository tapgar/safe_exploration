function [ X, y ] = generate_fake_DI_data( num_pts )

X = randn(num_pts,2)*0.15;
y = X(:,2) + randn(num_pts,1).*0.01;

end

