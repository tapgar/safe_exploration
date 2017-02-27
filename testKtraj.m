clear
close all

num_traj = 50;

%Set up the GP dynamics model structure
dynmodel.fcn = @gp1d;                % function for GP predictions
dynmodel.train = @train;             % function to train dynamics model
dynmodel.induce = zeros(300,0,1);    % shared inducing inputs (sparse GP)... wtf is this?
trainOpt = [300 500];                % defines the max. number of line searches
                                     % when training the GP dynamics models
                                     % trainOpt(1): full GP,
                                     % trainOpt(2): sparse GP (FITC)
                                     
num_timesteps = 30;
traj = gpuArray(zeros(num_traj,num_timesteps));

for i=1:1:num_traj
    GPs{i,1} = dynmodel;
    GPs{i,2} = traj;
end
%GPs = repmat(dynmodel, num_traj, num_timesteps);


test = cellfun(@PI2,traj);