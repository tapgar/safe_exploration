function [p_safe] = imaginary_ro(controller, DELTA_T, model, commanded, actual, POINTS_IN_TRAJ, ROLLOUTS, u_rollout, VIS_ROLLOUTS)

close all;

    for i_PIJ_real = 2:POINTS_IN_TRAJ
            model2 = model;
            p_safe = zeros(ROLLOUTS,1);
            fprintf('Point: %.f ', i_PIJ_real)
            for i_ro = 1:ROLLOUTS
                p_total = 0;
                GP_vals = zeros(POINTS_IN_TRAJ, 4); %[mean, Vbef, Vafter, KLDistance]
                for i_PIJ = i_PIJ_real:POINTS_IN_TRAJ
                    e(i_PIJ) = [commanded(i_PIJ, 1) - actual(i_PIJ-1,1)];
                    e_dot(i_PIJ) = [commanded(i_PIJ, 2) - actual(i_PIJ-1,2)];
                    u(i_PIJ) = controller(e(i_PIJ), e_dot(i_PIJ));
                    % Sample at each time step
                    [GP_vals(i_PIJ,1),  GP_vals(i_PIJ,2)] = model2.query_data_point([actual(i_PIJ-1,1:2), u_rollout(i_PIJ)]);
                    actual(i_PIJ,3) = GP_vals(i_PIJ,1) + GP_vals(i_PIJ,2) * randn;
                    model2 = model2.add_training_data([actual(i_PIJ-1,1:2), u_rollout(i_PIJ)], actual(i_PIJ,3));
                    actual(i_PIJ,2) = actual(i_PIJ-1,2) + DELTA_T * actual(i_PIJ,3); %velocity update
                    actual(i_PIJ,1) = actual(i_PIJ-1,1) + DELTA_T * actual(i_PIJ-1,2) + 1/2 * actual(i_PIJ,3) * DELTA_T^2; % position update
                    [~, GP_vals(i_PIJ, 3)] = model2.query_data_point([actual(i_PIJ-1,1:2), u_rollout(i_PIJ)]);
                    GP_vals(i_PIJ, 4) = 0.5 * log(GP_vals(i_PIJ, 2)^2 - GP_vals(i_PIJ, 3)^2); %KL distance between Gaussians
                    % check if it is safe
                    p = env_map(actual(i_PIJ-1,1), actual(i_PIJ,1));
                    if p == 0
    %                     disp('Unsafe Point!')
                        break
                    else
                        p_total = p_total + p;
                    end
                end
            p_safe(i_ro) = p_total / POINTS_IN_TRAJ;
            fprintf('Rollout %.f, Steps %.f: %.3f \n', i_ro, i_PIJ, p_safe(i_ro))
            end
            if VIS_ROLLOUTS
                hold on
                plot(actual(:,1),'color',rand(1,3),'marker','o')
                plot(commanded(:,1),'bo')
                plot(actual(:,2),'r*')
                plot(commanded(:,2),'b*')
            end

    end
    

end
