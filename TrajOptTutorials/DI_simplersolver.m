% minimize time for a trajectory
% can only change acceleration inputs



x_start = 0;
v_start = 0;
a_start = 0;

x_end = 10;
v_end = 0;
a_end = 0;


gridN = 20; % how much to discretize length
% delta_t = 0.1;

a0 = ones(gridN, 1); % initial guess












    function t = traj_time(x_start, v_start, a_start, a0)
        delta_t = 0.1;
        done_flag = 0;
        while done_flag == 0

            for i = 2:length(a0) - 1
               x_n =  x_start + delta_t * v_start + 0.5 * delta_t ^ 2 * a_start;
               v_n = v_start + delta_t * a_start;
               a_n = a0(i + 1);
            end

            if abs(x_n - x_end)/x_end < 0.01 %1 percent error
                t = delta_t;
                done_flag = 1;
            elseif x_n < x_end
                delta_t = delta_t * 1.1;
            elseif x_n > x_end
                delta_t = delta_t * 0.95;
            end
        end

    end

