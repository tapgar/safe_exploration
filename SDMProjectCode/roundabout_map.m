function [fric, safe] = roundabout_map(s)
% check all states to see if they are in danger zone!
    

    % set danger area (we can make this an image and do different checking)
    % dangerous obstacles
%     i_o = 1;
%     obstacle{i_o} = [8,10; 9,10; 9,9; 8,9]; i_o = i_o + 1;
%     obstacle{i_o} = [1,8; 2,8; 2,9; 1,9]; i_o = i_o + 1;
    %obstacle{i_o} = [-2, -2; -2, 2; 2, 2; 2, -2]; i_o = i_o + 1;
    % ice obstacles
%     i_c = 1;
%     ice{i_c} = [5, 5; 5, 6; 6, 6; 6, 5]; i_c = i_c + 1;
%     ice{i_c} = [-2, -2; -2, 2; 2, 2; 2, -2]; i_c = i_c + 1;
%     ice{i_c} = [9,9; 10,9; 10,8; 9,8]; i_c = i_c + 1;
    
    
    
    i_o = 1;
    obstacle{i_o} = [0,6; 7,6; 7,8; 0,8]; i_o = i_o + 1;
    obstacle{i_o} = [0,10; 4.5,10; 4.5,8; 0,8]; i_o = i_o + 1;
    % ice obstacles
    i_c = 1;
    %ice{i_c} = [4.5, 9; 5.5, 9; 5.5, 8; 4.5, 8]; i_c = i_c + 1;
%     ice{i_c} = [-2, -2; -2, 2; 2, 2; 2, -2]; i_c = i_c + 1;
    ice{i_c} = [6,10; 10,10; 10,6; 6,6]; i_c = i_c + 1;
    ice{i_c} = [6,0; 6,3; 10,3; 10,0]; i_c = i_c + 1;
    
    
    
%     obstacle{1} = [1, 1; 1, 9; 9, 9; 9, 1];
%     obstacle{2} = [-8, 20; -8, 25; -6, 25; -6, 20];


%     plot obstacles and ice patches
    for i_o = 1:size(obstacle, 2)
        ob_patch = patch(obstacle{i_o}(:,1), obstacle{i_o}(:,2), 'black');
        hold on
%         plot(s(:,1), s(:,2), 'ro')
    end
    
    for i_c = 1:size(ice, 2)
        ob_patch = patch(ice{i_c}(:,1), ice{i_c}(:,2), 'blue','EdgeColor','None');
        hold on
    end
        hold off

    
    in = zeros(size(s,1),2);
    for i_s = 1:size(s,1)
        for i_c = 1:size(ice, 2)
            in(i_s, 1) = in(i_s, 1) + inpolygon(s(i_s, 1), s(i_s,2), ice{i_c}(:,1), ice{i_c}(:,2));
        end
        for i_o = 1:size(obstacle,2)
            in(i_s, 2) = in(i_s, 2) + inpolygon(s(i_s, 1), s(i_s,2), obstacle{i_o}(:,1), obstacle{i_o}(:,2));
        end
    end
    
    fric = zeros(size(s,1),1);
    safe = zeros(size(s,1),1);
    for i = 1:size(s,1)
        % fric check
        if in(i,1)
            fric(i) = 0.05; % ice
        else
            fric(i) = 0.2; %regular road
        end

        % safe check
        if in(i,2)
            fric(i) = 0.05; % ice
        else
            fric(i) = 1; %regular road
        end
    end
end