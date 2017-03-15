function [surf] = check_surf_type(s)
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
    obstacle{i_o} = [0,10; 4.5,10; 4.5,8; 0,8];
    
    for i = 1:1:i_o
        if (s(1) > min(obstacle{i}(:,1)) && s(1) < max(obstacle{i}(:,1)) && s(2) > min(obstacle{i}(:,2)) && s(2) < max(obstacle{i}(:,2)))
            surf = 2;
            return;
        end
    end
    
    
    % ice obstacles
    i_c = 1;
    %ice{i_c} = [4.5, 9; 5.5, 9; 5.5, 8; 4.5, 8]; i_c = i_c + 1;
%     ice{i_c} = [-2, -2; -2, 2; 2, 2; 2, -2]; i_c = i_c + 1;
    ice{i_c} = [6,10; 10,10; 10,6; 6,6]; i_c = i_c + 1;
    ice{i_c} = [6,0; 6,3; 10,3; 10,0];
    
    surf = 1;
    
    for i = 1:1:i_c
        if (s(1) > min(ice{i}(:,1)) && s(1) < max(ice{i}(:,1)) && s(2) > min(ice{i}(:,2)) && s(2) < max(ice{i}(:,2)))
            surf = 0;
            return;
        end
    end
    
end