function check = dubbins_map(s)
% check all states to see if they are in danger zone!
    

    % set danger area (we can make this an image and do different checking)
    obstacle{1} = [5, 5; 5, 6; 6, 6; 6, 5];
    obstacle{1} = [1, 1; 1, 9; 9, 9; 9, 1];
    obstacle{2} = [-8, 20; -8, 25; -6, 25; -6, 20];
    in = zeros(size(s,1),2);
%     for i_o = 1:size(obstacle, 1)
%         ob_patch = patch(obstacle{i_o}(:,1), obstacle{i_o}(:,2), 'black');
%         hold on
% %         plot(s(:,1), s(:,2), 'ro')
%     end
%         hold off

    

    for i_s = 1:size(s,1)
        for i_o = 1:size(obstacle,2)
            in(i_s) = in(i_s) + inpolygon(s(i_s, 1), s(i_s,2), obstacle{i_o}(:,1), obstacle{i_o}(:,2));
        end
    end

    if any(in)
        check = 0.05; % ice
    else
        check = 10; %regular road
    end
    
    
    
end