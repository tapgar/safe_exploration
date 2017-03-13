function check = dubbins_map(s)
% check all states to see if they are in danger zone!
    

    % set danger area (we can make this an image and do different checking)
    obstacle = [5, 5; 5, 6; 6, 6; 6, 5];
    in = zeros(size(s,1),1);
%     ob_patch = patch(obstacle(:,1), obstacle(:,2), 'black');
%     hold on
%     plot(s(:,1), s(:,2), 'ro')
%     hold off

    

    for i_s = 1:size(s,1)
        in(i_s) = inpolygon(s(i_s, 1), s(i_s,2), obstacle(:,1), obstacle(:,2));
        
    end

    if any(in)
        check = false;
    else
        check = true;
    end
    
    
    
end