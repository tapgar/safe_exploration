function check = DI_map_1(s1, s2)
    
    SHOW_MAP = false;
    
    % unsafe limits
    b1 = 10;
    b2 = 10^5;
    
    %make s1 smaller then s2
    desc = sort([s1, s2]);
    s1 = desc(1); s2 = desc(2);
    
    if s1 < b1 && s2 < b1
        check = 1;
    elseif s1 > b1 && s2 > b2
        check = 1;
    else
        check = 0;
    end
    
    
    if SHOW_MAP
        hold on
        plot([0, b1 + 1], [0, 0],'k-', 'LineWidth', 3) % base line
        plot([s1, s2], [0.1, 0.1], 'g-') %movement zone
        plot([b1, b2], [-0.1, -0.1], 'r-') % unsafe zone
        xlim([0, b1 + 1])
        ylim([-0.2, 0.2])
    end
    
    
    
end