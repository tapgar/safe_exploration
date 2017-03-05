    function c = STOMP_COST(state, end_state) %this should be part of env?
        pos = state(:,1);
        vel = state(:,2);
        % penalized for being far from goal and having high velocity
        c = 0;%0.0000001.*(end_state(1) - pos).^2 ; %+ (end_state(2) - vel).^2; 
    end