function r = reward(s, a)
%R Reward function.
%   Returns reward for state-action pair (S,A) 
    global Racetrack size_statespace filter;

    r = 5;

    s = s - filter;
    
    % If driving in opposite direction, only for track (b)
    if size(Racetrack,1) == 9
        if s(1) <= 3 && s(2) >= 3 && s(2) <= 31 && s(4) > 0
            r = -10000;
            return;
        end

        if s(1) >= 7 && s(2) >= 3 && s(2) <= 31 && s(4) < 0
            r = -10000;
            return;
        end

        if s(2) <= 3 && s(3) < 0
            r = -10000;
            return;
        end

        if s(2) >= 31 && s(3) > 0
            r = -10000;
            return;
        end
    end
    
    % Check if reached finish
    s = s + filter;
    if finished(sub2ind(size_statespace, s(1), s(2), s(3), s(4)))
        r = 200;
        return;
    end
    
    % Check if going off-road
    if offroad(s)
        r = 0;
    end
end

