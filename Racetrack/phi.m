function f = phi(ind)
%PHI Features for a state.
%   PHI(S) returns a feature vector for state with state index IND.
    global Racetrack size_statespace filter num_features;

    s = zeros(4,1);
    [s(1), s(2), s(3), s(4)] = ind2sub(size_statespace, ind);
    s = s - filter;
    
    % Features for (1) Finish reached, (2) Off-road and (3) Speed
    f = zeros(num_features,1);

    f(1) = finished(ind);
    f(2) = ~offroad(s);
    f(3) = max(abs(s(3:4)) / 2);
    f(3) = -1 * f(3) + 1;
    % Feature (4) for driving in opposite direction, only for track (b)
    if size(Racetrack,1) == 9
        if s(1) <= 3 && s(2) >= 3 && s(2) <= 31 && s(4) > 0
            f(4) = 1;
            return;
        end

        if s(1) >= 7 && s(2) >= 3 && s(2) <= 31 && s(4) < 0
            f(4) = 1;
            return;
        end

        if s(2) <= 3 && s(3) < 0
            f(4) = 1;
            return;
        end

        if s(2) >= 31 && s(3) > 0
            f(4) = 1;
            return;
        end
    end
    
    f(4) = ~f(4);
end

