function f = finished(ind)
%FINISHED Check if agent is over finish line.
%   F is true only if state with index IND is a state behind the finish line
    global Racetrack size_statespace;

    s = zeros(4,1);
    [s(1), s(2), s(3), s(4)] = ind2sub(size_statespace, ind);
    
    if size(Racetrack,1) == 9
        f = s(1) > 6 && s(2) <= 3;
    else
        f = s(1) <= 1;
    end
end

