function b = offroad(s)
%OFFROAD Checks if vehicle is driving off road.
%   Check if vehicle in state S will land off the road at the next timestep
%   with its current velocity

    global Racetrack;

    new_pos = s(1:2) + s(3:4);
    b = max(new_pos' > size(Racetrack)) || max(new_pos' < 1) || ~Racetrack(new_pos(1), new_pos(2));
end

