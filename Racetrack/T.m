function p = T(s0, a0, s1)
%T Transition probability.
%   T computes the probability the agent will land in state S1 when taking
%   action A0 in state S0.
    global Racetrack;

    p = 0;
    
    s_new(1:2, 1) = s0(1:2) + s0(3:4);
    
    % Check if not off-road
    if s_new(1:2)' >= 1 & s_new(1:2)' <= size(Racetrack) & Racetrack(s_new(1), s_new(2))
        s0(1:2) = s_new;
    else
        s0(3:4) = 0; % Reset speed
        if s0 == s1
            p = 1;
        end
        
        return;
    end
    
    % Compute new speed
    speed = s0(3:4);
    new_speed = speed + a0;
    new_speed(new_speed < -2) = -2;
    new_speed(new_speed > 2) = 2;
    
    if max(abs(speed)) == 1 % Speed is low
        fail = 0.1;
    elseif max(abs(speed)) == 2 % Speed is high
        fail = 0.5;
    else % No speed
        fail = 0;
    end
    
    % If fail
    if s0 == s1
        p = p + fail;
    end
    
    % No fail
    s0(3:4) = new_speed;
    if s0 == s1
        p = p + (1 - fail);
    end
end

