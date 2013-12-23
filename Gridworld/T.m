function p = T(s0, a0, s1)
%T Transition probability.
%   T returns the probability that the agent will land in state S1 if in 
%   state S0 action A0 is taken.
    global n;

    success = 0.7;
    fail = 1 - 0.7;
    
    col = ceil(s0 / n); 
	row = mod(s0,n);
    if row == 0
        row = n;
    end
    
    neighbor = zeros(4,1);
    neighbor(1) = (s0 - 1) == s1 & row > 1;
    neighbor(2) = (s0 + n) == s1 & col < n;
    neighbor(3) = (s0 + 1) == s1 & row < n;
    neighbor(4) = (s0 - n) == s1 & col > 1;
    
    prob = zeros(4,1);
    prob(:) = fail / 3;
    prob(a0) = success;
    
    edge = zeros(4,1);
    edge(1) = row == 1;
    edge(2) = col == n;
    edge(3) = row == n;
    edge(4) = col == 1;
    
    if s0 == s1
        p = prob' * edge;
    elseif sum(neighbor) == 1
        p = prob' * neighbor;
    else
        p = 0;
    end
end

