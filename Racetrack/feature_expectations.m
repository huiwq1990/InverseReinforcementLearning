function mu = feature_expectations(P, discount, D, policy, num_samples, num_steps)
%FEATURE_EXPECTATIONS Expected features.
%   FEATURE_EXPECTATIONS produces the expected accumulated feature counts
%   if POLICY is followed.
    global num_features;
    Mu = zeros(num_samples, num_features);

    for i = 1:num_samples
        clear trajectory;
        cumprob = cumsum(D(:));
        r = rand();
        s = find(cumprob > r, 1);
        
        Mu(i,:) = phi(s)';

        for t = 2:num_steps
            a = policy(s);
            trajectory(t-1,:) = [s a];
            
            cumprob = cumsum(P{a}(s,:));
            r = rand();
            
            s = find(cumprob > r, 1);
           
            Mu(i,:) = Mu(i,:) + discount ^ (t-1) * phi(s)';
            
            if finished(s)
                break;
            end
        end
    end
    
    mu = mean(Mu)';
end

