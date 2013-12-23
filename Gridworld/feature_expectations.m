function mu = feature_expectations(P, discount, D, policy, num_samples, num_steps)
%FEATURE_EXPECTATIONS Expected features.
%   FEATURE_EXPECTATIONS produces the expected accumulated feature counts
%   if the given POLICY is followed.
    global num_macrocells;

    Mu = zeros(num_samples, num_macrocells);

    for i = 1:num_samples
        trajectory = zeros(num_steps,1);

        cumprob = cumsum(D);
        r = rand();
        s = find(cumprob > r, 1);
        trajectory(1) = s;
        Mu(i,:) = phi(s)';

        for t = 2:num_steps
            a = policy(s);
            cumprob = cumsum(P{a}(s,:));
            r = rand();
            
            s = find(cumprob > r, 1);

            trajectory(t) = s;
            Mu(i,:) = Mu(i,:) + discount ^ (t-1) * phi(s)';
        end
    end
    
    mu = mean(Mu)';
end

