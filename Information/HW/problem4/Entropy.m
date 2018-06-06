function entropy = Entropy(p,L)
    for i = 1:L
        for j = 1:L
            if p(i,j) < 0.001
                p(i,j) = 0.001;
            end
        end
    end
    entropy = sum(sum(-p.*log(p)));
end