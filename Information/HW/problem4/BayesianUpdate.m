function [probDist,entropyDist] = BayesianUpdate(probDist,likelihood,L)

    renorm = sum(sum(likelihood.*probDist));

    for i = 1:L
        for j = 1:L
            probDist(i,j) = likelihood(i,j)*probDist(i,j)/renorm;
        end
    end

end