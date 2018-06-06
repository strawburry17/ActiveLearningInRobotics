function [deltaS] = suggestDirections(x,y,probDist,likelihood,entropy,L)
    deltaS = zeros(1,5);
    steps = [0,1; 0,-1; -1,0; 1,0; 0,0];
    
    for k = 1:5
        candX = x + steps(k,1);
        candY = y + steps(k,2);
        
        if ((candX > 3) && (candX < 21) && (candY > 3) && (candY < 21))
            L1 = Likelihood(1,candX,candY,L);
            [probDist1] = BayesianUpdate(probDist,L1,L);
            deltaEntropy1 = Entropy(probDist1,L) - entropy;
            
            L0 = Likelihood(0,candX,candY,L);
            [probDist0] = BayesianUpdate(probDist,L0,L);
            deltaEntropy0 = Entropy(probDist0,L) - entropy;
            
            deltaS(k) = probDist(x,y)*(-entropy) + (1-probDist(x,y))*(deltaEntropy1 + deltaEntropy0);
        else
            deltaS(k) = 1;
        end
        
    end
end