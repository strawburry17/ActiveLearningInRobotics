function [likelihood] = Likelihood(measurement,x,y,L)

    if measurement
        likelihood = 0.01.*ones(L);
        likelihood(x,y) = 1;

        likelihood(x-1:x+1, y+1) = 0.5;
        likelihood(x-1:x+1, y-1) = 0.5;

        likelihood(x-2:x+2, y+2) = 1/3;
        likelihood(x-2:x+2, y-2) = 1/3;

        likelihood(x-3:x+3, y+3) = 0.25;
        likelihood(x-3:x+3, y-3) = 0.25;
    else
        likelihood = 0.99.*ones(L);
        likelihood(x,y) = 0;

        likelihood(x-1:x+1, y+1) = min(0.5,likelihood(x-1:x+1, y+1));
        likelihood(x-1:x+1, y-1) = min(0.5,likelihood(x-1:x+1, y-1));

        likelihood(x-2:x+2, y+2) = min(2/3,likelihood(x-2:x+2, y+2));
        likelihood(x-2:x+2, y-2) = min(2/3,likelihood(x-2:x+2, y-2));

        likelihood(x-3:x+3, y+3) = min(0.75,likelihood(x-3:x+3, y+3));
        likelihood(x-3:x+3, y-3) = min(0.75,likelihood(x-3:x+3, y-3));
    end

end