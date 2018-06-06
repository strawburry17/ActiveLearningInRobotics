function [inRange, likelihood] = Measurement(x,y,goalX,goalY,inRange,likelihood,L)

    if inRange
        likelihood = 0.01.*ones(L);
        for k = 1:4
               likelihood(goalX+(k-1),goalY-(k-1):goalY+(k-1)) = (1-(k-1)*.25);
        end
    else % if not in range
        if (abs(goalX - x) < 4) && (abs(goalY - y) < 4) && (abs(goalY - y) < abs(goalX - x))
           for k = 1:4
               likelihood(goalX+(k-1),goalY-(k-1):goalY+(k-1)) = (1-(k-1)*.25)*2;
           end
           
           inRange = 1;
        end
    end

end