%% radBasFcn.m
function  k = radBasFcn(x,xprime)
    if (isrow(x) && isrow(xprime))
        % transpose both vectors
        x = x';
        xprime = xprime';
        k = exp(-0.5.*((x-xprime)'*(x-xprime))^2);
    elseif (iscolumn(x) && iscolumn(xprime))
        k = exp(-0.5.*((x-xprime)'*(x-xprime))^2);
    else
        if isrow(x)
            x = x';
            k = exp(-0.5.*((x-xprime)'*(x-xprime))^2);
        else
            xprime = xprime';
            k = exp(-0.5.*((x-xprime)'*(x-xprime))^2);
        end
    end
end