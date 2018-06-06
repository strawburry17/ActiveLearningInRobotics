%% Problem 4: Infotaxis for door localization
% Dan Lynch
% I collaborated with Tito Fernandez on this problem.
clear;
close all;
clc;

%% System description
L = 25; % dimension of square gridworld
N = L*L; % number of locations in gridworld

goalX = randi([4,L-4]); goalY = randi([4,L-4]); % coordinates of the goal (unknown to the agent)

ent_thresh = 0.01; % convergence threshold (entropy)

startX = randi([4,L-4]); startY = randi([4,L-4]); % agent starts in a random spot

%% Initialize
fprintf("Initializing...\n");
% Intialize the prior as a uniform distribution
probDist = 1/N.*ones(L);

% Initialize entropy distribution
entropy = Entropy(probDist,L);

% Initialize likelihood map
likelihood = 0.99.*ones(L);

% initialize the number of unvisited squares:
nUnvisited = N;

% initialize the time value:
t = 0;

% initialize measurement:
inRange = 0;

%% Iterate
fprintf("Iterating...\n");

% move to the start position
x = startX; y = startY;

while (abs(entropy) > ent_thresh || (x ~= goalX) || (y ~= goalY))
% while t < 2
    if (x == goalX) && (y == goalY)
        fprintf("found door\n");
        break;
    end
    fprintf("t = %d\n",t);
    fprintf("\tn_visited = %d\n",N - nUnvisited);

    % log current position:
    xLog(t+1) = x; yLog(t+1) = y;
    nUnvisited = nUnvisited - 1;
    
    % take measurement:
    [inRange, likelihood] = Measurement(x,y,goalX,goalY,inRange,likelihood,L);
    measLog(t+1) = inRange;
    fprintf("\tmeasurement = %d\n",inRange);
    
    % update the posterior:
    likelihood = Likelihood(inRange,x,y,L);
    
    [probDist] = BayesianUpdate(probDist,likelihood,L);
    [~,suspectDoorLocX] = max(probDist(:,1))
    [~,suspectDoorLocY] = max(probDist(1,:))
%     fprintf("max(probDist) = %d, %d\n",[suspectDoorLocX,suspectDoorLocY]);
    
    % recalculate entropy:
    entropy = Entropy(probDist,L);
    
    deltaS = suggestDirections(x,y,probDist,likelihood,entropy,L)
    
    [minEdS, move] = min(deltaS)
    
    

%     figure;
    hold on
    surface(likelihood')
    plot3(xLog+0.5,yLog+0.5,ones(length(xLog)),'ko','MarkerFaceColor', 'k','MarkerSize',14);
    plot3(x+0.5,y+0.5,1,'ro','MarkerFaceColor', 'r','MarkerSize',14);
    plot3(goalX+0.5,goalY+0.5,1,'go','MarkerFaceColor', 'g','MarkerSize',14);
    hold off
    view(0,90)
    xlabel('x');ylabel('y');
    drawnow
    
    % apply input
    switch move
        case 1 % up
            if ((y+1) > L-4)
                fprintf("\ttop wall\n");
            else
                y = y+1;
                fprintf("\tup\n");
            end
        case 2 % down
            if ((y-1) < 4)
                fprintf("\tbottom wall\n");
            else
                fprintf("\tdown\n");
                y = y-1;
            end
        case 3 % left
            if ((x-1) < 4)
                fprintf("\tleft wall\n");
            else
                x = x-1;
                fprintf("\tleft\n");
            end
        case 4 % right
            if ((x+1) > L-4)
                fprintf("\tright wall\n");
            else
                fprintf("\tright\n");
                x = x+1;
            end
        case 5 % stay
            fprintf("\tstay\n");
        otherwise
            fprintf("error choosing move\n");
    end

    t = t+1;
    
end