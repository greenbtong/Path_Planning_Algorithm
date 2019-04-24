% cost function

function [z, gBest]=costFunc(temp,S)
    
    % get the best solution (path)
    gBest = bestPath(temp,S);
    
    % get the cost (distance)
    z = gBest.dis*(1 + 100 * gBest.vio);

