% best solution (best path)

function gBest2=bestPath(gBest1,S)

    % path
    temp1 = linspace(0,1,numel([S.s(1) gBest1.x S.t(1)]));
    temp2 = linspace(0,1,100);
    xx = spline(temp1, [S.s(1) gBest1.x S.t(1)], temp2);
    yy = spline(temp1, [S.s(2) gBest1.y S.t(2)], temp2); 

    % obstacle (check if path doesnt break the rule)
    wrong = 0;
    for k=1:numel(S.obs(1,:)) % Number of Obstacles
        d = sqrt((xx-S.obs(1,k)).^2+(yy-S.obs(2,k)).^2);
        m = max(1-d/S.robs(k),0);
        wrong = wrong + mean(m);
    end
    
    % gBest2 = (handling points x, handling points y, path x, path y,
    %           distance, violation value, true or false if path is allowed)
    gBest2 = struct('X', [S.s(1) gBest1.x S.t(1)], 'Y', [S.s(2) gBest1.y S.t(2)], ...
        'xx', xx, 'yy', yy, 'dis', sum(sqrt(diff(xx).^2 + diff(yy).^2)), ...
        'vio', wrong, 'allowed', (wrong==0));

