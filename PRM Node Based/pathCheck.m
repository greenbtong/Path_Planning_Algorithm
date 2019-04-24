% check if path is valid

function check=pathCheck(n,n_P,map)
check=true;
d=atan2(n_P(1)-n(1),n_P(2)-n(2));

for r=0:0.5:sqrt(sum((n-n_P).^2))
    posCheck=n+r.*[sin(d) cos(d)];
    
    % check if valid
    if ~(GoodP(ceil(posCheck),map) && GoodP(floor(posCheck),map) && GoodP([ceil(posCheck(1)) floor(posCheck(2))],map) && GoodP([floor(posCheck(1)) ceil(posCheck(2))],map))
        check=false;
        break;
    end
    
    % check if new path is valid
    if ~GoodP(n_P,map), 
        check=false; 
    end
end