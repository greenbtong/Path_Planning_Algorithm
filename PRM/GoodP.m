% check if point in the map is valid

function check=GoodP(point,map)
check=true;
% check if collission free in the map
if ~(point(1)>=1 && point(1)<=size(map,1) && point(2)>=1 && point(2)<=size(map,2) && map(point(1),point(2))==1)
    check=false;
end
