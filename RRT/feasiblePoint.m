% 检测点的可行性  ?

function feasible=feasiblePoint(point,map)
feasible=true;
% 检查该点是否为碰撞点和并在地图内部
if ~(point(1)>=1 && point(1)<=size(map,1) && point(2)>=1 && point(2)<=size(map,2) && map(point(1),point(2))==1)
    feasible=false;
end