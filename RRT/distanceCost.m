% 测量树结点到随机点或者目标点的距离代价
function h=distanceCost(a,b)
h = sqrt((a(:,1)-b(:,1)).^2 + (a(:,2)-b(:,2)).^2 );