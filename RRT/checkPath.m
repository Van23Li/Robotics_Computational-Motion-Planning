% 检测当前树结点到新结点的方向上，所有点在地图内部
function feasible=checkPath(n,newPos,map)
feasible=true;
dir=atan2(newPos(1)-n(1),newPos(2)-n(2));
for r=0:0.5:sqrt(sum((n-newPos).^2))
    posCheck=n+r.*[sin(dir) cos(dir)];
    % 分别检测取点朝正和朝负无穷大四舍五入的结果是否满足在图像中
    if ~(feasiblePoint(ceil(posCheck),map) && feasiblePoint(floor(posCheck),map) && ... 
            feasiblePoint([ceil(posCheck(1)) floor(posCheck(2))],map) && feasiblePoint([floor(posCheck(1)) ceil(posCheck(2))],map))
        feasible=false;break;
    end
    if ~feasiblePoint(newPos,map), feasible=false; end
end