% ����Ŀ�����  ?

function feasible=feasiblePoint(point,map)
feasible=true;
% ���õ��Ƿ�Ϊ��ײ��Ͳ��ڵ�ͼ�ڲ�
if ~(point(1)>=1 && point(1)<=size(map,1) && point(2)>=1 && point(2)<=size(map,2) && map(point(1),point(2))==1)
    feasible=false;
end