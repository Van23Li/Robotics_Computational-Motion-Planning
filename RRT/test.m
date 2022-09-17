%% �򵥵�RRT�㷨
%% ��ʼ��
map=im2bw(imread('map3.bmp')); % bmp����ѹ��ͼ��500x500,im2bw�ѻҶ�ͼת���ɶ�ֵͼ��01
source=[10 10]; % ��ʼ��λ��
goal=[490 490]; % Ŀ���λ��
vertex=[source;goal];
stepsize=20; % RRTÿ������
disTh=20; % ֱ��qnearest��Ŀ���qgaol����С��һ����ֵ
maxFailedAttempts = 10000;  % ����Դ���
display=true; % RRT�Ƿ�չʾ

%%  %%%% ���� %%%%%
pic_i = 1;
tic;  % ���浱ǰʱ��
if ~feasiblePoint(source,map), error('source lies on an obstacle or outside map'); end
if ~feasiblePoint(goal,map), error('goal lies on an obstacle or outside map'); end
if display, imshow(map);rectangle('position',[1 1 size(map)-1],'edgecolor','k'); end  %չʾͼ�񣬲��������м�ǵľ��α߿�E
if display, rectangle('Position',[vertex(1,2)-5,vertex(1,1)-5,10,10],'Curvature',[1,1],'FaceColor',[0 1 0]); end
if display, rectangle('Position',[vertex(2,2)-5,vertex(2,1)-5,10,10],'Curvature',[1,1],'FaceColor',[1 1 0]); end
saveas(gca,['C:\Users\21782\Documents\Van\������\LASA\Robotics_Computational-Motion-Planning\RRT\images\RRT_',num2str(pic_i)],'png');
pic_i = pic_i + 1;

RRTree=double([source -1]); % RRT ����㿪ʼ������Ϊ-1���������Ľ�������
failedAttempts=0;  % �Ѿ�����ʧ�ܵĴ���
counter=0;  % ѭ������
pathFound=false;  % �Ƿ��ҵ�·����flag
while failedAttempts<=maxFailedAttempts  % RRTѭ��
    if rand < 0.5,
        sample=rand(1,2) .* size(map);   % 50%��������ɵ�
    else
        sample=goal; % 50%������Ŀ��ǰ��
    end
    
    % ÿһ����֧���������֧
    [A, I]=min( distanceCost(RRTree(:,1:2),sample) ,[],1); % ���ֽ��������������С�����һ�У������ض�Ӧ����,[],1����ȥ��
    closestNode = RRTree(I(1),1:2); %�������������꣬�������ܶ��(1)����ȡ
    theta=atan2(sample(1)-closestNode(1),sample(2)-closestNode(2));  % �����½��ķ���
    newPoint = double(int32(closestNode(1:2) + stepsize * [sin(theta)  cos(theta)]));  % �����½�㣬�ȼ��������꣬�ټ��������
    
    if ~checkPath(closestNode(1:2), newPoint, map) % ��������㵽�½���·���Ƿ����
        failedAttempts=failedAttempts+1;
        continue;
    end
    if distanceCost(newPoint,goal)<disTh, pathFound=true;break; end % ����½���Ƿ񵽴�Ŀ��㣬��С��һ������ֵ
    [A, I2]=min( distanceCost(RRTree(:,1:2),newPoint) ,[],1); % ������Ƿ��Ѿ������������
    if distanceCost(newPoint,RRTree(I2(1),1:2))<disTh, failedAttempts=failedAttempts+1;continue; end   %����½����������У���ʧ��һ��
    RRTree=[RRTree;newPoint I(1)]; % ���½����뵽������
    failedAttempts=0;
    % ÿ��չһ���½�㣬��һ����
    if display,
        %         line([closestNode(2);newPoint(2)],[closestNode(1);newPoint(1)],'color',[0.5 0.5 0.5]);
        line([closestNode(2);newPoint(2)],[closestNode(1);newPoint(1)],'color',[0 0 1]);
        counter=counter+1;M(counter)=getframe;
        saveas(gca,['C:\Users\21782\Documents\Van\������\LASA\Robotics_Computational-Motion-Planning\RRT\images\RRT_',num2str(pic_i)],'png');
        pic_i = pic_i + 1;
    end
end
% �������һ���½����յ������
if display && pathFound
    line([closestNode(2);goal(2)],[closestNode(1);goal(1)]);
    counter=counter+1;M(counter)=getframe;
end
if display
    disp('click/press any key');
    waitforbuttonpress;
end
if ~pathFound, error('no path found. maximum attempts reached'); end

%% ����ԭ�켣
path=[goal];
prev=I(1);
while prev>0
    path=[RRTree(prev,1:2);path];
    prev=RRTree(prev,3);
end
pathLength=0;
for i=1:length(path)-1, pathLength=pathLength+distanceCost(path(i,1:2),path(i+1,1:2)); end
fprintf('processing time=%d \nPath Length=%d \n\n', toc,pathLength);   % ��ӡ����ʱ��toc��·������
% imshow(map);rectangle('position',[1 1 size(map)-1],'edgecolor','k');

% line(path(:,2),path(:,1),'color',[0.5 0.5 0.5],'linewidth',2);
line(path(:,2),path(:,1),'color',[1 0 0],'linewidth',2);
for i=1:20
    saveas(gca,['C:\Users\21782\Documents\Van\ѧϰ����\�����\RRT\RRT\RRT\RRT_images\RRT_',num2str(pic_i)],'png');
    pic_i = pic_i + 1;
end