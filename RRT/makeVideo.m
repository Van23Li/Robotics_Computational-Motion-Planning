% ��ͼƬ֡������Ƶ���ļ�
close all;
clear all;

fps = 30; % ��Ƶ֡�ʣ�������Ƶ�ٶ�
framesPath = ['.\RRT_images\']; % ͼ����������·��
videosPath = ['.\RRT_video\']; % ������Ƶ�ļ���
if ~exist(videosPath, 'dir')
    mkdir(videosPath);
end
files_struct = dir(fullfile(framesPath));
files_struct(1:2) = []; % ɾ��.��..�ļ���
num_seqs = length(files_struct);

fprintf('Total number of videos: %d\n', num_seqs);

t_video_make_st = clock;

% parfor seq_count = 1 : num_seqs
% for seq_count = 1 : num_seqs
video_name = 'RRT_images';
%     target_frames = dir([framesPath video_name '/*.png']); % ͼƬ��׺��png
start_frame = 1; % ��ʼ֡
%     end_frame = length(target_frames); % ����֡
end_frame = num_seqs; % ����֡
video = [videosPath video_name '.avi']; % ������������Ƶ
if exist(video, 'file') % �ļ����ڷ���2
    delete(video); % ��֤������Ƶ����ʱ��ʼʱ��Ϊ��
end
fprintf(['Start making video: ' video_name]);
video_object = VideoWriter(video);  % ����avi��Ƶ�ļ�����
video_object.FrameRate = fps;
open(video_object); % ���ļ��ȴ�д��
for frame = start_frame : end_frame % ����ͼƬ
    % fileName=sprintf('%04d',i); % �����ļ�������
    frames = imread([framesPath 'RRT_' num2str(frame),'.png']);
    writeVideo(video_object, frames); % д������
end
close(video_object); % �ر��ļ�
fprintf(' End!\n');
% end
t_video_make_end = clock;
t_video_make = etime(t_video_make_end, t_video_make_st);
fprintf('End making videos, total time spent: %.2fs. All videos are in: %s\n', t_video_make, videosPath);