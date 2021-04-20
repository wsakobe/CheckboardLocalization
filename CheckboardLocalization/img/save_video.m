clc,clear
route='F:\实验室\CheckboardLocalization\CheckboardLocalization\CheckboardLocalization\img';%基本路径
d=dir([route '\*.bmp']);

WriterObj=VideoWriter('result.mp4','MPEG-4');%待合成的视频(不仅限于avi格式)的文件路径
WriteObj.FrameRate = 25;
open(WriterObj);

n_frames=numel(d);% n_frames表示图像帧的总数
for i=6:n_frames - 1
    filename=strcat('F:\实验室\CheckboardLocalization\CheckboardLocalization\CheckboardLocalization\img\',num2str(i),'.bmp');
    frame=imread(filename);%读取图像，放在变量frame中
    writeVideo(WriterObj,frame);%将frame放到变量WriterObj中
end
close(WriterObj);