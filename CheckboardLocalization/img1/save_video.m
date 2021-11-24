clc,clear
route='F:\实验室\CheckboardLocalization\CheckboardLocalization\CheckboardLocalization\img1\';%基本路径
d=dir([route '\*.bmp']);

WriterObj=VideoWriter('result.avi','Uncompressed AVI');%待合成的视频(不仅限于avi格式)的文件路径
WriteObj.FrameRate = 25;
open(WriterObj);

n_frames=numel(d);% n_frames表示图像帧的总数
for i=1:n_frames
    if exist([route, num2str(i), '.bmp'],'file') %[route,'\Image0_w1920_h1200_fn',num2str(i,'%03d'),'.bmp'],'file'
        filename=strcat(route,num2str(i),'.bmp');
        frame=imread(filename);%读取图像，放在变量frame中
        writeVideo(WriterObj,frame);%将frame放到变量WriterObj中
    end
end
close(WriterObj);