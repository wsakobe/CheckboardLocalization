clc,clear
route='F:\ʵ����\CheckboardLocalization\CheckboardLocalization\CheckboardLocalization\img1';%����·��
d=dir([route '\*.bmp']);

WriterObj=VideoWriter('result.mp4','MPEG-4');%���ϳɵ���Ƶ(��������avi��ʽ)���ļ�·��
WriteObj.FrameRate = 25;
open(WriterObj);

n_frames=numel(d);% n_frames��ʾͼ��֡������
for i=0:n_frames - 1
    if (i>93 && i<98) 
        continue;
    end
    if (i==99) 
        continue;
    end
    filename=strcat('F:\ʵ����\CheckboardLocalization\CheckboardLocalization\CheckboardLocalization\img1\',num2str(i),'.bmp');
    frame=imread(filename);%��ȡͼ�񣬷��ڱ���frame��
    writeVideo(WriterObj,frame);%��frame�ŵ�����WriterObj��
end
close(WriterObj);