clc,clear
route='F:\ʵ����\CheckboardLocalization\CheckboardLocalization\CheckboardLocalization\img1\';%����·��
d=dir([route '\*.bmp']);

WriterObj=VideoWriter('result.avi','Uncompressed AVI');%���ϳɵ���Ƶ(��������avi��ʽ)���ļ�·��
WriteObj.FrameRate = 25;
open(WriterObj);

n_frames=numel(d);% n_frames��ʾͼ��֡������
for i=1:n_frames
    if exist([route, num2str(i), '.bmp'],'file') %[route,'\Image0_w1920_h1200_fn',num2str(i,'%03d'),'.bmp'],'file'
        filename=strcat(route,num2str(i),'.bmp');
        frame=imread(filename);%��ȡͼ�񣬷��ڱ���frame��
        writeVideo(WriterObj,frame);%��frame�ŵ�����WriterObj��
    end
end
close(WriterObj);