[Y,Fs] = audioread('1.wav'); %ֱ�Ӷ�ȡ�ļ�
sound(Y,Fs);%�ط���Ƶ
L=length(Y);
n=length(Y);%ȡ��������
t=(0:n-1)/Fs;%��ʾʵ��ʱ��
noise=0.3*random('Normal',0,1,L,2);
y_z=Y+noise;
sound(y_z,Fs)
IS=0.25; % ����ǰ���޻��γ���
wlen=200; % ����֡��Ϊ25ms
inc=80; % ����֡��Ϊ10ms
NIS=fix((IS*Fs-wlen)/inc +1); % ��ǰ���޻���֡��
a=4; b=0.001; % ���ò���a��b
e=SpectralSub(y_z,wlen,inc,NIS,a,b);% �׼�
sound(e,Fs)
