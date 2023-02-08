[Y,Fs] = audioread('1.wav'); %直接读取文件
sound(Y,Fs);%回放音频
L=length(Y);
n=length(Y);%取抽样点数
t=(0:n-1)/Fs;%显示实际时间
noise=0.3*random('Normal',0,1,L,2);
y_z=Y+noise;
sound(y_z,Fs)
IS=0.25; % 设置前导无话段长度
wlen=200; % 设置帧长为25ms
inc=80; % 设置帧移为10ms
NIS=fix((IS*Fs-wlen)/inc +1); % 求前导无话段帧数
a=4; b=0.001; % 设置参数a和b
e=SpectralSub(y_z,wlen,inc,NIS,a,b);% 谱减
sound(e,Fs)
