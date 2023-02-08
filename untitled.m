function varargout = untitled(varargin)
%UNTITLED M-file for untitled.fig
%      UNTITLED, by itself, creates a new UNTITLED or raises the existing
%      singleton*.
%
%      H = UNTITLED returns the handle to a new UNTITLED or the handle to
%      the existing singleton*.
%
%      UNTITLED('Property','Value',...) creates a new UNTITLED using the
%      given property value pairs. Unrecognized properties are passed via
%      varargin to untitled_OpeningFcn.  This calling syntax produces a
%      warning when there is an existing singleton*.
%
%      UNTITLED('CALLBACK') and UNTITLED('CALLBACK',hObject,...) call the
%      local function named CALLBACK in UNTITLED.M with the given input
%      arguments.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help untitled

% Last Modified by GUIDE v2.5 09-May-2006 17:38:13

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
    'gui_Singleton',  gui_Singleton, ...
    'gui_OpeningFcn', @untitled_OpeningFcn, ...
    'gui_OutputFcn',  @untitled_OutputFcn, ...
    'gui_LayoutFcn',  [], ...
    'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before untitled is made visible.
function untitled_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   unrecognized PropertyName/PropertyValue pairs from the
%            command line (see VARARGIN)

% Choose default command line output for untitled
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes untitled wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = untitled_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in select.
function select_Callback(hObject, eventdata, handles)
% hObject    handle to select (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

[filename, pathname] = ...
    uigetfile({'*.wav'},'File Selector');
if isequal(filename,0)|isequal(pathname,0)
    return;
else
    str=[pathname,filename];
    setappdata(handles.figure1,'str',str);
    [Y,FS]=wavread(str); %读取声音信号
    SigLength=length(Y); %计算信号长度
    t=(0:SigLength-1)/FS; %时间长度
    axes(handles.axes1);
    plot(t,Y);
    set(gca,'YLim',[-1 1]);
    xlabel('t1:时间');
    ylabel('Y（原音）');
    title('原始语音信号')
end



% --- Executes on button press in test2.
function test2_Callback(hObject, eventdata, handles)
% hObject    handle to test2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
str2= getappdata(handles.figure1,'str');
[Y,FS,NBITS]=wavread(str2); %读取声音信号
sound(Y);%播放

% --- Executes on selection change in popupmenu1.
function popupmenu1_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = get(hObject,'String') returns popupmenu1 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu1
axes(handles.axes2);
popup_sel_index = get(handles.popupmenu1, 'Value');
str2= getappdata(handles.figure1,'str');
[Y,FS]=wavread(str2); %读取声音信号
SigLength=length(Y); %计算信号长度
t=(0:SigLength-1)/FS;%时间长度
switch popup_sel_index
    case 1
    case 2
        x1=1/2*sin(2*pi*5000*t);
        xn=x1+0.02*x1.*randn(size(x1));
        xn=xn';
        xs=Y+xn;
        setappdata(handles.figure1,'xs',xs);
        plot(t,xs);
        set(gca,'YLim',[-2 2]);
        xlabel('t1:时间');
        title('加噪后的语音信号')
        spPower=sum(abs(Y( : )).^2)/length(Y( : ));
        noPower=sum(abs(xn( : )).^2)/length(xn( : ));
        spPower=10*log10(spPower);
        noPower=10*log10(noPower);
        SNR=spPower-noPower;
        set(handles.edit4,'string',num2str(SNR));
        
    case 3
        xn=sin(2*pi*120*t);
        xn=xn';
        xs=Y+xn;
        setappdata(handles.figure1,'xs',xs);
        plot(t,xs);
        set(gca,'YLim',[-2 2]);
        xlabel('t1:时间');
        title('加噪后的语音信号')
        spPower=sum(abs(Y( : )).^2)/length(Y( : ));
        noPower=sum(abs(xn( : )).^2)/length(xn( : ));
        spPower=10*log10(spPower);
        noPower=10*log10(noPower);
        SNR=spPower-noPower;
        set(handles.edit4,'string',num2str(SNR));
    case 4
        xs=awgn(Y,10);%加入高斯白噪声，信噪比随机产生
        setappdata(handles.figure1,'xs',xs);
        xn=xs-Y;
        plot(t,xs);
        set(gca,'YLim',[-2 2]);
        xlabel('t1:时间');
        title('加噪后的语音信号')
        spPower=sum(abs(Y( : )).^2)/length(Y( : ));
        noPower=sum(abs(xn( : )).^2)/length(xn( : ));
        spPower=10*log10(spPower);
        noPower=10*log10(noPower);
        SNR=spPower-noPower;
        set(handles.edit4,'string',num2str(SNR))
end

% --- Executes on button press in test3.
function test3_Callback(hObject, eventdata, handles)
% hObject    handle to test3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
xs1= getappdata(handles.figure1,'xs');%显示噪声信号
sound(xs1)%播放噪声信号

% --- Executes during object creation, after setting all properties.
function popupmenu1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit1_Callback(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit1 as text
%        str2double(get(hObject,'String')) returns contents of edit1 as a double


% --- Executes during object creation, after setting all properties.
function edit1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit4_Callback(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit4 as text
%        str2double(get(hObject,'String')) returns contents of edit4 as a double


% --- Executes during object creation, after setting all properties.
function edit4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit5_Callback(hObject, eventdata, handles)
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit5 as text
%        str2double(get(hObject,'String')) returns contents of edit5 as a double


% --- Executes during object creation, after setting all properties.
function edit5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end






function edit2_Callback(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit2 as text
%        str2double(get(hObject,'String')) returns contents of edit2 as a double


% --- Executes during object creation, after setting all properties.
function edit2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit3_Callback(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit3 as text
%        str2double(get(hObject,'String')) returns contents of edit3 as a double


% --- Executes during object creation, after setting all properties.
function edit3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in popupmenu2.
function popupmenu2_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = get(hObject,'String') returns popupmenu2 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu2
axes(handles.axes3);
popup_sel_index = get(handles.popupmenu2, 'Value');
str2= getappdata(handles.figure1,'str');
[Y,FS]=wavread(str2); %读取声音信号
SigLength=length(Y); %计算信号长度
t1=(0:SigLength-1)/FS;
xs1= getappdata(handles.figure1,'xs');
switch popup_sel_index
    case 1
    case 2
        d=xs1;
        x=xs1;
        u=0.0002;
        N=100;                                  % 自适应滤波权数(阶数)
        M=length(x);
        y=zeros(1,M);                           %参数初始化
        w=zeros(1,N);
        e=zeros(1,M);        
        for n=N:M                                %LMS算法
            x1=x(n:-1:n-N+1);
            y(n)=w*x1;
            e(n)=d(n)-y(n);                       %误差函数
            w=w+2*u*e(n)*x1';
        end        
        plot(t1,e);
        set(gca,'YLim',[-2 2]);
        title('LMS去噪后的语音信号')
        setappdata(handles.figure1,'e',e)
        SNR=SNR_Calc(Y,e);            % 计算信噪比
        set(handles.edit5,'string',num2str(SNR));
    case 3
        d=xs1;                                  %期望响应d(n)=x(n)
        x=xs1;
        N=100;                                  % 自适应滤波权数(阶数)
        M=length(x);
        y=zeros(1,M);
        e=zeros(1,M);
        w=zeros(N,1);                           %权矢量w(n)初始化
        lambda=1;                               %遗忘因子lambda
        delta=0.001;                            % 相关矩阵R的初始化
        T=delta*eye(N);                         %相关矩阵
        for n=N:M                               %RLS算法
            xl=x(n:-1:n-N+1);                      %延时函数
            pi=xl'*T;                              %互相关函数
            k=lambda+pi*xl;
            K=pi'/k;                               %增益矢量
            e(n)=d(n)-w'*xl;                       %误差函数
            w=w+K*e(n);                            %权系数递归公式
            pp=K*pi;
            T=(T-pp)/lambda;                       %误差相关矩阵
            y(n)=w'*xl;
            e(n)=d(n)-y(n);
        end
        plot(t1,e);
        set(gca,'YLim',[-2 2]);
        title('RLS去噪后的语音信号')
        setappdata(handles.figure1,'e',e)
        SNR=SNR_Calc(Y,e);            % 计算谱减后的信噪比        
        set(handles.edit5,'string',num2str(SNR));
    case 4
        IS=0.9;                                % 设置前导无话段长度
        wlen=30;                               % 设置帧长为25ms
        inc=10;                                 % 设置帧移为10ms        
        NIS=fix((IS*FS-wlen)/inc +1);           % 求前导无话段帧数
        alpha=0.5;        
        e=Weina_Im(xs1,wlen,inc,NIS,alpha) ;%维纳滤波
%         e=WienerScalart96(xs1,FS,IS);%维纳滤波
        plot(e);
        set(gca,'YLim',[-2 2]);
        title('维纳滤波去噪后的语音信号')
        setappdata(handles.figure1,'e',e)
        SNR=SNR_Calc(Y(1:length(e)),e);            % 计算信噪比
        set(handles.edit5,'string',num2str(SNR));
    case 5
        IS=0.25;                                % 设置前导无话段长度
        wlen=200;                               % 设置帧长为25ms
        inc=80;                                 % 设置帧移为10ms
                NIS=fix((IS*FS-wlen)/inc +1);           % 求前导无话段帧数
        a=4; b=0.001;                           % 设置参数a和b
        e=SpectralSub(xs1,wlen,inc,NIS,a,b);% 谱减
        plot(t1,e);
        set(gca,'YLim',[-2 2]);
        title('谱减法滤波去噪后的语音信号')
        setappdata(handles.figure1,'e',e)
        SNR=SNR_Calc(Y,e);            % 计算信噪比
        set(handles.edit5,'string',num2str(SNR));
end


% --- Executes on button press in test4.
function test4_Callback(hObject, eventdata, handles)
% hObject    handle to test4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
xs2= getappdata(handles.figure1,'e');
sound(xs2)


% --- Executes during object creation, after setting all properties.
function popupmenu2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
