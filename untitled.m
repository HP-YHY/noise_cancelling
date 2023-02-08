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
    [Y,FS]=wavread(str); %��ȡ�����ź�
    SigLength=length(Y); %�����źų���
    t=(0:SigLength-1)/FS; %ʱ�䳤��
    axes(handles.axes1);
    plot(t,Y);
    set(gca,'YLim',[-1 1]);
    xlabel('t1:ʱ��');
    ylabel('Y��ԭ����');
    title('ԭʼ�����ź�')
end



% --- Executes on button press in test2.
function test2_Callback(hObject, eventdata, handles)
% hObject    handle to test2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
str2= getappdata(handles.figure1,'str');
[Y,FS,NBITS]=wavread(str2); %��ȡ�����ź�
sound(Y);%����

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
[Y,FS]=wavread(str2); %��ȡ�����ź�
SigLength=length(Y); %�����źų���
t=(0:SigLength-1)/FS;%ʱ�䳤��
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
        xlabel('t1:ʱ��');
        title('�����������ź�')
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
        xlabel('t1:ʱ��');
        title('�����������ź�')
        spPower=sum(abs(Y( : )).^2)/length(Y( : ));
        noPower=sum(abs(xn( : )).^2)/length(xn( : ));
        spPower=10*log10(spPower);
        noPower=10*log10(noPower);
        SNR=spPower-noPower;
        set(handles.edit4,'string',num2str(SNR));
    case 4
        xs=awgn(Y,10);%�����˹��������������������
        setappdata(handles.figure1,'xs',xs);
        xn=xs-Y;
        plot(t,xs);
        set(gca,'YLim',[-2 2]);
        xlabel('t1:ʱ��');
        title('�����������ź�')
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
xs1= getappdata(handles.figure1,'xs');%��ʾ�����ź�
sound(xs1)%���������ź�

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
[Y,FS]=wavread(str2); %��ȡ�����ź�
SigLength=length(Y); %�����źų���
t1=(0:SigLength-1)/FS;
xs1= getappdata(handles.figure1,'xs');
switch popup_sel_index
    case 1
    case 2
        d=xs1;
        x=xs1;
        u=0.0002;
        N=100;                                  % ����Ӧ�˲�Ȩ��(����)
        M=length(x);
        y=zeros(1,M);                           %������ʼ��
        w=zeros(1,N);
        e=zeros(1,M);        
        for n=N:M                                %LMS�㷨
            x1=x(n:-1:n-N+1);
            y(n)=w*x1;
            e(n)=d(n)-y(n);                       %����
            w=w+2*u*e(n)*x1';
        end        
        plot(t1,e);
        set(gca,'YLim',[-2 2]);
        title('LMSȥ���������ź�')
        setappdata(handles.figure1,'e',e)
        SNR=SNR_Calc(Y,e);            % ���������
        set(handles.edit5,'string',num2str(SNR));
    case 3
        d=xs1;                                  %������Ӧd(n)=x(n)
        x=xs1;
        N=100;                                  % ����Ӧ�˲�Ȩ��(����)
        M=length(x);
        y=zeros(1,M);
        e=zeros(1,M);
        w=zeros(N,1);                           %Ȩʸ��w(n)��ʼ��
        lambda=1;                               %��������lambda
        delta=0.001;                            % ��ؾ���R�ĳ�ʼ��
        T=delta*eye(N);                         %��ؾ���
        for n=N:M                               %RLS�㷨
            xl=x(n:-1:n-N+1);                      %��ʱ����
            pi=xl'*T;                              %����غ���
            k=lambda+pi*xl;
            K=pi'/k;                               %����ʸ��
            e(n)=d(n)-w'*xl;                       %����
            w=w+K*e(n);                            %Ȩϵ���ݹ鹫ʽ
            pp=K*pi;
            T=(T-pp)/lambda;                       %�����ؾ���
            y(n)=w'*xl;
            e(n)=d(n)-y(n);
        end
        plot(t1,e);
        set(gca,'YLim',[-2 2]);
        title('RLSȥ���������ź�')
        setappdata(handles.figure1,'e',e)
        SNR=SNR_Calc(Y,e);            % �����׼���������        
        set(handles.edit5,'string',num2str(SNR));
    case 4
        IS=0.9;                                % ����ǰ���޻��γ���
        wlen=30;                               % ����֡��Ϊ25ms
        inc=10;                                 % ����֡��Ϊ10ms        
        NIS=fix((IS*FS-wlen)/inc +1);           % ��ǰ���޻���֡��
        alpha=0.5;        
        e=Weina_Im(xs1,wlen,inc,NIS,alpha) ;%ά���˲�
%         e=WienerScalart96(xs1,FS,IS);%ά���˲�
        plot(e);
        set(gca,'YLim',[-2 2]);
        title('ά���˲�ȥ���������ź�')
        setappdata(handles.figure1,'e',e)
        SNR=SNR_Calc(Y(1:length(e)),e);            % ���������
        set(handles.edit5,'string',num2str(SNR));
    case 5
        IS=0.25;                                % ����ǰ���޻��γ���
        wlen=200;                               % ����֡��Ϊ25ms
        inc=80;                                 % ����֡��Ϊ10ms
                NIS=fix((IS*FS-wlen)/inc +1);           % ��ǰ���޻���֡��
        a=4; b=0.001;                           % ���ò���a��b
        e=SpectralSub(xs1,wlen,inc,NIS,a,b);% �׼�
        plot(t1,e);
        set(gca,'YLim',[-2 2]);
        title('�׼����˲�ȥ���������ź�')
        setappdata(handles.figure1,'e',e)
        SNR=SNR_Calc(Y,e);            % ���������
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
