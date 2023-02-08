function varargout = myGui(varargin)
% MYGUI MATLAB code for myGui.fig
%      MYGUI, by itself, creates a new MYGUI or raises the existing
%      singleton*.
%
%      H = MYGUI returns the handle to a new MYGUI or the handle to
%      the existing singleton*.
%
%      MYGUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in MYGUI.M with the given input arguments.
%
%      MYGUI('Property','Value',...) creates a new MYGUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before myGui_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to myGui_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help myGui

% Last Modified by GUIDE v2.5 06-Nov-2022 13:02:52

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;    %��ֻ֤��һ�����ڣ�Ϊ0����Դ򿪶������
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @myGui_OpeningFcn, ...
                   'gui_OutputFcn',  @myGui_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
 %�ýṹ�����˸�GUI��״��������gui�����֡�����ʵ������ʼ����������������������Լ��ص�������
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
    %�ֱ�ȡ��figure�Ͱ����Ŀؼ���CreateFcn�ص����������������ؼ���
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before myGui is made visible.
function myGui_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to myGui (see VARARGIN)

% Choose default command line output for myGui
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes myGui wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = myGui_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global myspeech1 Fs
Fs = 44100;
t = eval(get(handles.edit1,'String'));  %¼��ʱ��
recObj = audiorecorder(Fs,16,1);  %����һ��¼����������˷�
recordblocking(recObj,t);  %��ʼ¼��
play(recObj);
myspeech1 = getaudiodata(recObj); 

% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global myspeech1
sound(myspeech1,44100);

% --- Executes on button press in pushbutton3.
function pushbutton3_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global myspeech1
axes(handles.axes1);  %��λ������ϵ1
plot((0:length(myspeech1)-1)/44100,myspeech1);  %���������źŲ���
xlabel('t(s)');
ylabel('��ֵ');
legend('ԭʼ��������');
axes(handles.axes2);
Y = abs(fft(myspeech1,length(myspeech1))/(length(myspeech1))/2);  %Ƶ�׷���
f = (0:length(myspeech1)/2-1)*44100/length(myspeech1);  %Ƶ��
plot(f,Y(1:length(myspeech1)/2));  %���������ź�Ƶ��ͼ
xlabel('f(Hz)');
ylabel('��ֵ');
legend('ԭʼ����Ƶ��');


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


% --- Executes on selection change in popupmenu1.
function popupmenu1_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu1 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu1


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

% --- Executes on button press in pushbutton4.
function pushbutton4_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global myspeech x Bz Az wn
switch get(handles.popupmenu1,'Value')
    case 1
        x = filter(wn,1,myspeech);% �˲�
    case 2
        x = filter(Bz, Az, myspeech);
end
% --- Executes on button press in pushbutton5.
function pushbutton5_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global x
sound(x,44100);

% --- Executes on button press in pushbutton6.
function pushbutton6_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global x
axes(handles.axes3);  %��λ������ϵ3
plot((0:length(x)-1)/44100,x);  %���������źŲ���
xlabel('t(s)');
ylabel('��ֵ');
axes(handles.axes4);
Y = abs(fft(x,length(x))/(length(x))/2);  %Ƶ�׷���
f = (0:length(x)/2-1)*44100/length(x);  %Ƶ��
plot(f,Y(1:length(x)/2));  %���������ź�Ƶ��ͼ
xlabel('f(Hz)');
ylabel('��ֵ');

% --- Executes on button press in pushbutton8.
function pushbutton8_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



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



function edit6_Callback(hObject, eventdata, handles)
% hObject    handle to edit6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit6 as text
%        str2double(get(hObject,'String')) returns contents of edit6 as a double


% --- Executes during object creation, after setting all properties.
function edit6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton9.
function pushbutton9_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global Bz Az wn Fs
fp = eval(get(handles.edit3,'String'));
fs = eval(get(handles.edit4,'String'));
Ap = eval(get(handles.edit5,'String'));
As = eval(get(handles.edit6,'String'));
switch get(handles.popupmenu1,'Value')
    case 1  %FIR
        switch get(handles.popupmenu2,'Value')
            case 1  %��ͨ
                mags=[1,0];% ��ͨ��ֵ
                fcuts=[fp,fs];% �߽�Ƶ��
                dev=[(10^(Ap/20)-1)/(10^(Ap/20)+1),10^(-As/20)];
                [N,Wn,beta,ftype]=kaiserord(fcuts,mags,dev,Fs);% ����FIR�˲�������
                wn=fir1(N,Wn,ftype,kaiser(N+1,beta));% FIR�˲������
            case 2  %��ͨ
                mags=[0,1];% ��ͨ��ֵ
                fcuts=[fs,fp];% �߽�Ƶ��
                dev=[10^(-As/20),(10^(Ap/20)-1)/(10^(Ap/20)+1)];
                [N,Wn,beta,ftype]=kaiserord(fcuts,mags,dev,Fs);% ����FIR�˲�������
                wn=fir1(N,Wn,ftype,kaiser(N+1,beta));% FIR�˲������
            case 3  %��ͨ
                mags=[0,1,0];% ��ͨ��ֵ
                fcuts=[fs(1),fp(1),fp(2),fs(2)];% �߽�Ƶ��
                dev=[10^(-As/20),(10^(Ap/20)-1)/(10^(Ap/20)+1),10^(-As/20)];% ����ƫ����
                [N,Wn,beta,ftype]=kaiserord(fcuts,mags,dev,Fs);% ����FIR�˲�������
                wn=fir1(N,Wn,ftype,kaiser(N+1,beta));% FIR�˲������
        end
    case 2  %IIR
        wp = 2*pi*fp/Fs;  %��ģ��Ƶ��ת��Ϊ����Ƶ��
        ws = 2*pi*fs/Fs;
        Wp = 2*tan(wp/2);
        Ws = 2*tan(ws/2);
        switch get(handles.popupmenu2,'Value')
            case 1  %��ͨ
               [N,wc]=buttord(Wp,Ws,Ap,As,'s'); %������Ӧ��ģ���˲�������N��3dB��ֹƵ��
               [B,A]=butter(N,wc,'s');   %������Ӧ��ģ���˲���ϵͳ����
               [Bz,Az]=bilinear(B,A,1);  %��˫���Ա任����ģ���˲���ת���������˲���

            case 2  %��ͨ
               [N,wc]=buttord(Wp,Ws,Ap,As,'s'); %������Ӧ��ģ���˲�������N��3dB��ֹƵ��
               [B,A]=butter(N,wc,'high','s');   %������Ӧ��ģ���˲���ϵͳ����
               [Bz,Az]=bilinear(B,A,1);  %��˫���Ա任����ģ���˲���ת���������˲���

            case 3  %��ͨ
               [N,wc]=buttord(Wp,Ws,Ap,As,'s'); %������Ӧ��ģ���˲�������N��3dB��ֹƵ��
               [B,A]=butter(N,wc,'bandpass','s');   %������Ӧ��ģ���˲���ϵͳ����
               [Bz,Az]=bilinear(B,A,1);  %��˫���Ա任����ģ���˲���ת���������˲���
               
        end
end
switch get(handles.popupmenu1,'Value')
    case 1
        [H, w] = freqz(wn);  %freqz�����󴰺�����Ƶ�����ԣ�H������ֵϵ����wΪ��Ӧ��Ƶ��
    case 2
        [H,w] = freqz(Bz,Az);
end
axes(handles.axes5);
plot(w/pi,20*log10(abs(H)),'m');
axis tight;
legend('��������');
axes(handles.axes6);
plot(w/pi,angle(H),'m');
axis tight;
legend('��λ����')
% --- Executes on selection change in popupmenu2.
function popupmenu2_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu2 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu2


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



function edit7_Callback(hObject, eventdata, handles)
% hObject    handle to edit7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit7 as text
%        str2double(get(hObject,'String')) returns contents of edit7 as a double


% --- Executes during object creation, after setting all properties.
function edit7_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit8_Callback(hObject, eventdata, handles)
% hObject    handle to edit8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit8 as text
%        str2double(get(hObject,'String')) returns contents of edit8 as a double


% --- Executes during object creation, after setting all properties.
function edit8_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton10.
function pushbutton10_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global myspeech1 myspeech2 key2
key1 = eval(get(handles.edit7,'String'));
key2 = eval(get(handles.edit8,'String'));
%=== ���� ====%
key2 = abs(key2);
key1 = abs(key1);
myspeech2 = (myspeech1 + key1)/key1;  %����
axes(handles.axes7);  %��λ������ϵ7
plot((0:length(myspeech2)-1)/44100,myspeech2);  %���������źŲ���
xlabel('t(s)');
ylabel('��ֵ');
legend('������������');
axes(handles.axes8);
Y = abs(fft(myspeech2,length(myspeech1))/(length(myspeech2))/2);  %Ƶ�׷���
f = (0:length(myspeech2)/2-1)*44100/length(myspeech2);  %Ƶ��
plot(f,Y(1:length(myspeech2)/2));  %���������ź�Ƶ��ͼ
xlabel('f(Hz)');
ylabel('��ֵ');
legend('��������Ƶ��');




% --- Executes on button press in pushbutton11.
function pushbutton11_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global myspeech2 key2 myspeech_new
myspeech3 = myspeech2*key2 - key2;  %����
if eval(get(handles.edit7,'String')) == eval(get(handles.edit8,'String'))
    axes(handles.axes7);  %��λ������ϵ7
    plot((0:length(myspeech3)-1)/44100,myspeech3);  %���������źŲ���
    xlabel('t(s)');
    ylabel('��ֵ');
    legend('������������');
    axes(handles.axes8);
    Y = abs(fft(myspeech3,length(myspeech3))/(length(myspeech3))/2);  %Ƶ�׷���
    f = (0:length(myspeech3)/2-1)*44100/length(myspeech3);  %Ƶ��
    plot(f,Y(1:length(myspeech3)/2));  %���������ź�Ƶ��ͼ
    xlabel('f(Hz)');
    ylabel('��ֵ');
    legend('��������Ƶ��');
    myspeech_new = myspeech3;  %���ܺ����������
else
    errordlg('��Կ����','����');
end

% --- Executes on button press in pushbutton12.
function pushbutton12_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global myspeech2
sound(myspeech2,44100);

% --- Executes on button press in pushbutton13.
function pushbutton13_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton13 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global myspeech_new
if str2double(get(handles.edit7,'String')) == str2double(get(handles.edit8,'String'))
    sound(myspeech_new,44100);
else
end


% --- Executes on button press in pushbutton14.
function pushbutton14_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton14 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global myspeech1 Fs
[FileName,PathName] = uigetfile('*.wav','Select the MATLAB code file');
[myspeech1,Fs] = audioread([PathName FileName]);
myspeech1=myspeech1(:,1);


% --- Executes on button press in pushbutton15.
function pushbutton15_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton15 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global myspeech_noise Fs
Fs = 44100;
t = eval(get(handles.edit1,'String'));  %¼��ʱ��
recObj1 = audiorecorder(Fs,16,1);  %����һ��¼����������˷�
recordblocking(recObj1,t);  %��ʼ¼��
play(recObj1);
myspeech_noise = getaudiodata(recObj1);


% --- Executes on button press in pushbutton16.
function pushbutton16_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton16 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global myspeech_noise
axes(handles.axes9);  %��λ������ϵ9
plot((0:length(myspeech_noise)-1)/44100,myspeech_noise);  %���������źŲ���
xlabel('t(s)');
ylabel('��ֵ');
legend('������������');
axes(handles.axes10);
Y = abs(fft(myspeech_noise,length(myspeech_noise))/(length(myspeech_noise))/2);  %Ƶ�׷���
f = (0:length(myspeech_noise)/2-1)*44100/length(myspeech_noise);  %Ƶ��
plot(f,Y(1:length(myspeech_noise)/2));  %���������ź�Ƶ��ͼ
xlabel('f(Hz)');
ylabel('��ֵ');
legend('��������Ƶ��');


% --- Executes on button press in pushbutton17.
function pushbutton17_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton17 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global myspeech_noise Fs
[FileName,PathName] = uigetfile('*.wav','Select the MATLAB code file');
[myspeech_noise,Fs] = audioread([PathName FileName]);
myspeech_noise=myspeech_noise(:,1);


% --- Executes on button press in pushbutton18.
function pushbutton18_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton18 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global myspeech_noise
sound(myspeech_noise,44100);


% --- Executes on button press in pushbutton19.
function pushbutton19_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton19 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global myspeech
axes(handles.axes11);  %��λ������ϵ11
plot((0:length(myspeech)-1)/44100,myspeech);  %���������źŲ���
xlabel('t(s)');
ylabel('��ֵ');
legend('�������������');
axes(handles.axes12);
Y = abs(fft(myspeech,length(myspeech))/(length(myspeech))/2);  %Ƶ�׷���
f = (0:length(myspeech)/2-1)*44100/length(myspeech);  %Ƶ��
plot(f,Y(1:length(myspeech)/2));  %���������ź�Ƶ��ͼ
xlabel('f(Hz)');
ylabel('��ֵ');
legend('���������Ƶ��');



% --- Executes on button press in pushbutton21.
function pushbutton21_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton21 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global myspeech1
global myspeech_noise Fs
Fs = 44100;
L=length(myspeech1);
myspeech_noise=0.1*randn(L,1);


% --- Executes on button press in pushbutton23.
function pushbutton23_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton23 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global myspeech
sound(myspeech,44100);


% --- Executes on button press in pushbutton24.
function pushbutton24_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton24 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global myspeech Fs
Fs = 44100;
global myspeech_noise
global myspeech1 
n1=length(myspeech_noise);
n2=length(myspeech1);
if(n1>n2)
    p = n1-n2;
    myspeech =myspeech_noise+padarray(myspeech1, p/2);
else
    p = n2-n1;
    myspeech =myspeech1+padarray(myspeech_noise, p/2);
end



% --- Executes during object creation, after setting all properties.
function pushbutton24_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pushbutton24 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% --- Executes during object creation, after setting all properties.
function axes9_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate axes9


% --- Executes on button press in pushbutton27.
function pushbutton27_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton27 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global e
global myspeech1
global myspeech Fs
Fs = 44100;
Y=myspeech1;
L=length(Y);
n=length(Y);%ȡ��������
t=(0:n-1)/Fs;%��ʾʵ��ʱ��
y_z=myspeech;
IS=0.25; % ����ǰ���޻��γ���
wlen=200; % ����֡��Ϊ25ms
inc=80; % ����֡��Ϊ10ms
NIS=fix((IS*Fs-wlen)/inc +1); % ��ǰ���޻���֡��
a=4; b=0.001; % ���ò���a��b
e=SpectralSub(y_z,wlen,inc,NIS,a,b);% �׼�
%��һ����ͨ�˲�
fp=1300;fc=5500;As=100;Ap=1;
wc=2*pi*fc/Fs;
wp=2*pi*fp/Fs;%ͨ���ı�ԵƵ��
wde1=wc*wp;
beta=0.112*(As*8.7);
N=ceil((As*8)/2.285/wde1);%�˲����Ľ���
wn=kaiser(N+1,beta);%�˲����Ľ�ֹƵ��,�ÿ�����
ws=(wp+wc)/2/pi;
b=fir1(N,ws,wn);%FIR�˲�������
e=fftfilt(b,e);
%






function edit14_Callback(hObject, eventdata, handles)
% hObject    handle to edit14 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit14 as text
%        str2double(get(hObject,'String')) returns contents of edit14 as a double


% --- Executes during object creation, after setting all properties.
function edit14_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit14 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit16_Callback(hObject, eventdata, handles)
% hObject    handle to edit16 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit16 as text
%        str2double(get(hObject,'String')) returns contents of edit16 as a double


% --- Executes during object creation, after setting all properties.
function edit16_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit16 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit15_Callback(hObject, eventdata, handles)
% hObject    handle to edit15 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit15 as text
%        str2double(get(hObject,'String')) returns contents of edit15 as a double


% --- Executes during object creation, after setting all properties.
function edit15_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit15 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit13_Callback(hObject, eventdata, handles)
% hObject    handle to edit13 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit13 as text
%        str2double(get(hObject,'String')) returns contents of edit13 as a double


% --- Executes during object creation, after setting all properties.
function edit13_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit13 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton28.
function pushbutton28_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton28 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes during object creation, after setting all properties.
function pushbutton27_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pushbutton27 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% --- Executes on button press in pushbutton29.
function pushbutton29_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton29 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global e
sound(e,44100);


% --- Executes on button press in pushbutton31.
function pushbutton31_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton31 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global e
axes(handles.axes15);  %��λ������ϵ1
plot((0:length(e)-1)/44100,e);  %���������źŲ���
xlabel('t(s)');
ylabel('��ֵ');
legend('�˲�����������');
axes(handles.axes16);
Y = abs(fft(e,length(e))/(length(e))/2);  %Ƶ�׷���
f = (0:length(e)/2-1)*44100/length(e);  %Ƶ��
plot(f,Y(1:length(e)/2));  %���������ź�Ƶ��ͼ
xlabel('f(Hz)');
ylabel('��ֵ');
legend('�˲�������Ƶ��');


% --- Executes during object creation, after setting all properties.
function pushbutton9_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pushbutton9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% --- Executes on button press in pushbutton32.
function pushbutton32_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton32 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global myspeech1
global myspeech Fs
global ee
Y=myspeech1;
%sound(y,Fs);%�ط���Ƶ
L=length(Y);
n=length(Y);%ȡ��������
t=(0:n-1)/Fs;%��ʾʵ��ʱ��
y_z=myspeech;
IS=0.9; % ����ǰ���޻��γ���
wlen=30; % ����֡��Ϊ25ms
inc=10; % ����֡��Ϊ10ms 
NIS=fix((IS*Fs-wlen)/inc +1); % ��ǰ���޻���֡��
alpha=0.5; 
ee=Weina_Im(y_z,wlen,inc,NIS,alpha) ;%ά���˲�
% ee=WienerScalart96(xs1,FS,IS);%ά���˲�
%��һ����ͨ�˲�
fp=1300;fc=5500;As=100;Ap=1;
wc=2*pi*fc/Fs;
wp=2*pi*fp/Fs;%ͨ���ı�ԵƵ��
wde1=wc*wp;
beta=0.112*(As*8.7);
N=ceil((As*8)/2.285/wde1);%�˲����Ľ���
wn=kaiser(N+1,beta);%�˲����Ľ�ֹƵ��,�ÿ�����
ws=(wp+wc)/2/pi;
b=fir1(N,ws,wn);%FIR�˲�������
ee=fftfilt(b,ee);
%

% --- Executes on button press in pushbutton33.
function pushbutton33_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton33 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global ee
sound(ee,44100);



% --- Executes on button press in pushbutton34.
function pushbutton34_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton34 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global ee
axes(handles.axes18);  %��λ������ϵ1
plot((0:length(ee)-1)/44100,ee);  %���������źŲ���
xlabel('t(s)');
ylabel('��ֵ');
legend('�˲�����������');
axes(handles.axes17);
Y = abs(fft(ee,length(ee))/(length(ee))/2);  %Ƶ�׷���
f = (0:length(ee)/2-1)*44100/length(ee);  %Ƶ��
plot(f,Y(1:length(ee)/2));  %���������ź�Ƶ��ͼ
xlabel('f(Hz)');
ylabel('��ֵ');
legend('�˲�������Ƶ��');
