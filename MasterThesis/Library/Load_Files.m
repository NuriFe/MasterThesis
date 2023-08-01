function varargout = Load_Files(varargin)
% LOAD_FILES MATLAB code for Load_Files.fig
%      LOAD_FILES, by itself, creates a new LOAD_FILES or raises the existing
%      singleton*.
%
%      H = LOAD_FILES returns the handle to a new LOAD_FILES or the handle to
%      the existing singleton*.
%
%      LOAD_FILES('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in LOAD_FILES.M with the given input arguments.
%
%      LOAD_FILES('Property','Value',...) creates a new LOAD_FILES or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before Load_Files_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to Load_Files_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help Load_Files

% Last Modified by GUIDE v2.5 15-Jan-2020 16:59:36

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @Load_Files_OpeningFcn, ...
                   'gui_OutputFcn',  @Load_Files_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
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


% --- Executes just before Load_Files is made visible.
function Load_Files_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to Load_Files (see VARARGIN)

% Choose default command line output for Load_Files
handles.output = hObject;
addpath('Functions')
set(handles.tagFile1, 'String', '')
set(handles.tagFile2, 'String', '')
set(handles.tagFile3, 'String', '')
set(handles.tagFile4, 'String', '')
set(handles.titleFile1, 'String','')
set(handles.titleFile2, 'String','')
set(handles.titleFile3, 'String','')
set(handles.titleFile4, 'String','')


handles.fileName1 = '';
handles.fileName2 = '';
handles.fileName3 = '';
handles.fileName4 = '';
handles.path1 = '';
handles.path2 = '';
handles.path3 = '';
handles.path4 = '';


% Update handles structure
guidata(hObject, handles);

% UIWAIT makes Load_Files wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = Load_Files_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% --- Executes on button press in file1.
function file1_Callback(hObject, eventdata, handles)

[fileName1, path1] = uigetfile({'*.sto;*.mot;*.trc;*.txt',...
    'OpenSIM files (*.sto,*.mot,*.trc,*.txt)'});


handles.path1 = path1;
handles.fileName1 = fileName1;
if ~isequal(fileName1,0)
    set(handles.titleFile1, 'String',fileName1);
end
guidata( hObject, handles);



function tagFile1_Callback(hObject, eventdata, handles)



% --- Executes during object creation, after setting all properties.
function tagFile1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to tagFile1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% --- Executes on button press in file1.
function file2_Callback(hObject, eventdata, handles)

[fileName2, path2] = uigetfile({'*.sto;*.mot;*.trc;*.txt',...
    'OpenSIM files (*.sto,*.mot,*.trc,*.txt)'});


handles.path2 = path2;
handles.fileName2 = fileName2;
if ~isequal(fileName2,0)
    set(handles.titleFile2, 'String',fileName2);
end
guidata( hObject, handles);


function tagFile2_Callback(hObject, eventdata, handles)



% --- Executes during object creation, after setting all properties.
function tagFile2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to tagFile2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% --- Executes on button press in file3.
function file3_Callback(hObject, eventdata, handles)

[fileName3, path3] = uigetfile({'*.sto;*.mot;*.trc;*.txt',...
    'OpenSIM files (*.sto,*.mot,*.trc,*.txt)'});


handles.path3 = path3;
handles.fileName3 = fileName3;
if ~isequal(fileName3,0)
    set(handles.titleFile3, 'String',fileName3);
end
guidata( hObject, handles);



function tagFile3_Callback(hObject, eventdata, handles)
% hObject    handle to tagFile3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of tagFile3 as text
%        str2double(get(hObject,'String')) returns contents of tagFile3 as a double


% --- Executes during object creation, after setting all properties.
function tagFile3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to tagFile3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% --- Executes on button press in file4.
function file4_Callback(hObject, eventdata, handles)

[fileName4, path4] = uigetfile({'*.sto;*.mot;*.trc;*.txt',...
    'OpenSIM files (*.sto,*.mot,*.trc,*.txt)'});


handles.path4 = path4;
handles.fileName4 = fileName4;
if ~isequal(fileName4,0)
    set(handles.titleFile4, 'String',fileName4);
end
guidata( hObject, handles);

function tagFile4_Callback(hObject, eventdata, handles)
% hObject    handle to tagFile4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of tagFile4 as text
%        str2double(get(hObject,'String')) returns contents of tagFile4 as a double


% --- Executes during object creation, after setting all properties.
function tagFile4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to tagFile4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end









% --- Executes on button press in loadButton.
function loadButton_Callback(hObject, eventdata, handles)

tag1 = get(handles.tagFile1, 'String');
tag2 = get(handles.tagFile2, 'String');
tag3 = get(handles.tagFile3, 'String');
tag4 = get(handles.tagFile4, 'String');

tags = {tag1;tag2;tag3;tag4};
paths = {handles.path1; handles.path2; handles.path3; handles.path4};
fileNames = {handles.fileName1;handles.fileName2;handles.fileName3;handles.fileName4};

T = table(paths,fileNames,tags);
Plot_Data(T);
% assignin('base','T',T);


% disp(['file4' handles.fileName4])
% disp(['file3' handles.fileName3])
% disp(['file2' handles.fileName2])
% disp(['file1' handles.fileName1])
% disp([ 'path4' handles.path4])
% disp([ 'path3' handles.path3])
% disp([ 'path2' handles.path2])
% disp([ 'path1' handles.path1])

% FN1 = get(handles.fileName1,'String');
% disp(FN1)
