function varargout = Plot_Data(varargin)
% PLOT_DATA MATLAB code for Plot_Data.fig
%      PLOT_DATA, by itself, creates a new PLOT_DATA or raises the existing
%      singleton*.
%
%      H = PLOT_DATA returns the handle to a new PLOT_DATA or the handle to
%      the existing singleton*.
%
%      PLOT_DATA('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in PLOT_DATA.M with the given input arguments.
%
%      PLOT_DATA('Property','Value',...) creates a new PLOT_DATA or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before Plot_Data_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to Plot_Data_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help Plot_Data

% Last Modified by GUIDE v2.5 20-Jan-2020 15:39:40

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @Plot_Data_OpeningFcn, ...
                   'gui_OutputFcn',  @Plot_Data_OutputFcn, ...
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


% --- Executes just before Plot_Data is made visible.
function Plot_Data_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to Plot_Data (see VARARGIN)

% Choose default command line output for Plot_Data
handles.output = hObject;
handles.T = varargin{1};
set(handles.titlePlot,'String','')

for i = 1 : height(handles.T)
    
    
    if ~isempty(handles.T.paths{i})
    set(handles.(['data', num2str(i)]),'Visible','on')  
    set(handles.(['fn', num2str(i)]),'Visible','on') 
    set(handles.(['tag', num2str(i)]),'Visible','on') 
    set(handles.(['sd', num2str(i)]),'Visible','on') 
    set(handles.(['x', num2str(i)]),'Visible','on') 
    set(handles.(['ea', num2str(i)]),'Visible','on') %export all button
    set(handles.(['es', num2str(i)]),'Visible','on') % export selected button

    
    [handles.T.data{i}, handles.T.variableNames{i}, handles.T.tableData{i}] = readMOTSTOTRCfiles(handles.T.paths{i} , handles.T.fileNames{i});
    set(handles.(['data', num2str(i)]),'String',handles.T.variableNames{i})
    set(handles.(['fn', num2str(i)]),'String',handles.T.fileNames{i})
    set(handles.(['tag', num2str(i)]),'String',handles.T.tags{i})
    
    else
      set(handles.(['data', num2str(i)]),'Visible','off')  
      set(handles.(['fn', num2str(i)]),'Visible','off') 
      set(handles.(['tag', num2str(i)]),'Visible','off') 
      set(handles.(['sd', num2str(i)]),'Visible','off') 
      set(handles.(['x', num2str(i)]),'Visible','off') 
      set(handles.(['ea', num2str(i)]),'Visible','off') 
      set(handles.(['es', num2str(i)]),'Visible','off') 
    end
      
    
end






% Update handles structure
guidata(hObject, handles);

% UIWAIT makes Plot_Data wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = Plot_Data_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on selection change in data1.
function data1_Callback(hObject, eventdata, handles)
% hObject    handle to data1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns data1 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from data1


% --- Executes during object creation, after setting all properties.
function data1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to data1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in data2.
function data2_Callback(hObject, eventdata, handles)
% hObject    handle to data2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns data2 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from data2


% --- Executes during object creation, after setting all properties.
function data2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to data2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in data3.
function data3_Callback(hObject, eventdata, handles)
% hObject    handle to data3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns data3 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from data3


% --- Executes during object creation, after setting all properties.
function data3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to data3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in data4.
function data4_Callback(hObject, eventdata, handles)
% hObject    handle to data4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns data4 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from data4


% --- Executes during object creation, after setting all properties.
function data4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to data4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end







% --- Executes when x1 is resized.
function x1_SizeChangedFcn(hObject, eventdata, handles)
% hObject    handle to x1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)




% --- Executes on button press in plotButton.
function plotButton_Callback(hObject, eventdata, handles)

% valeur = get(handles.time1,'Value');
% disp(valeur)

titleFigure = get(handles.titlePlot,'String');
xlabeltext = get(handles.XLabelInput,'String');
ylabeltext = get(handles.YLabelInput,'String');
allLegends = {};
figure; hold on
set(gcf, 'Units', 'Normalized', 'OuterPosition', [0, 0.04, 1, 0.96]);
for i = 1 : height(handles.T)
    
    
    if ~isempty(handles.T.paths{i})   
    selection = get(handles.(['data' , num2str(i)]),'Value');
    xaxisSelection  = get(handles.(['time' num2str(i)]), 'Value');
    if ~isempty(selection)
        
        tagForLegend = [get(handles.(['tag' num2str(i)]), 'String'), '-'];
        dataLegend = handles.T.variableNames{i}(selection);
        dataLegend = strcat(tagForLegend, dataLegend);
        dataLegend = strrep(dataLegend, '_','-');
        
        allLegends = [allLegends,dataLegend ];
        
        if xaxisSelection == 1
        plot(handles.T.tableData{i}.time , handles.T.data{i}(:,selection),'LineWidth',2.5) % plot Time
        else
        plot(handles.T.data{i}(:,selection),'LineWidth',2.5) % plot Frames
        end
        
        
        
    end
    end
    title(titleFigure);
    xlabel(xlabeltext);
    ylabel(ylabeltext);
    legend(allLegends,'location','eastoutside')
end




function titlePlot_Callback(hObject, eventdata, handles)



% --- Executes during object creation, after setting all properties.
function titlePlot_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function XLabelInput_Callback(hObject, eventdata, handles)
% hObject    handle to XLabelInput (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of XLabelInput as text
%        str2double(get(hObject,'String')) returns contents of XLabelInput as a double


% --- Executes during object creation, after setting all properties.
function XLabelInput_CreateFcn(hObject, eventdata, handles)
% hObject    handle to XLabelInput (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function YLabelInput_Callback(hObject, eventdata, handles)
% hObject    handle to YLabelInput (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of YLabelInput as text
%        str2double(get(hObject,'String')) returns contents of YLabelInput as a double


% --- Executes during object creation, after setting all properties.
function YLabelInput_CreateFcn(hObject, eventdata, handles)
% hObject    handle to YLabelInput (see GCBO)
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


% --- Executes on button press in ea1.
function ea1_Callback(hObject, eventdata, handles)
fileNameSaving = inputdlg('Enter the desired filename','Output filename', [1,50]);

if isempty(fileNameSaving)  
    return
end

while isempty(fileNameSaving{1}) 
    fileNameSaving = inputdlg('Please, enter a valid filename','Output filename', [1,50]);
if isempty(fileNameSaving)  
    return
end
end
data = handles.T.tableData{1};
save([fileNameSaving{1} '.mat'],'data')



% --- Executes on button press in ea2.
function ea2_Callback(hObject, eventdata, handles)
fileNameSaving = inputdlg('Enter the desired filename','Output filename', [1,50]);

if isempty(fileNameSaving)  
    return
end

while isempty(fileNameSaving{1}) 
    fileNameSaving = inputdlg('Please, enter a valid filename','Output filename', [1,50]);
if isempty(fileNameSaving)  
    return
end
end
data = handles.T.tableData{2};
save([fileNameSaving{1} '.mat'],'data')


% --- Executes on button press in ea3.
function ea3_Callback(hObject, eventdata, handles)
fileNameSaving = inputdlg('Enter the desired filename','Output filename', [1,50]);

if isempty(fileNameSaving)  
    return
end

while isempty(fileNameSaving{1}) 
    fileNameSaving = inputdlg('Please, enter a valid filename','Output filename', [1,50]);
if isempty(fileNameSaving)  
    return
end
end
data = handles.T.tableData{3};
save([fileNameSaving{1} '.mat'],'data')


% --- Executes on button press in ea4.
function ea4_Callback(hObject, eventdata, handles)

fileNameSaving = inputdlg('Enter the desired filename','Output filename', [1,50]);

if isempty(fileNameSaving)  
    return
end

while isempty(fileNameSaving{1}) 
    fileNameSaving = inputdlg('Please, enter a valid filename','Output filename', [1,50]);
if isempty(fileNameSaving)  
    return
end
end
data = handles.T.tableData{4};
save([fileNameSaving{1} '.mat'],'data')



% --- Executes on button press in es1.
function es1_Callback(hObject, eventdata, handles)

selection = get(handles.(['data' , num2str(1)]),'Value');
if ~isempty(selection)
fileNameSaving = inputdlg('Enter the desired filename','Output filename', [1,50]);

if isempty(fileNameSaving)  
    return
end

while isempty(fileNameSaving{1}) 
    fileNameSaving = inputdlg('Please, enter a valid filename','Output filename', [1,50]);
if isempty(fileNameSaving)  
    return
end

end
selectedVariables = handles.T.variableNames{1}(selection);
data = handles.T.tableData{1}(:, contains(handles.T.tableData{1}.Properties.VariableNames, selectedVariables));

save([fileNameSaving{1} '.mat'],'data')
end


% --- Executes on button press in es2.
function es2_Callback(hObject, eventdata, handles)
% hObject    handle to es2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
selection = get(handles.(['data' , num2str(2)]),'Value');
if ~isempty(selection)
fileNameSaving = inputdlg('Enter the desired filename','Output filename', [1,50]);

if isempty(fileNameSaving)  
    return
end

while isempty(fileNameSaving{1}) 
    fileNameSaving = inputdlg('Please, enter a valid filename','Output filename', [1,50]);
if isempty(fileNameSaving)  
    return
end
end

selectedVariables = handles.T.variableNames{2}(selection);
data = handles.T.tableData{2}(:, contains(handles.T.tableData{2}.Properties.VariableNames, selectedVariables));
save([fileNameSaving{1} '.mat'],'data')
end


% --- Executes on button press in es3.
function es3_Callback(hObject, eventdata, handles)
% hObject    handle to es3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
selection = get(handles.(['data' , num2str(3)]),'Value');
if ~isempty(selection)
fileNameSaving = inputdlg('Enter the desired filename','Output filename', [1,50]);

if isempty(fileNameSaving)  
    return
end

while isempty(fileNameSaving{1}) 
    fileNameSaving = inputdlg('Please, enter a valid filename','Output filename', [1,50]);
if isempty(fileNameSaving)  
    return
end

end
selectedVariables = handles.T.variableNames{1}(selection);
data = handles.T.tableData{3}(:, contains(handles.T.tableData{3}.Properties.VariableNames, selectedVariables));
save([fileNameSaving{1} '.mat'],'data')
end


% --- Executes on button press in es4.
function es4_Callback(hObject, eventdata, handles)
selection = get(handles.(['data' , num2str(4)]),'Value');
if ~isempty(selection)
fileNameSaving = inputdlg('Enter the desired filename','Output filename', [1,50]);

if isempty(fileNameSaving)  
    return
end

while isempty(fileNameSaving{1}) 
    fileNameSaving = inputdlg('Please, enter a valid filename','Output filename', [1,50]);
if isempty(fileNameSaving)  
    return
end

end
selectedVariables = handles.T.variableNames{1}(selection);
data = handles.T.tableData{4}(:, contains(handles.T.tableData{4}.Properties.VariableNames, selectedVariables));
save([fileNameSaving{1} '.mat'],'data')
end


% --- Executes on button press in quitButton.
function quitButton_Callback(hObject, eventdata, handles)
close all
