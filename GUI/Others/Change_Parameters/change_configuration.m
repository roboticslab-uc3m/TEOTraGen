function varargout = change_configuration(varargin)
% change_configuration MATLAB code for change_configuration.fig
%      change_configuration, by itself, creates a new change_configuration or raises the existing
%      singleton*.
%
%      H = change_configuration returns the handle to a new change_configuration or the handle to
%      the existing singleton*.
%
%      change_configuration('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in change_configuration.M with the given input arguments.
%
%      change_configuration('Property','Value',...) creates a new change_configuration or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before change_configuration_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to change_configuration_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help change_configuration

% Last Modified by GUIDE v2.5 22-Apr-2014 17:05:22

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @change_configuration_OpeningFcn, ...
                   'gui_OutputFcn',  @change_configuration_OutputFcn, ...
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


function change_configuration_OpeningFcn(hObject, eventdata, handles, varargin)
% Choose default command line output for change_configuration
handles.output = hObject;

handles.q = zeros(26,1);
guidata(hObject, handles);

if isempty(varargin)
  warning('WarnTEOTrajGen:noInputGUI', 'No inputs for change_configuration');
  for ii = 1:26,
    set(handles.(strcat('edit_',num2str(ii))),'String', num2str(0));
  end
else
  if length(varargin) > 1
    handles.humanoid = varargin{2};
  else
    handles.humanoid = 0;
  end
  % Get previous panels
  for ii = 1:26,
    handles.q(ii) = varargin{1}(ii);
    set(handles.(strcat('edit_',num2str(ii))),'String', varargin{1}(ii));
  end
end

% Update handles structure
guidata(hObject, handles);

% UIWAIT
uiwait(handles.figure1);


function varargout = change_configuration_OutputFcn(hObject, eventdata, handles)
varargout{1} = handles.q;
close(gcf);



function pushbutton_ok_Callback(hObject, eventdata, handles)
for ii = 1:26,
  handles.q(ii) = str2double(get(handles.(strcat('edit_',num2str(ii))),'String'));
end
guidata(hObject, handles);

if isequal(get(gcf, 'waitstatus'), 'waiting')
  % The GUI is still in UIWAIT, us UIRESUME
  uiresume(gcf);
else
  % The GUI is no longer waiting, just close it
  close(gcf);
end



function edit_1_Callback(hObject, eventdata, handles)


function edit_1_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function pushbutton_cancel_Callback(hObject, eventdata, handles)
if isequal(get(gcf, 'waitstatus'), 'waiting')
    % The GUI is still in UIWAIT, us UIRESUME
    uiresume(gcf);
else
    % The GUI is no longer waiting, just close it
    close(gcf);
end



function edit_4_Callback(hObject, eventdata, handles)
% hObject    handle to edit_4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_4 as text
%        str2double(get(hObject,'String')) returns contents of edit_4 as a double


% --- Executes during object creation, after setting all properties.
function edit_4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_2_Callback(hObject, eventdata, handles)
% hObject    handle to edit_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_2 as text
%        str2double(get(hObject,'String')) returns contents of edit_2 as a double


% --- Executes during object creation, after setting all properties.
function edit_2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_3_Callback(hObject, eventdata, handles)
% hObject    handle to edit_3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_3 as text
%        str2double(get(hObject,'String')) returns contents of edit_3 as a double


% --- Executes during object creation, after setting all properties.
function edit_3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_7_Callback(hObject, eventdata, handles)
% hObject    handle to edit_7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_7 as text
%        str2double(get(hObject,'String')) returns contents of edit_7 as a double


% --- Executes during object creation, after setting all properties.
function edit_7_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_8_Callback(hObject, eventdata, handles)
% hObject    handle to edit_8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_8 as text
%        str2double(get(hObject,'String')) returns contents of edit_8 as a double


% --- Executes during object creation, after setting all properties.
function edit_8_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_9_Callback(hObject, eventdata, handles)
% hObject    handle to edit_9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_9 as text
%        str2double(get(hObject,'String')) returns contents of edit_9 as a double


% --- Executes during object creation, after setting all properties.
function edit_9_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_13_Callback(hObject, eventdata, handles)
% hObject    handle to edit_13 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_13 as text
%        str2double(get(hObject,'String')) returns contents of edit_13 as a double


% --- Executes during object creation, after setting all properties.
function edit_13_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_13 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_15_Callback(hObject, eventdata, handles)
% hObject    handle to edit_15 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_15 as text
%        str2double(get(hObject,'String')) returns contents of edit_15 as a double


% --- Executes during object creation, after setting all properties.
function edit_15_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_15 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_16_Callback(hObject, eventdata, handles)
% hObject    handle to edit_16 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_16 as text
%        str2double(get(hObject,'String')) returns contents of edit_16 as a double


% --- Executes during object creation, after setting all properties.
function edit_16_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_16 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_17_Callback(hObject, eventdata, handles)
% hObject    handle to edit_17 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_17 as text
%        str2double(get(hObject,'String')) returns contents of edit_17 as a double


% --- Executes during object creation, after setting all properties.
function edit_17_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_17 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_21_Callback(hObject, eventdata, handles)
% hObject    handle to edit_21 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_21 as text
%        str2double(get(hObject,'String')) returns contents of edit_21 as a double


% --- Executes during object creation, after setting all properties.
function edit_21_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_21 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_5_Callback(hObject, eventdata, handles)
% hObject    handle to edit_5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_5 as text
%        str2double(get(hObject,'String')) returns contents of edit_5 as a double


% --- Executes during object creation, after setting all properties.
function edit_5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_6_Callback(hObject, eventdata, handles)
% hObject    handle to edit_6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_6 as text
%        str2double(get(hObject,'String')) returns contents of edit_6 as a double


% --- Executes during object creation, after setting all properties.
function edit_6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_10_Callback(hObject, eventdata, handles)
% hObject    handle to edit_10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_10 as text
%        str2double(get(hObject,'String')) returns contents of edit_10 as a double


% --- Executes during object creation, after setting all properties.
function edit_10_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_11_Callback(hObject, eventdata, handles)
% hObject    handle to edit_11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_11 as text
%        str2double(get(hObject,'String')) returns contents of edit_11 as a double


% --- Executes during object creation, after setting all properties.
function edit_11_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_12_Callback(hObject, eventdata, handles)
% hObject    handle to edit_12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_12 as text
%        str2double(get(hObject,'String')) returns contents of edit_12 as a double


% --- Executes during object creation, after setting all properties.
function edit_12_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_14_Callback(hObject, eventdata, handles)
% hObject    handle to edit_14 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_14 as text
%        str2double(get(hObject,'String')) returns contents of edit_14 as a double


% --- Executes during object creation, after setting all properties.
function edit_14_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_14 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_18_Callback(hObject, eventdata, handles)
% hObject    handle to edit_18 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_18 as text
%        str2double(get(hObject,'String')) returns contents of edit_18 as a double


% --- Executes during object creation, after setting all properties.
function edit_18_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_18 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_19_Callback(hObject, eventdata, handles)
% hObject    handle to edit_19 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_19 as text
%        str2double(get(hObject,'String')) returns contents of edit_19 as a double


% --- Executes during object creation, after setting all properties.
function edit_19_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_19 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_20_Callback(hObject, eventdata, handles)
% hObject    handle to edit_20 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_20 as text
%        str2double(get(hObject,'String')) returns contents of edit_20 as a double


% --- Executes during object creation, after setting all properties.
function edit_20_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_20 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_24_Callback(hObject, eventdata, handles)
% hObject    handle to edit_24 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_24 as text
%        str2double(get(hObject,'String')) returns contents of edit_24 as a double


% --- Executes during object creation, after setting all properties.
function edit_24_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_24 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_22_Callback(hObject, eventdata, handles)
% hObject    handle to edit_22 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_22 as text
%        str2double(get(hObject,'String')) returns contents of edit_22 as a double


% --- Executes during object creation, after setting all properties.
function edit_22_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_22 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_23_Callback(hObject, eventdata, handles)
% hObject    handle to edit_23 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_23 as text
%        str2double(get(hObject,'String')) returns contents of edit_23 as a double


% --- Executes during object creation, after setting all properties.
function edit_23_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_23 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_25_Callback(hObject, eventdata, handles)
% hObject    handle to edit_25 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_25 as text
%        str2double(get(hObject,'String')) returns contents of edit_25 as a double


% --- Executes during object creation, after setting all properties.
function edit_25_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_25 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit29_Callback(hObject, eventdata, handles)
% hObject    handle to edit29 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit29 as text
%        str2double(get(hObject,'String')) returns contents of edit29 as a double


% --- Executes during object creation, after setting all properties.
function edit29_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit29 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_26_Callback(hObject, eventdata, handles)
% hObject    handle to edit_26 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_26 as text
%        str2double(get(hObject,'String')) returns contents of edit_26 as a double


% --- Executes during object creation, after setting all properties.
function edit_26_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_26 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit31_Callback(hObject, eventdata, handles)
% hObject    handle to edit_7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_7 as text
%        str2double(get(hObject,'String')) returns contents of edit_7 as a double


% --- Executes during object creation, after setting all properties.
function edit31_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit32_Callback(hObject, eventdata, handles)
% hObject    handle to edit_8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_8 as text
%        str2double(get(hObject,'String')) returns contents of edit_8 as a double


% --- Executes during object creation, after setting all properties.
function edit32_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit33_Callback(hObject, eventdata, handles)
% hObject    handle to edit_9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_9 as text
%        str2double(get(hObject,'String')) returns contents of edit_9 as a double


% --- Executes during object creation, after setting all properties.
function edit33_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit34_Callback(hObject, eventdata, handles)
% hObject    handle to edit_10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_10 as text
%        str2double(get(hObject,'String')) returns contents of edit_10 as a double


% --- Executes during object creation, after setting all properties.
function edit34_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit35_Callback(hObject, eventdata, handles)
% hObject    handle to edit_11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_11 as text
%        str2double(get(hObject,'String')) returns contents of edit_11 as a double


% --- Executes during object creation, after setting all properties.
function edit35_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit36_Callback(hObject, eventdata, handles)
% hObject    handle to edit_12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_12 as text
%        str2double(get(hObject,'String')) returns contents of edit_12 as a double


% --- Executes during object creation, after setting all properties.
function edit36_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit37_Callback(hObject, eventdata, handles)
% hObject    handle to edit_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_1 as text
%        str2double(get(hObject,'String')) returns contents of edit_1 as a double


% --- Executes during object creation, after setting all properties.
function edit37_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit38_Callback(hObject, eventdata, handles)
% hObject    handle to edit_4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_4 as text
%        str2double(get(hObject,'String')) returns contents of edit_4 as a double


% --- Executes during object creation, after setting all properties.
function edit38_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit39_Callback(hObject, eventdata, handles)
% hObject    handle to edit_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_2 as text
%        str2double(get(hObject,'String')) returns contents of edit_2 as a double


% --- Executes during object creation, after setting all properties.
function edit39_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit40_Callback(hObject, eventdata, handles)
% hObject    handle to edit_3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_3 as text
%        str2double(get(hObject,'String')) returns contents of edit_3 as a double


% --- Executes during object creation, after setting all properties.
function edit40_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit41_Callback(hObject, eventdata, handles)
% hObject    handle to edit_5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_5 as text
%        str2double(get(hObject,'String')) returns contents of edit_5 as a double


% --- Executes during object creation, after setting all properties.
function edit41_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit42_Callback(hObject, eventdata, handles)
% hObject    handle to edit_6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_6 as text
%        str2double(get(hObject,'String')) returns contents of edit_6 as a double


% --- Executes during object creation, after setting all properties.
function edit42_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function pushbutton_from_file_Callback(hObject, eventdata, handles)
  [FileCSV PathCSV] = uigetfile({'*.csv'}, 'Choose csv file with the initial configuration');

  if isequal(FileCSV,0)||isequal(PathCSV,0)
    CSVerror = 1;
    errorText = 'No initial csv file loaded! Using default configuration';
  else
    try 
      if isempty(size(load(FileCSV)))
        CSVerror = 1;
        errorText = 'Empty csv file loaded! Using default configuration';

      else
        configuration = csvread(FileCSV, size(load(FileCSV), 1) - 1);

        if length(configuration) ~= 26
          CSVerror = 1;
          errorText = 'Wrong size of the csv file loaded! Using default configuration';
        elseif length(configuration) == 26
          CSVerror = 0;
          handles.q = reshape(configuration,26,1);
          guidata(hObject, handles);
          for ii=1:26,
            set(handles.(strcat('edit_',num2str(ii))),'String', handles.q(ii));
          end
        end
      end

    catch err
      CSVerror = 1;
      errorText = 'Error loading csv file, please check your data! Using default configuration';
      error('ErrorTEOTraGen:wrongInputFile','Error loading csv file, please check your data! Using default configuration');
    end    

  end

  if CSVerror
    h = msgbox(errorText, 'Error charging data','error');
  end


function pushbutton_insert_joints_rl_Callback(hObject, eventdata, handles)
  if isstruct(handles.humanoid)
    humanoid_part_position = 1:6;
    q = joint_insert('RF', handles.q, handles.humanoid);
    for ii = humanoid_part_position,
      set(handles.(strcat('edit_',num2str(ii))),'String', num2str(q(ii)));
    end
  else
    display('It is necessary a humanoid structure');
  end
% end

guidata(hObject, handles);

function pushbutton_insert_joints_ll_Callback(hObject, eventdata, handles)
  if isstruct(handles.humanoid)
    humanoid_part_position = 7:12;
    q = joint_insert('LF', handles.q, handles.humanoid);
    for ii = humanoid_part_position,
      set(handles.(strcat('edit_',num2str(ii))),'String', num2str(q(ii)));
    end
  else
    display('It is necessary a humanoid structure');
  end
% end

guidata(hObject, handles);

function pushbutton_insert_joints_ra_Callback(hObject, eventdata, handles)
  if isstruct(handles.humanoid)
    humanoid_part_position = 15:20;
    q = joint_insert('RH', handles.q, handles.humanoid);
    for ii = humanoid_part_position,
      set(handles.(strcat('edit_',num2str(ii))),'String', num2str(q(ii)));
    end
  else
    display('It is necessary a humanoid structure');
  end
% end

guidata(hObject, handles);

function pushbutton_insert_joints_la_Callback(hObject, eventdata, handles)
  if isstruct(handles.humanoid)
    humanoid_part_position = 21:26;
    q = joint_insert('LH', handles.q, handles.humanoid);
    for ii = humanoid_part_position,
      set(handles.(strcat('edit_',num2str(ii))),'String', num2str(q(ii)));
    end
  else
    display('It is necessary a humanoid structure');
  end
% end

guidata(hObject, handles);
