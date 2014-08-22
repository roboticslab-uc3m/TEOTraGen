function varargout = change_cart_table_parameters(varargin)
% CHANGE_CART_TABLE_PARAMETERS MATLAB code for change_cart_table_parameters.fig
%      CHANGE_CART_TABLE_PARAMETERS, by itself, creates a new CHANGE_CART_TABLE_PARAMETERS or raises the existing
%      singleton*.
%
%      H = CHANGE_CART_TABLE_PARAMETERS returns the handle to a new CHANGE_CART_TABLE_PARAMETERS or the handle to
%      the existing singleton*.
%
%      CHANGE_CART_TABLE_PARAMETERS('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in CHANGE_CART_TABLE_PARAMETERS.M with the given input arguments.
%
%      CHANGE_CART_TABLE_PARAMETERS('Property','Value',...) creates a new CHANGE_CART_TABLE_PARAMETERS or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before change_cart_table_parameters_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to change_cart_table_parameters_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help change_cart_table_parameters

% Last Modified by GUIDE v2.5 08-Jun-2014 17:55:22

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @change_cart_table_parameters_OpeningFcn, ...
                   'gui_OutputFcn',  @change_cart_table_parameters_OutputFcn, ...
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


function change_cart_table_parameters_OpeningFcn(hObject, eventdata, handles, varargin)
% Choose default command line output for change_cart_table_parameters
handles.output = hObject;

if isempty(varargin)
  warning('WarnTEOTraGen:noInputGUI', 'No inputs for change_ik_gains');
  handles.kp = 0;
  handles.ko = 0;
else
  % Get previous panels
  handles.kp = varargin{1};
  handles.ko = varargin{2};
end

set(handles.edit_kp,'String', handles.kp);
set(handles.edit_ko,'String', handles.ko);

% Update handles structure
guidata(hObject, handles);

% UIWAIT
uiwait(handles.figure1);


function varargout = change_cart_table_parameters_OutputFcn(hObject, eventdata, handles)
  varargout{1} = handles.kp;
  varargout{2} = handles.ko;
  close(gcf);



function pushbutton_ok_Callback(hObject, eventdata, handles)
handles.kp = str2double(get(handles.edit_kp,'String'));
handles.ko = str2double(get(handles.edit_ko,'String'));
guidata(hObject, handles);

if isequal(get(gcf, 'waitstatus'), 'waiting')
    % The GUI is still in UIWAIT, us UIRESUME
    uiresume(gcf);
else
    % The GUI is no longer waiting, just close it
    close(gcf);
end



function edit_kp_Callback(hObject, eventdata, handles)


function edit_kp_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_ko_Callback(hObject, eventdata, handles)


function edit_ko_CreateFcn(hObject, eventdata, handles)
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
