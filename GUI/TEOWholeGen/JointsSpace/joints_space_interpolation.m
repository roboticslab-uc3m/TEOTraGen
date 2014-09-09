function varargout = joints_space_interpolation(varargin)
% joints_space_interpolation MATLAB code for joints_space_interpolation.fig
%      joints_space_interpolation, by itself, creates a new joints_space_interpolation or raises the existing
%      singleton*.
%
%      H = joints_space_interpolation returns the handle to a new joints_space_interpolation or the handle to
%      the existing singleton*.
%
%      joints_space_interpolation('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in joints_space_interpolation.M with the given input arguments.
%
%      joints_space_interpolation('Property','Value',...) creates a new joints_space_interpolation or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before joints_space_interpolation_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to joints_space_interpolation_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help joints_space_interpolation

% Last Modified by GUIDE v2.5 09-Sep-2014 17:45:29

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @joints_space_interpolation_OpeningFcn, ...
                   'gui_OutputFcn',  @joints_space_interpolation_OutputFcn, ...
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


function joints_space_interpolation_OpeningFcn(hObject, eventdata, handles, varargin)
% Choose default command line output for joints_space_interpolation
handles.output = hObject;

guidata(hObject, handles);


DOF = 26;

handles.q_initial = zeros(DOF,1);
handles.q_final = zeros(DOF,1);
handles.dq_initial = zeros(DOF,1);
handles.dq_final = zeros(DOF,1);
handles.ddq_initial = zeros(DOF,1);
handles.ddq_final = zeros(DOF,1);
handles.insert_option = 0;

handles.input_data_type = 'Poses';

if isempty(varargin)
  %warning('WarnTEOTrajGen:noInputGUI', 'No joints_space_interpolation inputs');
  for ii = 1:DOF,
    set(handles.(strcat('edit_initial_',num2str(ii))),'String', num2str(0));
    set(handles.(strcat('edit_final_',num2str(ii))),'String', num2str(0));
  end
  handles.humanoid = TEO_structure('numeric', 'rad', 'm');
  handles.h = TEO_kinematics_library;
  
  set(handles.uipanel_others, 'Visible', 'off');

  % Update handles structure
  guidata(hObject, handles);
else
  if length(varargin) == 5,
    handles.humanoid = varargin{1};
    handles.h = varargin{2};
    if length(varargin{3}) == DOF,
      for ii = 1:DOF,
        handles.q_final(ii) = varargin{3}(ii);
        set(handles.(strcat('edit_final_',num2str(ii))),'String', varargin{3}(ii));
      end
    else
      error('ErrorTEOTraGen:wrongInputGUI', 'Wrong DOF of humanoid final pose');
    end
      
    if length(varargin{4}) == DOF,
      for ii = 1:DOF,
        handles.q_initial(ii) = varargin{4}(ii);
        set(handles.(strcat('edit_initial_',num2str(ii))),'String', varargin{4}(ii));
      end 
    else
      error('ErrorTEOTraGen:wrongInputGUI', 'Wrong DOF of humanoid initial pose');
    end
    
    set(handles.edit_Ts, 'String', num2str(varargin{5}));
    
    handles.insert_option = 1;
    % Update handles structure
    guidata(hObject, handles);
    % UIWAIT
    uiwait(handles.figure1);
  elseif length(varargin) == 4,
    handles.humanoid = varargin{1};
    handles.h = varargin{2};
    if length(varargin{3}) == DOF,
      for ii = 1:DOF,
        handles.q_final(ii) = varargin{3}(ii);
        set(handles.(strcat('edit_final_',num2str(ii))),'String', varargin{3}(ii));
      end
    else
      error('ErrorTEOTraGen:wrongInputGUI', 'Wrong DOF of humanoid final pose');
    end
      
    if length(varargin{4}) == DOF,
      for ii = 1:DOF,
        handles.q_initial(ii) = varargin{4}(ii);
        set(handles.(strcat('edit_initial_',num2str(ii))),'String', varargin{4}(ii));
      end 
    else
      error('ErrorTEOTraGen:wrongInputGUI', 'Wrong DOF of humanoid initial pose');
    end
    
    handles.insert_option = 1;
    % Update handles structure
    guidata(hObject, handles);
    % UIWAIT
    uiwait(handles.figure1);
  elseif length(varargin) == 3,
    handles.humanoid = varargin{1};
    handles.h = varargin{2};
    if length(varargin{3}) == DOF,
      for ii = 1:DOF,
        handles.q_final(ii) = varargin{3}(ii);
        set(handles.(strcat('edit_final_',num2str(ii))),'String', varargin{3}(ii));
        set(handles.(strcat('edit_initial_',num2str(ii))),'String', 0);
      end
    else
      error('ErrorTEOTraGen:wrongInputGUI', 'Wrong DOF of humanoid initial pose');
    end
    
    handles.insert_option = 1;
    % Update handles structure
    guidata(hObject, handles);
    % UIWAIT
    uiwait(handles.figure1);
    
  elseif length(varargin) == 2,
    handles.humanoid = varargin{1};
    handles.h = varargin{2};
    % Update handles structure
    guidata(hObject, handles);
    
  elseif length(varargin) == 1,
    handles.humanoid = varargin{1};
    handles.h = TEO_kinematics_library;
    % Update handles structure
    guidata(hObject, handles);
    
  else
    error('ErrorTEOTraGen:wrongInputGUI', 'Wrong inputs option');
  end
end






function varargout = joints_space_interpolation_OutputFcn(hObject, eventdata, handles)
% varargout{1} = handles.output;
if handles.insert_option == 1,
  if handles.trajectory_generated == 1
    varargout{1} = handles.q;
    varargout{2} = handles.dq;
    varargout{3} = handles.ddq;
    varargout{4} = handles.sf;
  else
    varargout{1} = [];
    varargout{2} = [];
    varargout{3} = [];
    varargout{4} = [];
  end
  close(gcf);
else
  varargout{1} = handles.output;
end



function pushbutton_insert_Callback(hObject, eventdata, handles)
% global q dq ddq
% global q_init q_final
% 
% DOF = 26;
% 
% % Get the trajectory parameters
% Ts = str2double(get(handles.edit_Ts,'String'));
% T = str2double(get(handles.edit_T,'String'));
% 
% % Check introduced values
% if ~check_input_value(Ts, 'Sample Time (Ts)')
%   return;
% end
% if ~check_input_value(T, 'Total Time (T)')
%   return;
% end
% 
% tt = 0:Ts:(T);
% 
% x = zeros(length(tt),DOF);
% dx = zeros(length(tt),DOF);
% ddx = zeros(length(tt),DOF);
% 
% for ii = 1:DOF,
%   q_init(ii,1) = str2double(get(handles.(strcat('edit_initial_',num2str(ii))),'String'));
%   q_final(ii,1) = str2double(get(handles.(strcat('edit_final_',num2str(ii))),'String'));
%   
%   
%   % hObject    handle to popupmenu_interpolation7 (see GCBO)
% % eventdata  reserved - to be defined in a future version of MATLAB
% % handles    structure with handles and user data (see GUIDATA)
% 
% % Hints: contents = cellstr(get(hObject,'String')) returns popupmenu_interpolation7 contents as cell array
% %        contents{get(hObject,'Value')} returns selected item from popupmenu_interpolation7
%   contents_interp_options = cellstr(get(handles.(strcat('popupmenu_interpolation',num2str(ii))),'String'));
%   interpolation = contents_interp_options{get(handles.(strcat('popupmenu_interpolation',num2str(ii))),'Value')};
%   
%   switch interpolation
%     case 'Polynomial3'
%       [x(:,ii), dx(:,ii), ddx(:,ii)] = poly3_traj(q_init(ii,1), q_final(ii,1), tt, 0, 0);
%     case 'Polynomial5'
%       [x(:,ii), dx(:,ii), ddx(:,ii)] = poly5_traj(q_init(ii,1), q_final(ii,1), tt, 0, 0);
%     case 'Polynomial7'
%       [x(:,ii), dx(:,ii), ddx(:,ii)] = poly7_traj(q_init(ii,1), q_final(ii,1), tt, 0, 0);
%   end
% end
% 
% q = x';
% dq = dx';
% ddq = ddx';


handles.trajectory_generated = 1;
handles.insert_option = 1;
guidata(hObject, handles);

if isequal(get(gcf, 'waitstatus'), 'waiting')
  % The GUI is still in UIWAIT, us UIRESUME
  uiresume(gcf);
else
  % The GUI is no longer waiting, just close it
  close(gcf);
end



function ok = check_input_value(value, name, option)
if nargin < 3
  option = 'full';
end

if strcmp(option,'full')
  if isnan(value)
    errordlg(strcat('ERROR: ', name, ' value is not a number'),strcat(name, ' value Error'));
    warning('WarnTEOTraGen:wrongInputObject',strcat(name,' introduced is not a number'));
    ok = 0;
  elseif value == 0,
    errordlg(strcat('ERROR: ', name, ' cannot be 0'),strcat(name,' value Error'));
    warning('WarnTEOTraGen:wrongInputObject',strcat(name,' introduced cannot be zero.'));
    ok = 0;
  elseif value < 0,
    errordlg(strcat('ERROR: ', name, ' cannot be negative'),strcat(name,' value Error'));
    warning('WarnTEOTraGen:wrongInputObject',strcat(name,' introduced cannot be negative.'));
    ok = 0;
  else
    ok = 1;
  end
  
elseif strcmp(option,'nan')
  if isnan(value)
    errordlg(strcat('ERROR: ', name, ' value is not a number'),strcat(name, ' value Error'));
    warning('WarnTEOTraGen:wrongInputObject',strcat(name,' introduced is not a number'));
    ok = 0;
  else
    ok = 1;
  end
  
elseif strcmp(option,'zero')
  if value == 0,
    errordlg(strcat('ERROR: ', name, ' cannot be 0'),strcat(name,' value Error'));
    warning('WarnTEOTraGen:wrongInputObject',strcat(name,' introduced cannot be zero.'));
    ok = 0;
  else
    ok = 1;
  end
  
elseif strcmp(option,'negative')
  if value < 0,
    errordlg(strcat('ERROR: ', name, ' cannot be negative'),strcat(name,' value Error'));
    warning('WarnTEOTraGen:wrongInputObject',strcat(name,' introduced cannot be negative.'));
    ok = 0;
  else
    ok = 1;
  end
  
end



function edit_initial_1_Callback(hObject, eventdata, handles)


function edit_initial_1_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function pushbutton_cancel_Callback(hObject, eventdata, handles)
handles.trajectory_generated = 0;
guidata(hObject, handles);
if isequal(get(gcf, 'waitstatus'), 'waiting')
  % The GUI is still in UIWAIT, us UIRESUME
  uiresume(gcf);
else
  % The GUI is no longer waiting, just close it
  close(gcf);
end


function edit_initial_4_Callback(hObject, eventdata, handles)

function edit_initial_4_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit_initial_2_Callback(hObject, eventdata, handles)

function edit_initial_2_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_initial_3_Callback(hObject, eventdata, handles)

function edit_initial_3_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit_7_Callback(hObject, eventdata, handles)

function edit_7_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit_8_Callback(hObject, eventdata, handles)

function edit_8_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit_9_Callback(hObject, eventdata, handles)

function edit_9_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit_initial_13_Callback(hObject, eventdata, handles)

function edit_initial_13_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit_15_Callback(hObject, eventdata, handles)

function edit_15_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit_16_Callback(hObject, eventdata, handles)

function edit_16_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit_17_Callback(hObject, eventdata, handles)

function edit_17_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit_21_Callback(hObject, eventdata, handles)

function edit_21_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit_initial_5_Callback(hObject, eventdata, handles)

function edit_initial_5_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit_initial_6_Callback(hObject, eventdata, handles)

function edit_initial_6_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit_10_Callback(hObject, eventdata, handles)

function edit_10_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit_11_Callback(hObject, eventdata, handles)

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

function edit_14_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_18_Callback(hObject, eventdata, handles)

function edit_18_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit_19_Callback(hObject, eventdata, handles)

function edit_19_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_20_Callback(hObject, eventdata, handles)

function edit_20_CreateFcn(hObject, eventdata, handles)
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

function edit_22_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_23_Callback(hObject, eventdata, handles)

function edit_23_CreateFcn(hObject, eventdata, handles)
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
% hObject    handle to edit_initial_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_initial_1 as text
%        str2double(get(hObject,'String')) returns contents of edit_initial_1 as a double


% --- Executes during object creation, after setting all properties.
function edit37_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_initial_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit38_Callback(hObject, eventdata, handles)
% hObject    handle to edit_initial_4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_initial_4 as text
%        str2double(get(hObject,'String')) returns contents of edit_initial_4 as a double


% --- Executes during object creation, after setting all properties.
function edit38_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_initial_4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
  set(hObject,'BackgroundColor','white');
end



function edit39_Callback(hObject, eventdata, handles)
% hObject    handle to edit_initial_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_initial_2 as text
%        str2double(get(hObject,'String')) returns contents of edit_initial_2 as a double


% --- Executes during object creation, after setting all properties.
function edit39_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_initial_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit40_Callback(hObject, eventdata, handles)
% hObject    handle to edit_initial_3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_initial_3 as text
%        str2double(get(hObject,'String')) returns contents of edit_initial_3 as a double


% --- Executes during object creation, after setting all properties.
function edit40_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_initial_3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit41_Callback(hObject, eventdata, handles)
% hObject    handle to edit_initial_5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_initial_5 as text
%        str2double(get(hObject,'String')) returns contents of edit_initial_5 as a double


% --- Executes during object creation, after setting all properties.
function edit41_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_initial_5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit42_Callback(hObject, eventdata, handles)
% hObject    handle to edit_initial_6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_initial_6 as text
%        str2double(get(hObject,'String')) returns contents of edit_initial_6 as a double


% --- Executes during object creation, after setting all properties.
function edit42_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_initial_6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
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



function edit_final_4_Callback(hObject, eventdata, handles)
% hObject    handle to edit_final_4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_final_4 as text
%        str2double(get(hObject,'String')) returns contents of edit_final_4 as a double


% --- Executes during object creation, after setting all properties.
function edit_final_4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_final_4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_final_3_Callback(hObject, eventdata, handles)
% hObject    handle to edit_final_3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_final_3 as text
%        str2double(get(hObject,'String')) returns contents of edit_final_3 as a double


% --- Executes during object creation, after setting all properties.
function edit_final_3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_final_3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_final_5_Callback(hObject, eventdata, handles)
% hObject    handle to edit_final_5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_final_5 as text
%        str2double(get(hObject,'String')) returns contents of edit_final_5 as a double


% --- Executes during object creation, after setting all properties.
function edit_final_5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_final_5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_final_6_Callback(hObject, eventdata, handles)
% hObject    handle to edit_final_6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_final_6 as text
%        str2double(get(hObject,'String')) returns contents of edit_final_6 as a double


% --- Executes during object creation, after setting all properties.
function edit_final_6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_final_6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_final_1_Callback(hObject, eventdata, handles)
% hObject    handle to edit_final_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_final_1 as text
%        str2double(get(hObject,'String')) returns contents of edit_final_1 as a double


% --- Executes during object creation, after setting all properties.
function edit_final_1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_final_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_final_2_Callback(hObject, eventdata, handles)
% hObject    handle to edit_final_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_final_2 as text
%        str2double(get(hObject,'String')) returns contents of edit_final_2 as a double


% --- Executes during object creation, after setting all properties.
function edit_final_2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_final_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in popupmenu_interpolation1.
function popupmenu_interpolation1_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu_interpolation1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu_interpolation1 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu_interpolation1


% --- Executes during object creation, after setting all properties.
function popupmenu_interpolation1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu_interpolation1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in popupmenu_interpolation2.
function popupmenu_interpolation2_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu_interpolation2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu_interpolation2 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu_interpolation2


% --- Executes during object creation, after setting all properties.
function popupmenu_interpolation2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu_interpolation2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in popupmenu_interpolation3.
function popupmenu_interpolation3_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu_interpolation3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu_interpolation3 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu_interpolation3


% --- Executes during object creation, after setting all properties.
function popupmenu_interpolation3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu_interpolation3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in popupmenu_interpolation4.
function popupmenu_interpolation4_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu_interpolation4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu_interpolation4 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu_interpolation4


% --- Executes during object creation, after setting all properties.
function popupmenu_interpolation4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu_interpolation4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in popupmenu_interpolation5.
function popupmenu_interpolation5_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu_interpolation5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu_interpolation5 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu_interpolation5


% --- Executes during object creation, after setting all properties.
function popupmenu_interpolation5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu_interpolation5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in popupmenu_interpolation6.
function popupmenu_interpolation6_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu_interpolation6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu_interpolation6 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu_interpolation6


% --- Executes during object creation, after setting all properties.
function popupmenu_interpolation6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu_interpolation6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_initial_10_Callback(hObject, eventdata, handles)
% hObject    handle to edit_initial_10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_initial_10 as text
%        str2double(get(hObject,'String')) returns contents of edit_initial_10 as a double


% --- Executes during object creation, after setting all properties.
function edit_initial_10_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_initial_10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_initial_9_Callback(hObject, eventdata, handles)
% hObject    handle to edit_initial_9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_initial_9 as text
%        str2double(get(hObject,'String')) returns contents of edit_initial_9 as a double


% --- Executes during object creation, after setting all properties.
function edit_initial_9_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_initial_9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_initial_11_Callback(hObject, eventdata, handles)
% hObject    handle to edit_initial_11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_initial_11 as text
%        str2double(get(hObject,'String')) returns contents of edit_initial_11 as a double


% --- Executes during object creation, after setting all properties.
function edit_initial_11_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_initial_11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_initial_12_Callback(hObject, eventdata, handles)
% hObject    handle to edit_initial_12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_initial_12 as text
%        str2double(get(hObject,'String')) returns contents of edit_initial_12 as a double


% --- Executes during object creation, after setting all properties.
function edit_initial_12_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_initial_12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_final_10_Callback(hObject, eventdata, handles)
% hObject    handle to edit_final_10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_final_10 as text
%        str2double(get(hObject,'String')) returns contents of edit_final_10 as a double


% --- Executes during object creation, after setting all properties.
function edit_final_10_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_final_10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_final_9_Callback(hObject, eventdata, handles)
% hObject    handle to edit_final_9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_final_9 as text
%        str2double(get(hObject,'String')) returns contents of edit_final_9 as a double


% --- Executes during object creation, after setting all properties.
function edit_final_9_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_final_9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_final_11_Callback(hObject, eventdata, handles)
% hObject    handle to edit_final_11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_final_11 as text
%        str2double(get(hObject,'String')) returns contents of edit_final_11 as a double


% --- Executes during object creation, after setting all properties.
function edit_final_11_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_final_11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_final_12_Callback(hObject, eventdata, handles)
% hObject    handle to edit_final_12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_final_12 as text
%        str2double(get(hObject,'String')) returns contents of edit_final_12 as a double


% --- Executes during object creation, after setting all properties.
function edit_final_12_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_final_12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_initial_7_Callback(hObject, eventdata, handles)
% hObject    handle to edit_initial_7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_initial_7 as text
%        str2double(get(hObject,'String')) returns contents of edit_initial_7 as a double


% --- Executes during object creation, after setting all properties.
function edit_initial_7_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_initial_7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_initial_8_Callback(hObject, eventdata, handles)
% hObject    handle to edit_initial_8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_initial_8 as text
%        str2double(get(hObject,'String')) returns contents of edit_initial_8 as a double


% --- Executes during object creation, after setting all properties.
function edit_initial_8_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_initial_8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_final_7_Callback(hObject, eventdata, handles)
% hObject    handle to edit_final_7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_final_7 as text
%        str2double(get(hObject,'String')) returns contents of edit_final_7 as a double


% --- Executes during object creation, after setting all properties.
function edit_final_7_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_final_7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_final_8_Callback(hObject, eventdata, handles)
% hObject    handle to edit_final_8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_final_8 as text
%        str2double(get(hObject,'String')) returns contents of edit_final_8 as a double


% --- Executes during object creation, after setting all properties.
function edit_final_8_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_final_8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in popupmenu_interpolation7.
function popupmenu_interpolation7_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu_interpolation7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu_interpolation7 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu_interpolation7


% --- Executes during object creation, after setting all properties.
function popupmenu_interpolation7_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu_interpolation7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in popupmenu_interpolation8.
function popupmenu_interpolation8_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu_interpolation8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu_interpolation8 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu_interpolation8


% --- Executes during object creation, after setting all properties.
function popupmenu_interpolation8_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu_interpolation8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in popupmenu_interpolation9.
function popupmenu_interpolation9_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu_interpolation9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu_interpolation9 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu_interpolation9


% --- Executes during object creation, after setting all properties.
function popupmenu_interpolation9_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu_interpolation9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in popupmenu_interpolation10.
function popupmenu_interpolation10_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu_interpolation10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu_interpolation10 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu_interpolation10


% --- Executes during object creation, after setting all properties.
function popupmenu_interpolation10_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu_interpolation10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in popupmenu_interpolation11.
function popupmenu_interpolation11_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu_interpolation11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu_interpolation11 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu_interpolation11


% --- Executes during object creation, after setting all properties.
function popupmenu_interpolation11_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu_interpolation11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in popupmenu_interpolation12.
function popupmenu_interpolation12_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu_interpolation12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu_interpolation12 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu_interpolation12


% --- Executes during object creation, after setting all properties.
function popupmenu_interpolation12_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu_interpolation12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit_final_13_Callback(hObject, eventdata, handles)

function edit_final_13_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function popupmenu_interpolation13_Callback(hObject, eventdata, handles)

function popupmenu_interpolation13_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit_initial_14_Callback(hObject, eventdata, handles)

function edit_initial_14_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit_final_14_Callback(hObject, eventdata, handles)

function edit_final_14_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function popupmenu_interpolation14_Callback(hObject, eventdata, handles)

function popupmenu_interpolation14_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit_initial_24_Callback(hObject, eventdata, handles)

function edit_initial_24_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit_initial_23_Callback(hObject, eventdata, handles)

function edit_initial_23_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit_initial_25_Callback(hObject, eventdata, handles)

function edit_initial_25_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit_initial_26_Callback(hObject, eventdata, handles)

function edit_initial_26_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit_final_24_Callback(hObject, eventdata, handles)

function edit_final_24_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit_final_23_Callback(hObject, eventdata, handles)

function edit_final_23_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit_final_25_Callback(hObject, eventdata, handles)

function edit_final_25_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit_final_26_Callback(hObject, eventdata, handles)

function edit_final_26_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_initial_21_Callback(hObject, eventdata, handles)

function edit_initial_21_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit_initial_22_Callback(hObject, eventdata, handles)

function edit_initial_22_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit_final_21_Callback(hObject, eventdata, handles)

function edit_final_21_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit_final_22_Callback(hObject, eventdata, handles)

function edit_final_22_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function popupmenu_interpolation21_Callback(hObject, eventdata, handles)

function popupmenu_interpolation21_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function popupmenu_interpolation22_Callback(hObject, eventdata, handles)

function popupmenu_interpolation22_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function popupmenu_interpolation23_Callback(hObject, eventdata, handles)

function popupmenu_interpolation23_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function popupmenu_interpolation24_Callback(hObject, eventdata, handles)

function popupmenu_interpolation24_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function popupmenu_interpolation25_Callback(hObject, eventdata, handles)

function popupmenu_interpolation25_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function popupmenu_interpolation26_Callback(hObject, eventdata, handles)

function popupmenu_interpolation26_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit_initial_18_Callback(hObject, eventdata, handles)

function edit_initial_18_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit_initial_17_Callback(hObject, eventdata, handles)

function edit_initial_17_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit_initial_19_Callback(hObject, eventdata, handles)

function edit_initial_19_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit_initial_20_Callback(hObject, eventdata, handles)

function edit_initial_20_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit_final_18_Callback(hObject, eventdata, handles)

function edit_final_18_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit_final_17_Callback(hObject, eventdata, handles)

function edit_final_17_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit_final_19_Callback(hObject, eventdata, handles)

function edit_final_19_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit_final_20_Callback(hObject, eventdata, handles)

function edit_final_20_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit_initial_15_Callback(hObject, eventdata, handles)

function edit_initial_15_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit_initial_16_Callback(hObject, eventdata, handles)

function edit_initial_16_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit_final_15_Callback(hObject, eventdata, handles)

function edit_final_15_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit_final_16_Callback(hObject, eventdata, handles)

function edit_final_16_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function popupmenu_interpolation15_Callback(hObject, eventdata, handles)

function popupmenu_interpolation15_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function popupmenu_interpolation16_Callback(hObject, eventdata, handles)

function popupmenu_interpolation16_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function popupmenu_interpolation17_Callback(hObject, eventdata, handles)

function popupmenu_interpolation17_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function popupmenu_interpolation18_Callback(hObject, eventdata, handles)

function popupmenu_interpolation18_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function popupmenu_interpolation19_Callback(hObject, eventdata, handles)

function popupmenu_interpolation19_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function popupmenu_interpolation20_Callback(hObject, eventdata, handles)

function popupmenu_interpolation20_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit_Ts_Callback(hObject, eventdata, handles)

function edit_Ts_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit_T_Callback(hObject, eventdata, handles)

function edit_T_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Untitled_1_Callback(hObject, eventdata, handles)

function insert_initial_Callback(hObject, eventdata, handles)

function Untitled_4_Callback(hObject, eventdata, handles)



% --------------------------------------------------------------------
function insert_final_csv_Callback(hObject, eventdata, handles)
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
        set(handles.(strcat('edit_final_',num2str(ii))),'String', handles.q(ii));
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


function insert_initial_csv_Callback(hObject, eventdata, handles)
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
        set(handles.(strcat('edit_initial_',num2str(ii))),'String', handles.q(ii));
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


function pushbutton_interpolate_Callback(hObject, eventdata, handles)
global q dq ddq
global q_init q_final
global trajectory

DOF = 26;

% Get the trajectory parameters
Ts = str2double(get(handles.edit_Ts,'String'));
T = str2double(get(handles.edit_T,'String'));

  contents_interp_options = cellstr(get(handles.popupmenu_support,'String'));
  support_foot = contents_interp_options{get(handles.popupmenu_support,'Value')};

% Check introduced values
if ~check_input_value(Ts, 'Sample Time (Ts)')
  return;
end
if ~check_input_value(T, 'Total Time (T)')
  return;
end

tt = 0:Ts:(T);

x = zeros(length(tt),DOF);
dx = zeros(length(tt),DOF);
ddx = zeros(length(tt),DOF);

% Get the selection of the Input Data type
contents = cellstr(get(handles.popupmenu_input_data_type,'String'));
new_input_data_type = contents{get(handles.popupmenu_input_data_type,'Value')};
handles = get_input_data (hObject, handles, new_input_data_type);
guidata(hObject, handles);


for ii = 1:DOF,
%   q_init(ii,1) = str2double(get(handles.(strcat('edit_initial_',num2str(ii))),'String'));
%   q_final(ii,1) = str2double(get(handles.(strcat('edit_final_',num2str(ii))),'String'));
  
  contents_interp_options = cellstr(get(handles.(strcat('popupmenu_interpolation',num2str(ii))),'String'));
  interpolation = contents_interp_options{get(handles.(strcat('popupmenu_interpolation',num2str(ii))),'Value')};
  
  switch interpolation
    case 'Polynomial3'
%       [x(:,ii), dx(:,ii), ddx(:,ii)] = poly3_traj(q_init(ii,1), q_final(ii,1), tt, 0, 0);
      [x(:,ii), dx(:,ii), ddx(:,ii)] = poly3_traj(handles.q_initial(ii,1), handles.q_final(ii,1), tt, handles.q_initial(ii,1), handles.dq_final(ii,1), handles.ddq_initial(ii,1), handles.ddq_final(ii,1));
    case 'Polynomial5'
%       [x(:,ii), dx(:,ii), ddx(:,ii)] = poly5_traj(q_init(ii,1), q_final(ii,1), tt, 0, 0);
      [x(:,ii), dx(:,ii), ddx(:,ii)] = poly5_traj(handles.q_initial(ii,1), handles.q_final(ii,1), tt, handles.q_initial(ii,1), handles.dq_final(ii,1), handles.ddq_initial(ii,1), handles.ddq_final(ii,1));
    case 'Polynomial7'
%       [x(:,ii), dx(:,ii), ddx(:,ii)] = poly7_traj(q_init(ii,1), q_final(ii,1), tt, 0, 0);
      [x(:,ii), dx(:,ii), ddx(:,ii)] = poly7_traj(handles.q_initial(ii,1), handles.q_final(ii,1), tt, handles.q_initial(ii,1), handles.dq_final(ii,1), handles.ddq_initial(ii,1), handles.ddq_final(ii,1));
  end
end

q = x';
dq = dx';
ddq = ddx';

% Save in handles
handles.q = q;
handles.dq = dq;
handles.ddq = ddq;

% Operational Trajectory
[trajectory, d_trajectory, dd_trajectory] = operational_traj_from_joints( q, dq, ddq, Ts, handles.h, support_foot, tt);

handles.trajectory = trajectory;
handles.d_trajectory = d_trajectory;
handles.dd_trajectory = dd_trajectory;

handles.Ts = Ts;

% Support foot
if ~isnumeric(support_foot),
  switch support_foot
    case 'Double Support'
      support_foot = 0;
    case 'Right Foot Support'
      support_foot = 1;
    case 'Left Foot Support'
      support_foot = -1;
    otherwise
      error('ErrorTEOTraGen:wrongInputGUI', 'Wrong support_foot value');
  end
else
  if ~((support_foot == 0) || (support_foot == 1) || (support_foot == -1))
    error('ErrorTEOTraGen:wrongInputGUI', 'Wrong support_foot value');
  end
end

handles.sf = support_foot*ones(1,size(q,2));

set(handles.pushbutton_insert,'Enable','On')

guidata(hObject, handles);


function popupmenu_support_Callback(hObject, eventdata, handles)
  contents = cellstr(get(hObject,'String'));
  handles.xx = contents{get(hObject,'Value')};


function popupmenu_support_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function pushbutton12_Callback(hObject, eventdata, handles)
global trajectories data
trajectory_plots(trajectories.operational, trajectories.joints, data.trajectory_settings.TEO);


function pushbutton_plot_Callback(hObject, eventdata, handles)
operational.trajectory = handles.trajectory;
operational.d_trajectory = handles.d_trajectory;
operational.dd_trajectory = handles.dd_trajectory;

joints.q = handles.q;
joints.dq = handles.dq;
joints.ddq = handles.ddq;
trajectory_plots(operational, joints, handles.humanoid);


function pushbutton_save_trajectories_Callback(hObject, eventdata, handles)
traj_name_string = 'joint_trajectory';
save_trajectory_window(traj_name_string, handles.q, handles.dq, handles.ddq, 'joints');


function popupmenu_input_data_type_Callback(hObject, eventdata, handles)

handles = get_input_data (hObject, handles, handles.input_data_type);

% Get the selection of the Input Data type
contents = cellstr(get(hObject,'String'));
new_input_data_type = contents{get(hObject,'Value')};

handles = set_input_data (hObject, handles, new_input_data_type);

guidata(hObject, handles);


function handles = get_input_data (hObject, handles, input_data_type)

DOF = 26;

switch input_data_type
  case 'Poses'
    for ii = 1:DOF,
      handles.q_initial(ii) = str2double(get(handles.(strcat('edit_initial_',num2str(ii))),'String'));
      handles.q_final(ii) = str2double(get(handles.(strcat('edit_final_',num2str(ii))),'String'));
    end 
  case 'Velocities'
  	for ii = 1:DOF,
      handles.dq_initial(ii) = str2double(get(handles.(strcat('edit_initial_',num2str(ii))),'String'));
      handles.dq_final(ii) = str2double(get(handles.(strcat('edit_final_',num2str(ii))),'String'));
    end 
  case 'Accelerations'
    for ii = 1:DOF,
      handles.dq_initial(ii) = str2double(get(handles.(strcat('edit_initial_',num2str(ii))),'String'));
      handles.ddq_final(ii) = str2double(get(handles.(strcat('edit_final_',num2str(ii))),'String'));
    end 
  otherwise
    disp('Wrong input data type')
end

guidata(hObject, handles);

function handles = set_input_data (hObject, handles, new_input_data_type)

DOF = 26;

switch new_input_data_type
  case 'Poses'
    for ii = 1:DOF,
      set(handles.(strcat('edit_initial_',num2str(ii))),'String', handles.q_initial(ii));
      set(handles.(strcat('edit_final_',num2str(ii))),'String', handles.q_final(ii));
    end 
  case 'Velocities'
  	for ii = 1:DOF,
      set(handles.(strcat('edit_initial_',num2str(ii))),'String', handles.dq_initial(ii));
      set(handles.(strcat('edit_final_',num2str(ii))),'String', handles.dq_final(ii));
    end 
  case 'Accelerations'
    for ii = 1:DOF,
      set(handles.(strcat('edit_initial_',num2str(ii))),'String', handles.ddq_initial(ii));
      set(handles.(strcat('edit_final_',num2str(ii))),'String', handles.ddq_final(ii));
    end 
  otherwise
    disp('Wrong input data type')
end

handles.input_data_type = new_input_data_type;

guidata(hObject, handles);



% --- Executes during object creation, after setting all properties.
function popupmenu_input_data_type_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu_input_data_type (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function pushbutton_ros_visualization_Callback(hObject, eventdata, handles)
ros_visualization(handles.q, handles.trajectory.SF, handles.Ts);


% --------------------------------------------------------------------
function Back_Callback(hObject, eventdata, handles)
% hObject    handle to Back (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function back_teowholegen_Callback(hObject, eventdata, handles)
% TEOWholeGen
close (gcf)
TEOWholeGen

% --------------------------------------------------------------------
function main_menu_Callback(hObject, eventdata, handles)
close (gcf)
TEOTraGen
