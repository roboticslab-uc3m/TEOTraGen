function varargout = teo_joystick(varargin)
% TEO_JOYSTICK MATLAB code for teo_joystick.fig
%      TEO_JOYSTICK, by itself, creates a new TEO_JOYSTICK or raises the existing
%      singleton*.
%
%      H = TEO_JOYSTICK returns the handle to a new TEO_JOYSTICK or the handle to
%      the existing singleton*.
%
%      TEO_JOYSTICK('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in TEO_JOYSTICK.M with the given input arguments.
%
%      TEO_JOYSTICK('Property','Value',...) creates a new TEO_JOYSTICK or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before teo_joystick_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to teo_joystick_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help teo_joystick

% Last Modified by GUIDE v2.5 08-Apr-2014 23:31:52

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @teo_joystick_OpeningFcn, ...
                   'gui_OutputFcn',  @teo_joystick_OutputFcn, ...
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


% --- Executes just before teo_joystick is made visible.
function teo_joystick_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to teo_joystick (see VARARGIN)

% Choose default command line output for teo_joystick
handles.output = hObject;

handles.keys = {'f','fl','l','bl','b','br','r','fr','rl','rr'};
handles.current_mode = 0;
handles.prev_mode = 0;
handles.prev_key = 0;
handles.prev_modi = 0;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes teo_joystick wait for user response (see UIRESUME)
% uiwait(handles.figure1);




% --- Outputs from this function are returned to the command line.
function varargout = teo_joystick_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in pushbutton_f.
function pushbutton_f_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_f (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% disp('APRETO F');
if handles.current_mode
%   strcat('handles.pushbutton_', handles.current_mode)
  update_button_color(hObject)
end
% if handles.prev_mode
%   strcat('handles.pushbutton_', handles.prev_mode)
%   reset_buttons_color(strcat('handles.pushbutton_', handles.prev_mode))
% end


% --- Executes on button press in pushbutton_b.
function pushbutton_b_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_b (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if handles.current_mode
  update_button_color(hObject)
end

% --- Executes on button press in pushbutton_r.
function pushbutton_r_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_r (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if handles.current_mode
  update_button_color(hObject)
end

% --- Executes on button press in pushbutton_l.
function pushbutton_l_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_l (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if handles.current_mode
  update_button_color(hObject)
end

% --- Executes on button press in pushbutton_br.
function pushbutton_br_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_br (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if handles.current_mode
  update_button_color(hObject)
end

% --- Executes on button press in pushbutton_fr.
function pushbutton_fr_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_fr (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if handles.current_mode
  update_button_color(hObject)
end

% --- Executes on button press in pushbutton_fl.
function pushbutton_fl_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_fl (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if handles.current_mode
  update_button_color(hObject)
end

% --- Executes on button press in pushbutton_bl.
function pushbutton_bl_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_bl (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if handles.current_mode
  update_button_color(hObject)
end

% --- Executes on button press in pushbutton_rl.
function pushbutton_rl_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_rl (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if handles.current_mode
  update_button_color(hObject)
end

% --- Executes on button press in pushbutton_rr.
function pushbutton_rr_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_rr (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if handles.current_mode
  update_button_color(hObject)
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


% --- Executes on key press with focus on figure1 or any of its controls.
function figure1_WindowKeyPressFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  structure with the following fields (see FIGURE)
%	Key: name of the key that was pressed, in lower case
%	Character: character interpretation of the key(s) that was pressed
%	Modifier: name(s) of the modifier key(s) (i.e., control, shift) pressed
% handles    structure with handles and user data (see GUIDATA)
% val = double(get(hObject,'CurrentCharacter'));


if ~strcmp(handles.prev_key, eventdata.Key),
  if strcmp(eventdata.Key,'w')
    reset_all(hObject, handles);
    handles.current_mode = 'f';
    pushbutton_f_Callback(handles.pushbutton_f, eventdata, handles)
  elseif strcmp(eventdata.Key,'x')
    reset_all(hObject, handles);
    handles.current_mode = 'b';
    pushbutton_b_Callback(handles.pushbutton_b, eventdata, handles)
  elseif strcmp(eventdata.Key,'a') && isempty(eventdata.Modifier)
      reset_all(hObject, handles);
      handles.current_mode = 'l';
      pushbutton_l_Callback(handles.pushbutton_l, eventdata, handles)
  elseif strcmp(eventdata.Key,'a') && strcmp(eventdata.Modifier,'shift')
      reset_all(hObject, handles);
      handles.current_mode = 'rl';
      pushbutton_rl_Callback(handles.pushbutton_rl, eventdata, handles)    
  elseif strcmp(eventdata.Key,'d') && isempty(eventdata.Modifier)
      reset_all(hObject, handles);
      handles.current_mode = 'r';
      pushbutton_r_Callback(handles.pushbutton_r, eventdata, handles)
  elseif strcmp(eventdata.Key,'d') && strcmp(eventdata.Modifier,'shift')
      reset_all(hObject, handles);
      handles.current_mode = 'rr';
      pushbutton_rr_Callback(handles.pushbutton_rr, eventdata, handles)
  elseif strcmp(eventdata.Key,'q')
    reset_all(hObject, handles);
    handles.current_mode = 'fl';
    pushbutton_fl_Callback(handles.pushbutton_fl, eventdata, handles)
  elseif strcmp(eventdata.Key,'e')
    reset_all(hObject, handles);
    handles.current_mode = 'fr';
    pushbutton_fr_Callback(handles.pushbutton_fr, eventdata, handles)
  elseif strcmp(eventdata.Key,'z')
    reset_all(hObject, handles);
    handles.current_mode = 'fr';
    pushbutton_bl_Callback(handles.pushbutton_bl, eventdata, handles)
  elseif strcmp(eventdata.Key,'c')
    reset_all(hObject, handles);
    handles.current_mode = 'fr';
    pushbutton_br_Callback(handles.pushbutton_br, eventdata, handles)
  elseif strcmp(eventdata.Key,'shift')

  else
      reset_all(hObject, handles)
  end
end

handles.prev_key = eventdata.Key;
handles.prev_modi = eventdata.Modifier;

guidata(hObject, handles);

% if handles.prev_mode
%   reset_buttons_color(strcat('handles.pushbutton_', handles.prev_mode))
% end

function reset_all(hObject, handles)
handles.current_mode = 0;
handles.prev_mode = 0;
reset_all_buttons(handles)
guidata(hObject, handles);

function reset_all_buttons(handles)
%Grey: [0.702 0.702 0.702]
for ii=1:length(handles.keys),
  set(handles.(strcat('pushbutton_',handles.keys{ii})),'BackgroundColor', [0.702 0.702 0.702]);
end

function update_button_color(hObject)
%Green: [0 0.6 0]
  set(hObject,'BackgroundColor',[0 0.6 0]);
  
function reset_button_color(hObject)
%Grey: [0.702 0.702 0.702]
set(hObject,'BackgroundColor',[0.702 0.702 0.702]);


% --- Executes on key release with focus on figure1 or any of its controls.
function figure1_WindowKeyReleaseFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  structure with the following fields (see FIGURE)
%	Key: name of the key that was released, in lower case
%	Character: character interpretation of the key(s) that was released
%	Modifier: name(s) of the modifier key(s) (i.e., control, shift) released
% handles    structure with handles and user data (see GUIDATA)
% handles.current_mode = 0;
% guidata(hObject, handles);
% previo = handles.prev_key
% actual = eventdata.Key
% if ~strcmp(handles.prev_key, eventdata.Key)
%   disp('reseteado')
% end
