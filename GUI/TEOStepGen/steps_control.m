function varargout = steps_control(varargin)
% STEPS_CONTROL MATLAB code for steps_control.fig
%      STEPS_CONTROL, by itself, creates a new STEPS_CONTROL or raises the existing
%      singleton*.
%
%      H = STEPS_CONTROL returns the handle to a new STEPS_CONTROL or the handle to
%      the existing singleton*.
%
%      STEPS_CONTROL('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in STEPS_CONTROL.M with the given input arguments.
%
%      STEPS_CONTROL('Property','Value',...) creates a new STEPS_CONTROL or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before steps_control_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to steps_control_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help steps_control

% Last Modified by GUIDE v2.5 11-Sep-2013 21:04:48



% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @steps_control_OpeningFcn, ...
                   'gui_OutputFcn',  @steps_control_OutputFcn, ...
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


% --- Executes just before steps_control is made visible.
function steps_control_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to steps_control (see VARARGIN)

% Choose default command line output for steps_control
handles.output = hObject;

% Put window correctly
movegui(gcf,'center')

%Default variables
 handles.uipanelInitialPosition='Default';
 handles.uipanelSimpleDoubleSupport='Double and Simple';
 handles.uipanelLegSupport='Right Leg';

% Update handles structure
guidata(hObject, handles);

% Default q0
global Input_data
q0 = [0; 0.00325683448936741; -0.308647699300050; 0.796421295515307; -0.487773596215257; 0.0278918646012491;...  % Right Leg
                     0; 0.00325683448936741; -0.311486990906165; 0.796421295515307; -0.484850796032492; -0.0354911450764397;...  % Left Leg
                     0.0349065850398866; 0;...                                                                                   % Waist
                     -0.261799387799149; -0.167017153300893; 0; -0.734875474523928; 0; 0;...                                     % Right Arm
                     -0.261799387799149; 0.167017153300893; 0;  -0.734875474523928; 0; 0];                                       % Left Arm ;
Input_data.q0 = q0;

% Show UC3M logo
axes(handles.UC3M_logo)
[r,map]=imread('uc3m.png','png');
image(r);colormap(map);axis off


% UIWAIT makes steps_control wait for user response (see UIRESUME)
% uiwait(handles.figureStepsControl);


% --- Outputs from this function are returned to the command line.
function varargout = steps_control_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in radiobuttonInitialStepDefault.
function radiobuttonInitialStepDefault_Callback(hObject, eventdata, handles)
% hObject    handle to radiobuttonInitialStepDefault (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of radiobuttonInitialStepDefault


% --- Executes on button press in radiobuttonInitialStepFile.
function radiobuttonInitialStepFile_Callback(hObject, eventdata, handles)
% hObject    handle to radiobuttonInitialStepFile (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of radiobuttonInitialStepFile



function editStepLength_Callback(hObject, eventdata, handles)
% hObject    handle to editStepLength (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editStepLength as text
%        str2double(get(hObject,'String')) returns contents of editStepLength as a double


% --- Executes during object creation, after setting all properties.
function editStepLength_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editStepLength (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function editStepHigh_Callback(hObject, eventdata, handles)
% hObject    handle to editStepHigh (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editStepHigh as text
%        str2double(get(hObject,'String')) returns contents of editStepHigh as a double


% --- Executes during object creation, after setting all properties.
function editStepHigh_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editStepHigh (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function editStepTimeT_Callback(hObject, eventdata, handles)
% hObject    handle to editStepTimeT (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editStepTimeT as text
%        str2double(get(hObject,'String')) returns contents of editStepTimeT as a double


% --- Executes during object creation, after setting all properties.
function editStepTimeT_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editStepTimeT (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function editStepTimeTs_Callback(hObject, eventdata, handles)
% hObject    handle to editStepTimeTs (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editStepTimeTs as text
%        str2double(get(hObject,'String')) returns contents of editStepTimeTs as a double


% --- Executes during object creation, after setting all properties.
function editStepTimeTs_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editStepTimeTs (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function pushbutton_step_design_Callback(hObject, eventdata, handles)
global Input_data

conversion = pi/180/209;
Input_data.Leg = handles.uipanelLegSupport;
Input_data.DS_or_SS = handles.uipanelSimpleDoubleSupport;
Input_data.L_val = str2double(get(handles.editStepLength,'String'));
Input_data.H_val = str2double(get(handles.editStepHigh,'String'));
Input_data.T_val = str2double(get(handles.editStepTimeT,'String'));
Input_data.Ts_val = str2double(get(handles.editStepTimeTs,'String'));
Input_data.alpha_ds = 1/3; % Percentage time for double support
Input_data.alpha_sf = 0.2; % Percentage of movement in double support
design_step(Input_data);
close(handles.figureStepsControl)


function uipanelLegSupport_SelectionChangeFcn(hObject, eventdata, handles)
new_val = get(eventdata.NewValue,'String');
old_val = get(eventdata.OldValue,'String');
handles.uipanelLegSupport = new_val;
guidata(hObject,handles)


% --- Executes when selected object is changed in uipanelSimpleDoubleSupport.
function uipanelSimpleDoubleSupport_SelectionChangeFcn(hObject, eventdata, handles)
new_val = get(eventdata.NewValue,'String');
old_val = get(eventdata.OldValue,'String');
handles.uipanelSimpleDoubleSupport = new_val;
guidata(hObject,handles)


% --- Executes when selected object is changed in uipanelInitialPosition.
function uipanelInitialPosition_SelectionChangeFcn(hObject, eventdata, handles)
global Input_data

new_val = get(eventdata.NewValue,'String');
% old_val = get(eventdata.OldValue,'String');

CSVerror = 0;

% Default q0
q0 = [0; 0.00325683448936741; -0.308647699300050; 0.796421295515307; -0.487773596215257; 0.0278918646012491;...  % Right Leg
                     0; 0.00325683448936741; -0.311486990906165; 0.796421295515307; -0.484850796032492; -0.0354911450764397;...  % Left Leg
                     0.0349065850398866; 0;...                                                                                   % Waist
                     -0.261799387799149; -0.167017153300893; 0; -0.734875474523928; 0; 0;...                                     % Right Arm
                     -0.261799387799149; 0.167017153300893; 0;  -0.734875474523928; 0; 0];                                       % Left Arm ;

switch new_val
  case 'Default'

   Input_data.q0 = q0;

  case 'File' % This read the last row
%     [FileCSV PathCSV] = uigetfile({'*.csv'}, 'Choose initial csv file (values of the motors)');
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
        %         motors = csvread(FileCSV,size(load(FileCSV),1) - 1);
            configuration = csvread(FileCSV, size(load(FileCSV), 1) - 1);

            if length(configuration) ~= 26
              CSVerror = 1;
              errorText = 'Wrong size of the csv file loaded! Using default configuration';
            elseif length(configuration) == 26
              CSVerror = 0;
              Input_data.q0 = reshape(configuration,26,1);
            end
        end

      catch
        CSVerror = 1;
        errorText = 'Error loading csv file, please check your data! Using default configuration';
      end

    end
      
    if CSVerror
      h = msgbox(errorText, 'Error','error');
      Input_data.q0 = q0;
    end
    
end


guidata(hObject,handles)
