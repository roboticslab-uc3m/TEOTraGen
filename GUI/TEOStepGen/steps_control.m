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

% Last Modified by GUIDE v2.5 15-Dec-2014 20:39:30



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


function steps_control_OpeningFcn(hObject, eventdata, handles, varargin)
  % Choose default command line output for steps_control
  handles.output = hObject;

  % Put window correctly
  movegui(gcf,'center')

  % Default variables
  handles.uipanelInitialPosition     = 'Default';
  handles.uipanelLegSupport          = 'Right Leg';

  
  % Get Data
  handles.GUIConfig = TEOStepGenConfig();
  
  % Check Values of the Configuration File
  if isfield(handles.GUIConfig, 'q0'),
    handles.Input_data.q0 = handles.GUIConfig.q0;
  else
    % -0.01 M
    %   handles.Input_data.q0 = [ 0; 0; -0.1716; 0.3605; -0.1889; 0; ... % Right Leg
    %                     0; 0; -0.1716; 0.3605; -0.1889; 0; ... % Left Leg                   
    %                     0; 0;...                                                                                                                  % Waist
    %                     0.420000000000000; -0.167017153300893; 0; -1.250000000000000; 0; 0; ...                                                   % Right Arm
    %                     0.420000000000000; 0.167017153300893; 0; -1.250000000000000; 0; 0];                                                       % Left Arm ;
    % -0.025 M   
    handles.Input_data.q0 = [  0; 0; -0.2417; 0.5081; -0.2664; 0; ...                                  % Right Leg
                               0; 0; -0.2417; 0.5081; -0.2664; 0; ...                                  % Left Leg 
                               0; 0; ...                                                               % Waist
                               0.420000000000000; -0.167017153300893; 0; -1.250000000000000; 0; 0; ... % Right Arm
                               0.420000000000000; 0.167017153300893; 0; -1.250000000000000; 0; 0];     % Left Arm
  end
  
  if isfield(handles.GUIConfig, 'alpha_ds'),
    handles.Input_data.alpha_ds = handles.GUIConfig.alpha_ds;
  else
    handles.Input_data.alpha_ds = 0.5;
  end
  
  if isfield(handles.GUIConfig, 'gamma_com'),
    handles.Input_data.gamma_com = handles.GUIConfig.gamma_com;
  else
    handles.Input_data.gamma_com = 0.4;
  end
  
  if isfield(handles.GUIConfig, 'L_val'),
    handles.Input_data.L_val = handles.GUIConfig.L_val;
  else
    handles.Input_data.L_val	= 0.1;
  end
  
  if isfield(handles.GUIConfig, 'H_val'),
    handles.Input_data.H_val = handles.GUIConfig.H_val;
  else
    handles.Input_data.H_val = 0.01;
  end
  
  if isfield(handles.GUIConfig, 'Ts'),
    handles.Input_data.Ts_val = handles.GUIConfig.Ts;
  else
    handles.Input_data.Ts_val = 0.01;
  end
  
  if isfield(handles.GUIConfig, 'TStep'),
    handles.Input_data.T_val = handles.GUIConfig.TStep;
  else
    handles.Input_data.T_val = 5;
  end
  
  if isfield(handles.GUIConfig, 'InitialSupportLeg'),
    handles.Input_data.SupportLeg = handles.GUIConfig.InitialSupportLeg;
  else
    handles.Input_data.SupportLeg = 'Right';
  end

  if isfield(handles.GUIConfig, 'kp'),
    handles.parameters.kp = handles.GUIConfig.kp;
  else
    handles.parameters.kp = 0.01;
  end

  if isfield(handles.GUIConfig, 'ko'),
    handles.parameters.ko = handles.GUIConfig.ko;
  else
    handles.parameters.ko = pi/8;
  end
  
  set(handles.editAlphaDS,'String',num2str(handles.Input_data.alpha_ds));
  set(handles.editGammaCOM,'String',num2str(handles.Input_data.gamma_com));
  set(handles.editStepLength,'String',num2str(handles.Input_data.L_val));
  set(handles.editStepHeight,'String',num2str(handles.Input_data.H_val));
  set(handles.editStepTimeT,'String',num2str(handles.Input_data.T_val));
  set(handles.editStepTimeTs,'String',num2str(handles.Input_data.Ts_val));

  
  
  % Update handles structure
  guidata(hObject, handles);

                   
  % Show UC3M logo
  axes(handles.UC3M_logo)
  [r,map] = imread('uc3m.png','png');
  image(r); colormap(map); axis off
  
% Update handles structure
guidata(hObject, handles);
  
% UIWAIT makes steps_control wait for user response (see UIRESUME)
% uiwait(handles.figureStepsControl);


function varargout = steps_control_OutputFcn(hObject, eventdata, handles) 
  % Get default command line output from handles structure
  varargout{1} = handles.output;

function radiobuttonInitialStepDefault_Callback(hObject, eventdata, handles)

function radiobuttonInitialStepFile_Callback(hObject, eventdata, handles)

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



function editStepHeight_Callback(hObject, eventdata, handles)
% hObject    handle to editStepHeight (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editStepHeight as text
%        str2double(get(hObject,'String')) returns contents of editStepHeight as a double


% --- Executes during object creation, after setting all properties.
function editStepHeight_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editStepHeight (see GCBO)
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
handles.Input_data.SupportLeg = handles.uipanelLegSupport;
handles.Input_data.L_val = str2double(get(handles.editStepLength,'String'));
handles.Input_data.H_val = str2double(get(handles.editStepHeight,'String'));
handles.Input_data.T_val = str2double(get(handles.editStepTimeT,'String'));
handles.Input_data.Ts_val = str2double(get(handles.editStepTimeTs,'String'));
handles.Input_data.alpha_ds = str2double(get(handles.editAlphaDS,'String')); % Percentage time for double support phases (1 and 2)
handles.Input_data.gamma_com = str2double(get(handles.editGammaCOM,'String')); % Percentage of movement in double support

% Update handles structure
guidata(hObject, handles);

design_step(handles.Input_data);
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


function uipanelInitialPosition_SelectionChangeFcn(hObject, eventdata, handles)
% --- Executes when selected object is changed in uipanelInitialPosition.

  new_val = get(eventdata.NewValue,'String');

  CSVerror = 0;

  switch new_val
    case 'Default'
     handles.Input_data.q0 = handles.GUIConfig.q0;

    case 'File' % This read the last row
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
                handles.Input_data.q0 = reshape(configuration,26,1);
              end
          end

        catch
          CSVerror = 1;
          errorText = 'Error loading csv file, please check your data! Using default configuration';
        end

      end

      if CSVerror
        msgbox(errorText, 'Error','error');
        handles.Input_data.q0 = handles.GUIConfig.q0;
      end

  end

% Update handles structure
guidata(hObject, handles);



function editAlphaDS_Callback(hObject, eventdata, handles)


function editAlphaDS_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function editGammaCOM_Callback(hObject, eventdata, handles)


function editGammaCOM_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
