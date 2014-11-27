function varargout = settings_trajectory_generation(varargin)
% ***************************
% SETTINGS_TRAJECTORY_GENERATION MATLAB code for settings_trajectory_generation.fig
% This window generates the main functions about Trajectory Generation
% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @settings_trajectory_generation_OpeningFcn, ...
                   'gui_OutputFcn',  @settings_trajectory_generation_OutputFcn, ...
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


% --- Executes just before settings_trajectory_generation is made visible.
function settings_trajectory_generation_OpeningFcn(hObject, eventdata, handles, varargin)
% -----------------
% Initial function
% -----------------

handles.output = hObject;

% Window Position
scrsz = get(0,'ScreenSize');
pos_act = get(gcf,'Position');
xr = scrsz(3) - pos_act(3);
xp = round(xr/2);
yr = scrsz(4) - pos_act(4);
yp = round(yr/2);
set(gcf,'Position',[xp yp pos_act(3) pos_act(4)]);

% UC3M Logo
axes(handles.axes1)
[r,map] = imread('uc3m.png','png');
image(r); colormap(map); axis off;

% Window Layout


% Default trajectory parameters
handles.trajectory_settings.units.pos = 'm';
handles.trajectory_settings.units.orient = 'rad';
handles.trajectory_settings.units.def = 'Relative';


% handles.default.q0 = [0; 0.00325683448936741; -0.308647699300050; 0.796421295515307; -0.487773596215257; 0.0278918646012491;...  % Right Leg
%                      0; 0.00325683448936741; -0.311486990906165; 0.796421295515307; -0.484850796032492; -0.0354911450764397;...  % Left Leg
%                      0.0349065850398866; 0;...                                                                                   % Waist
%                      -0.261799387799149; -0.167017153300893; 0; -0.734875474523928; 0; 0;...                                     % Right Arm
%                      -0.261799387799149; 0.167017153300893; 0;  -0.734875474523928; 0; 0];                                       % Left Arm ;
                   

% handles.default.q0 = [  0.000000002196439; 0.010832397471816; -0.551079679399186; 1.164867614763564; -0.613787939728849; -0.010832398780057; ...  % Right Leg
%                         0.000000000586019; -0.010832395710800; -0.551079681180760; 1.164867615158001; -0.613787930690068; 0.010832389542969; ...  % Left Leg


handles.default.q0 = [  0; 0; -0.2417; 0.5081; -0.2664; 0; ... % Right Leg
                                 0; 0; -0.2417; 0.5081; -0.2664; 0; ... % Left Leg 
                                 0; 0;...                                                                                                                  % Waist
                                 0.420000000000000; -0.167017153300893; 0; -1.250000000000000; 0; 0; ...                                                   % Right Arm
                                 0.420000000000000; 0.167017153300893; 0; -1.250000000000000; 0; 0];   
% handles.default.q0 = [ 0; 0; -0.1716; 0.3605; -0.1889; 0; ... % Right Leg
%                        0; 0; -0.1716; 0.3605; -0.1889; 0; ... % Left Leg                   
%                        0; 0;...                                                                                                                  % Waist
%                        0.420000000000000; -0.167017153300893; 0; -1.250000000000000; 0; 0; ...                                                   % Right Arm
%                        0.420000000000000; 0.167017153300893; 0; -1.250000000000000; 0; 0];                                                       % Left Arm ;
                     
handles.default.Ts = 0.02;
handles.default.kp = 0.01;
handles.default.ko = pi/8;

handles.trajectory_settings.parameters.Ts = handles.default.Ts;
handles.trajectory_settings.parameters.kp = handles.default.kp;
handles.trajectory_settings.parameters.ko = handles.default.ko;

handles.trajectory_settings.save.fdat = 1;
handles.trajectory_settings.save.fcsv = 1;

handles.trajectory_settings.q0.from = '';
handles.trajectory_settings.q0.data = handles.default.q0;

handles.trajectory_settings.body_parts.RH = 1;
handles.trajectory_settings.body_parts.LH = 1;
handles.trajectory_settings.body_parts.RF = 1;
handles.trajectory_settings.body_parts.LF = 1;
handles.trajectory_settings.body_parts.CoM = 1;
handles.trajectory_settings.body_parts.Waist = 1;

% Update handles structure
guidata(hObject, handles);


function varargout = settings_trajectory_generation_OutputFcn(hObject, eventdata, handles) 

varargout{1} = handles.output;

% ------------------
% "Continue" Button
% ------------------
function push_continue_Callback(hObject, eventdata, handles)

global trajectory d_trajectory dd_trajectory Itrajectory

% Get TimeStep and Gains
handles.trajectory_settings.parameters.Ts = str2num(get(handles.Ts_edit,'String'));
handles.trajectory_settings.parameters.kp = str2num(get(handles.kp_edit,'String'));
handles.trajectory_settings.parameters.ko = str2num(get(handles.ko_edit,'String'));

% Check TimeStep and Gains values
if isempty(handles.trajectory_settings.parameters.Ts)
  h = msgbox('Error loading Ts parameter. Using default value', 'Error','error');
  handles.trajectory_settings.parameters.Ts = handles.default.Ts;
  set(handles.Ts_edit,'String',num2str(handles.trajectory_settings.parameters.Ts));
  waitfor(h);
end

if isempty(handles.trajectory_settings.parameters.kp)
  h = msgbox('Error loading ko parameter. Using default value', 'Error','error');
  handles.trajectory_settings.parameters.kp = handles.default.kp;
  set(handles.kp_edit,'String',num2str(handles.trajectory_settings.parameters.kp));
  waitfor(h);
end

if isempty(handles.trajectory_settings.parameters.ko)
  h = msgbox('Error loading kp parameter. Using default value', 'Error','error');
  handles.trajectory_settings.parameters.ko = handles.default.ko;
  set(handles.ko_edit,'String',num2str(handles.trajectory_settings.parameters.ko));
  waitfor(h);
end

% Assign TEO kinematics library, TEO's structure with the selected units
% and TEO's humanoid fields
handles.trajectory_settings.h = TEO_kinematics_library;
if exist('TEO', 'var') == 0
    handles.trajectory_settings.TEO = TEO_structure('numeric', handles.trajectory_settings.units.orient, handles.trajectory_settings.units.pos);
end

waitbarOpening = waitbar(0,'Please wait...');

handles.trajectory_settings.humanoid_fields = humanoid_operational_fields (); 

% Create trajectories templates
trajectory = create_trajectory_template (handles.trajectory_settings.humanoid_fields, (handles.trajectory_settings.parameters.Ts));
d_trajectory = create_trajectory_template (handles.trajectory_settings.humanoid_fields, (handles.trajectory_settings.parameters.Ts));
dd_trajectory = create_trajectory_template (handles.trajectory_settings.humanoid_fields, (handles.trajectory_settings.parameters.Ts));

% Create global trajectories templates
Itrajectory = trajectory;
d_Itrajectory = d_trajectory;
dd_Itrajectory = dd_trajectory;

waitbar(20);

q0 = handles.trajectory_settings.q0.data;
Ts = handles.trajectory_settings.parameters.Ts;


% Assign initial poses expressed in Inertial Frame and Local Frames
[trajectory, Itrajectory] = assign_operational_trajectory(trajectory, q0, Ts, handles.trajectory_settings.TEO, handles.trajectory_settings.humanoid_fields, handles.trajectory_settings.h, handles.trajectory_settings.units, Itrajectory, 1);

% 
% if strcmp(handles.trajectory_settings.units.pos,'m') && strcmp(handles.trajectory_settings.units.orient,'rad')
%     pose_CoM = pose_tr2rpy(pose_quat2tr(pose_rpy2quat(pose_RF))*pose_quat2tr(handles.trajectory_settings.h.RF_T_CoM(q0)));  % RF is the standing leg
%     pose_RH = pose_tr2rpy(pose_quat2tr(pose_rpy2quat(pose_CoM))*pose_quat2tr(handles.trajectory_settings.h.CoM_T_RH(q0)));
%     pose_LH = pose_tr2rpy(pose_quat2tr(pose_rpy2quat(pose_CoM))*pose_quat2tr(handles.trajectory_settings.h.CoM_T_LH(q0)));
% elseif strcmp(handles.trajectory_settings.units.pos,'mm') && strcmp(handles.trajectory_settings.units.orient,'rad')
%     temp=handles.trajectory_settings.h.RF_T_CoM(q0);
%     pose_CoM = pose_tr2rpy(pose_quat2tr(pose_rpy2quat(pose_RF))*pose_quat2tr([temp(1:3)*1000; temp(4:7)]));  % RF is the standing leg
%     temp=handles.trajectory_settings.h.CoM_T_RH(q0);
%     pose_RH = pose_tr2rpy(pose_quat2tr(pose_rpy2quat(pose_CoM))*pose_quat2tr([temp(1:3)*1000; temp(4:7)]));
%     temp=handles.trajectory_settings.h.CoM_T_LH(q0);
%     pose_LH = pose_tr2rpy(pose_quat2tr(pose_rpy2quat(pose_CoM))*pose_quat2tr([temp(1:3)*1000; temp(4:7)]));
% elseif strcmp(handles.trajectory_settings.units.pos,'mm') && strcmp(handles.trajectory_settings.units.orient,'degrees')
%     temp=pose_quat2rpy(handles.trajectory_settings.h.RF_T_CoM(q0));
%     pose_CoM = pose_tr2rpy(pose_quat2tr(pose_rpy2quat(pose_RF))*pose_rpy2tr([temp(1:3)*1000; temp(4:6)*180/pi]));  % RF is the standing leg
%     temp=pose_quat2rpy(handles.trajectory_settings.h.CoM_T_RH(q0));
%     pose_RH = pose_tr2rpy(pose_quat2tr(pose_rpy2quat(pose_CoM))*pose_rpy2tr([temp(1:3)*1000; temp(4:6)*180/pi]));
%     temp=pose_quat2rpy(handles.trajectory_settings.h.CoM_T_LH(q0));
%     pose_LH = pose_tr2rpy(pose_quat2tr(pose_rpy2quat(pose_CoM))*pose_rpy2tr([temp(1:3)*1000; temp(4:6)*180/pi]));   
% elseif strcmp(handles.trajectory_settings.units.pos,'m') && strcmp(handles.trajectory_settings.units.orient,'degrees')
%     temp=pose_quat2rpy(handles.trajectory_settings.h.RF_T_CoM(q0));
%     pose_CoM = pose_tr2rpy(pose_quat2tr(pose_rpy2quat(pose_RF))*pose_rpy2tr([temp(1:3); temp(4:6)*180/pi]));  % RF is the standing leg
%     temp=pose_quat2rpy(handles.trajectory_settings.h.CoM_T_RH(q0));
%     pose_RH = pose_tr2rpy(pose_quat2tr(pose_rpy2quat(pose_CoM))*pose_rpy2tr([temp(1:3); temp(4:6)*180/pi]));
%     temp=pose_quat2rpy(handles.trajectory_settings.h.CoM_T_LH(q0));
%     pose_LH = pose_tr2rpy(pose_quat2tr(pose_rpy2quat(pose_CoM))*pose_rpy2tr([temp(1:3); temp(4:6)*180/pi]));  
% else
%     disp('ERROR: No m/mm or rad/deg value')
% end
% 
% 
% trajectory = insert_trajectory(trajectory, handles.trajectory_settings.humanoid_fields, create_trajectory_structure(pose_RF, Ts, 0), 'RF');
% trajectory = insert_trajectory(trajectory, handles.trajectory_settings.humanoid_fields, create_trajectory_structure(pose_LF, Ts, 0), 'LF');
% trajectory = insert_trajectory(trajectory, handles.trajectory_settings.humanoid_fields, create_trajectory_structure(pose_CoM, Ts, 0), 'CoM');
% trajectory = insert_trajectory(trajectory, handles.trajectory_settings.humanoid_fields, create_trajectory_structure(pose_RH, Ts, 0), 'RH');
% trajectory = insert_trajectory(trajectory, handles.trajectory_settings.humanoid_fields, create_trajectory_structure(pose_LH, Ts, 0), 'LH');
% 

trajectories.trajectory = trajectory;
trajectories.d_trajectory = d_trajectory;
trajectories.dd_trajectory = dd_trajectory;
Inertialtrajectories.Itrajectory = Itrajectory;
Inertialtrajectories.d_Itrajectory = d_Itrajectory;
Inertialtrajectories.dd_Itrajectory = dd_Itrajectory;

handles.trajectory_settings.initial = trajectories;
handles.trajectory_settings.Iinitial = Inertialtrajectories;

% Save gui parameters
guidata(hObject,handles)

% Open trajectory_generation window
waitbar(0.35);
trajectory_generation(handles.trajectory_settings, trajectories, Inertialtrajectories, waitbarOpening);
close(handles.figure1)


% ********** OBJECTS CALLBACKS **********

% Units changes
function position_unit_SelectionChangeFcn(hObject, eventdata, handles)
  option = get(eventdata.NewValue, 'String');
  if strcmp(option,'[mm] milimeters')
    handles.trajectory_settings.units.pos = 'mm';
  else
    handles.trajectory_settings.units.pos = 'm';
  end
  guidata(hObject,handles)

  
function orientation_unit_SelectionChangeFcn(hObject, eventdata, handles)
  option = get(eventdata.NewValue, 'String');
  if strcmp(option,'[deg] degrees')
  	handles.trajectory_settings.units.orient = 'degrees';
  else
    handles.trajectory_settings.units.orient = 'rad';
  end
  guidata(hObject,handles)

  
function abs_diff_SelectionChangeFcn(hObject, eventdata, handles)
  handles.trajectory_settings.units.def = get(eventdata.NewValue,'String');
  guidata(hObject,handles)


function initial_data_panel_SelectionChangeFcn(hObject, eventdata, handles)
  ini_val = get(eventdata.NewValue,'String');
  CSVerror = 0;
  switch ini_val
    case 'default position'
      handles.trajectory_settings.q0.data = handles.default.q0;

    case 'from file'
%       [FileCSV PathCSV] = uigetfile({'*.csv'}, 'Choose initial csv file (values of the motors)');
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
                  handles.trajectory_settings.q0.data = reshape(configuration,26,1);
                end
                %[data(1:6);data(11:16);data(21);data(7:10);data(23);data(17:20);data(23)];
            end

          catch
            CSVerror = 1;
            errorText = 'Error loading csv file, please check your data! Using default configuration';
          end
          
        end
      
      if CSVerror
        h = msgbox(errorText, 'Error','error');
        handles.trajectory_settings.q0.data = handles.default.q0;
      end
        
  end
  guidata(hObject,handles)



% *************************** MENUS ******************************
% --------------------------------------------------------------------
function main_menu_of_settings_Callback(hObject, eventdata, handles)
  TEOTraGen
  close(handles.figure1)


% ------------ CHECKBOX -------------------

function check_filetxt_Callback(hObject, eventdata, handles)
  if (get(hObject,'Value') == get(hObject,'Max'))
    handles.trajectory_settings.save.fdat = 1;
  else
    handles.trajectory_settings.save.fdat = 0;
  end
guidata(hObject,handles)

  
function check_filecsv_Callback(hObject, eventdata, handles)
  if (get(hObject,'Value') == get(hObject,'Max'))
  	handles.trajectory_settings.save.fcsv = 1;
  else
    handles.trajectory_settings.save.fcsv = 0;
  end
guidata(hObject,handles)

  
function check_LA_Callback(hObject, eventdata, handles)
  if (get(hObject,'Value') == get(hObject,'Max'))
    handles.trajectory_settings.body_parts.LH = 1;
  else
    handles.trajectory_settings.body_parts.LH = 0;
  end
guidata(hObject,handles)

  
function check_LL_Callback(hObject, eventdata, handles)
  if (get(hObject,'Value') == get(hObject,'Max'))
    handles.trajectory_settings.body_parts.LF = 1;
  else
  	handles.trajectory_settings.body_parts.LF = 0;
  end
guidata(hObject,handles)

  
function check_RA_Callback(hObject, eventdata, handles)
  if (get(hObject,'Value') == get(hObject,'Max'))
      handles.trajectory_settings.body_parts.RH = 1;
  else
      handles.trajectory_settings.body_parts.RH = 0;
  end
guidata(hObject,handles)

  
function check_RL_Callback(hObject, eventdata, handles)
  if (get(hObject,'Value') == get(hObject,'Max'))
      handles.trajectory_settings.body_parts.RF = 1;
  else
      handles.trajectory_settings.body_parts.RF = 0;
  end
guidata(hObject,handles)

function check_CoM_Callback(hObject, eventdata, handles)
  if (get(hObject,'Value') == get(hObject,'Max'))
      handles.trajectory_settings.body_parts.CoM = 1;
  else
      handles.trajectory_settings.body_parts.CoM = 0;
  end
guidata(hObject,handles)


function check_Waist_Callback(hObject, eventdata, handles)
  if (get(hObject,'Value') == get(hObject,'Max'))
      handles.trajectory_settings.body_parts.Waist = 1;
  else
      handles.trajectory_settings.body_parts.Waist = 0;
  end
guidata(hObject,handles)
  
  
  
% ----------------------- Text & Edit ------------------------------------

function Ts_edit_Callback(hObject, eventdata, handles)

% --- Executes during object creation, after setting all properties.
function Ts_edit_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function kp_edit_Callback(hObject, eventdata, handles)


function kp_edit_CreateFcn(hObject, eventdata, handles)

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function ko_edit_Callback(hObject, eventdata, handles)


function ko_edit_CreateFcn(hObject, eventdata, handles)
  if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
      set(hObject,'BackgroundColor','white');
  end


function radiobutton1_ButtonDownFcn(hObject, eventdata, handles)


function ko_text_CreateFcn(hObject, eventdata, handles)


% --------------------------------------------------------------------
function back_Callback(hObject, eventdata, handles)
% hObject    handle to back (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


function menu_back_teowholegen_Callback(hObject, eventdata, handles)
  TEOWholeGen
  close(handles.figure1)
