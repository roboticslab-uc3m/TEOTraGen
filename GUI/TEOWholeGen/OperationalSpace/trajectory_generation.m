function varargout = trajectory_generation(varargin)
% TRAJECTORY_GENERATION MATLAB code for trajectory_generation.fig
%      TRAJECTORY_GENERATION, by itself, creates a new TRAJECTORY_GENERATION or raises the existing
%      singleton*.
%
%      H = TRAJECTORY_GENERATION returns the handle to a new TRAJECTORY_GENERATION or the handle to
%      the existing singleton*.
%
%      TRAJECTORY_GENERATION('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in TRAJECTORY_GENERATION.M with the given input arguments.
%
%      TRAJECTORY_GENERATION('Property','Value',...) creates a new TRAJECTORY_GENERATION or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before trajectory_generation_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to trajectory_generation_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help trajectory_generation

% Last Modified by GUIDE v2.5 09-Sep-2014 18:23:58

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @trajectory_generation_OpeningFcn, ...
                   'gui_OutputFcn',  @trajectory_generation_OutputFcn, ...
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


% --- Executes just before trajectory_generation is made visible.
function trajectory_generation_OpeningFcn(hObject, eventdata, handles, varargin)
  
clear trajectory d_trajectory dd_trajectory q dq ddq

% Choose default command line output for trajectory_generation
handles.output = hObject;
global data trajectories

% Put the window to the center of the screen
scrsz = get(0,'ScreenSize');
pos_act = get(gcf,'Position');
xr = scrsz(3) - pos_act(3);
xp = round(xr/2);
yr = scrsz(4) - pos_act(4);
yp = round(yr/2);
set(gcf,'Position',[xp yp pos_act(3) pos_act(4)]);

% Evaluate Input Data
if isempty(varargin)
  warning('WarnTEOTraGen:noInputGUI', 'No inputs for trajectory_generation. Opening settings_trajectory_generation window');
  close(gcf)
  settings_trajectory_generation;
  return;
else
  data.trajectory_settings = varargin{1};
  trajectories.operational = varargin{2};
  trajectories.Inertialtrajectories = varargin{3};
  waitbarOpening = varargin{4};
  waitbar(0.4);
end


% Default layout
set([handles.data_panel handles.operational_panel], 'Visible','off');
set([handles.save_operational_button handles.gen_joint_button ],'Enable','off');
set([handles.diff_pos_button handles.abs_pos_button handles.diff_orient_button handles.abs_orient_button handles.diff_time_button handles.abs_time_button],'Value',0);
set(handles.Ts_val,'String', data.trajectory_settings.parameters.Ts);

% Enable/Disable generation for corresponding humanoid part
if data.trajectory_settings.body_parts.CoM == 0,
  set(handles.CoM_button,'Enable','off');
  disp('Disable CoM generation');
end
if data.trajectory_settings.body_parts.RF == 0,
  set(handles.RF_button,'Enable','off');
  disp('Disable RF generation');
end
if data.trajectory_settings.body_parts.LF == 0,
  set(handles.LF_button,'Enable','off');
  disp('Disable LF generation');
end
if data.trajectory_settings.body_parts.RH == 0,
  set(handles.RH_button,'Enable','off');
  disp('Disable RH generation');
end
if data.trajectory_settings.body_parts.LH == 0,
  set(handles.LH_button,'Enable','off');
  disp('Disable LH generation');
end
waitbar(0.5);

set(handles.Waist_button,'Enable','off'); % Disable for now...
% if data.trajectory_settings.body_parts.Waist == 0
%     set(handles.Waist_button,'Enable','off');
%     disp('Disable Waist generation');
% end


% Enable default GUI objects
set ([handles.diff_pos_button, handles.diff_orient_button, handles.diff_time_button handles.support_popup], 'Value', 1.0)
set(handles.CoM_button, 'Value', 0)  % Avoid CoM_button be pressed

% At least one one point in total_points
data.trajectory_settings.total_points = 1;
set (handles.num_points_text, 'String', num2str(data.trajectory_settings.total_points))

% Variables for manipulate stretchs
data.stretchs.trajectory = {}; 
data.stretchs.number = get(handles.stretch_trajectory_popup, 'Value'); % Number of current trajectory
data.stretchs.trajectory{1} = (get(handles.stretch_trajectory_popup, 'String')); % Text of current trajectory

% humanoid_part = 'CoM';
% globdata.q0 = data.trajectory_settings.q0.data;
% figure(waitbarOpening);
waitbar(0.7);

data.trajectory_points.input_type = 'Poses';
% humanoid_part_to_plot = 'RH';

% Create initial values
for jj = 1:(length(data.trajectory_settings.humanoid_fields)-1)
  data.trajectory_points.initial_point.(data.trajectory_settings.humanoid_fields(jj).name) = zeros(data.trajectory_settings.humanoid_fields(jj).size, 1);
  data.trajectory_points.initial_velocity.(data.trajectory_settings.humanoid_fields(jj).name) = zeros(data.trajectory_settings.humanoid_fields(jj).size, 1);
  data.trajectory_points.initial_acceleration.(data.trajectory_settings.humanoid_fields(jj).name) = zeros(data.trajectory_settings.humanoid_fields(jj).size, 1);
  data.trajectory_points.final_point.(data.trajectory_settings.humanoid_fields(jj).name) = zeros(data.trajectory_settings.humanoid_fields(jj).size, 1);
  data.trajectory_points.final_velocity.(data.trajectory_settings.humanoid_fields(jj).name) = zeros(data.trajectory_settings.humanoid_fields(jj).size, 1);
  data.trajectory_points.final_acceleration.(data.trajectory_settings.humanoid_fields(jj).name) = zeros(data.trajectory_settings.humanoid_fields(jj).size, 1);  
  data.trajectory_points.t0_val.(data.trajectory_settings.humanoid_fields(jj).name) = 0;
  data.trajectory_points.T_val.(data.trajectory_settings.humanoid_fields(jj).name) = 0;
  data.trajectory_points.Ts_val.(data.trajectory_settings.humanoid_fields(jj).name) = data.trajectory_settings.parameters.Ts;
  data.trajectory_points.pos_diff.(data.trajectory_settings.humanoid_fields(jj).name){1} = 'Diff';
  data.trajectory_points.orient_diff.(data.trajectory_settings.humanoid_fields(jj).name){1} = 'Diff';
  data.trajectory_points.time_diff.(data.trajectory_settings.humanoid_fields(jj).name){1} = 'Diff';
  data.trajectory_points.interpola_pos.(data.trajectory_settings.humanoid_fields(jj).name){1} = 'Linear';
  data.trajectory_points.interpola_orient.(data.trajectory_settings.humanoid_fields(jj).name){1} = 'Linear';
  data.trajectory_points.support_foot.(data.trajectory_settings.humanoid_fields(jj).name) = 0; % Double support
end

% figure(waitbarOpening);
waitbar(0.9);

% UC3M logo
	axes(handles.axesUC3M)
  [r,map] = imread('uc3m.png','png');
  image(r); colormap(map); axis off

% Update handles structure   
guidata(hObject, handles);
waitbar(1);
close(waitbarOpening)
% UIWAIT makes trajectory_generation wait for user response (see UIRESUME)
% uiwait(handles.trajectory_generation);


% --- Outputs from this function are returned to the command line.
function varargout = trajectory_generation_OutputFcn(hObject, eventdata, handles) 

% Get default command line output from handles structure
varargout{1} = handles;


function support_popup_Callback(hObject, eventdata, handles)
global data
if isfield(data.trajectory_points, 'current_humanoid_part')
  update_support_foot(handles);
  messages_master(hObject, handles);
end
guidata(hObject,handles)


function support_popup_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
	set(hObject,'BackgroundColor','white');
end


function t0_text_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
	set(hObject,'BackgroundColor','white');
end


function T_text_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
	set(hObject,'BackgroundColor','white');
end


function t0_val_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
	set(hObject,'BackgroundColor','white');
end


function T_val_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
	set(hObject,'BackgroundColor','white');
end


function Ts_val_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
	set(hObject,'BackgroundColor','white');
end


function interpola_pos_popup_Callback(hObject, eventdata, handles)
global data
contents = cellstr(get(hObject,'String'));
data.trajectory_points.interpola_pos.(data.trajectory_points.current_humanoid_part){data.stretchs.number} = contents{get(hObject,'Value')};
if contents{get(hObject,'Value')} == 2
  set(handles.label_points,'String','No Circular interpolation implemented yet!');
  set(handles.label_points,'BackGroundColor',[1 0.2 0.2]);
else
  set(handles.label_points,'String','');
  set(handles.label_points,'BackGroundColor',[0.702 0.78 1]);
end


function interpola_pos_popup_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
  set(hObject,'BackgroundColor','white');
end

function xf_val_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
  set(hObject,'BackgroundColor','white');
end


function yf_val_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
  set(hObject,'BackgroundColor','white');
end


function zf_val_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
  set(hObject,'BackgroundColor','white');
end


function xi_val_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
  set(hObject,'BackgroundColor','white');
end


function yi_val_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
  set(hObject,'BackgroundColor','white');
end


function zi_val_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
  set(hObject,'BackgroundColor','white');
end


function interpola_orient_popup_Callback(hObject, eventdata, handles)
global data
contents = cellstr(get(hObject,'String'));
data.trajectory_points.interpola_orient.(data.trajectory_points.current_humanoid_part){data.stretchs.number} = contents{get(hObject,'Value')};


function interpola_orient_popup_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
  set(hObject,'BackgroundColor','white');
end


function yawf_val_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
  set(hObject,'BackgroundColor','white');
end


function pitchf_val_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
  set(hObject,'BackgroundColor','white');
end


function rollf_val_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
  set(hObject,'BackgroundColor','white');
end


function rolli_val_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function pitchi_val_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function yawi_val_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function gen_operational_button_Callback(hObject, eventdata, handles)
global data;
global trajectories;

% Show message
set(handles.save_operational_button,'ForegroundColor','black');
set(handles.label_points,'String','Calculating Operational Trajectory. Please wait...');
set(handles.label_points,'BackGroundColor',[1 1 0.2]);
pause(0);

% Update data
update_points(data.trajectory_points.current_humanoid_part, handles);
update_support_foot(handles);
data.trajectory_settings.total_points = str2double(get(handles.num_points_text,'String'));

guidata(hObject,handles)

% Check T_values before generate trajectory
if check_T_values(handles) == 1,
  % Compute operational trajectories
  [trajectories.operational.trajectory trajectories.operational.d_trajectory trajectories.operational.dd_trajectory] = generate_trajectory(trajectories.operational.trajectory, trajectories.operational.d_trajectory, trajectories.operational.dd_trajectory, data.trajectory_points, data.trajectory_settings.total_points, data.trajectory_settings);

  set([handles.save_joint_button, handles.pushbutton_ros_visualization, handles.pushbutton_matlab_visualization],'Enable','off')
  set([handles.gen_joint_button handles.axis_movement_pushbutton handles.pushbutton_plots],'Enable','on')

  set(handles.label_points,'String','Trajectory Generation Complete!');
  set(handles.label_points,'BackGroundColor',[0.702 0.78 1]);
  
  % Clear trajectories in Joint Space
  trajectories.joints = [];
  
end

guidata(hObject,handles)


function save_operational_button_Callback(hObject, eventdata, handles)
global data trajectories
traj_name_string = get(handles.traj_name_text,'String');
if isempty(traj_name_string),
  warndlg('ERROR: Trajectory name field is empty. You need to define a name for the trajectory','Trajectory Name Error');
  return;
end
save_trajectory = trajectories.operational.trajectory.(data.trajectory_points.current_humanoid_part);
save_d_trajectory = trajectories.operational.d_trajectory.(data.trajectory_points.current_humanoid_part);
save_dd_trajectory = trajectories.operational.dd_trajectory.(data.trajectory_points.current_humanoid_part);
save_trajectory_window (traj_name_string, save_trajectory, save_d_trajectory, save_dd_trajectory, 'operational');
guidata(hObject,handles)


function gen_joint_button_Callback(hObject, eventdata, handles)

%
set(handles.save_operational_button,'ForegroundColor','black');
set(handles.label_points,'String','Calculating Inverse kinematics. Please wait...');
set(handles.label_points,'BackGroundColor',[1 1 0.2]);

pause(0);

global data trajectories

% [q, dq, ddq] = ik_TEO2(q0, trajectory, d_trajectory, dd_trajectory, data.trajectory_settings.h, data.trajectory_settings.parameters, SETTINGS_TEO, humanoid_part);
% [q, dq, ddq] = ik_TEOPRUEBA(q0, trajectory, d_trajectory, dd_trajectory, data.trajectory_settings.h, data.trajectory_settings.parameters, SETTINGS_TEO, humanoid_part);
% [q, dq, ddq] = ik_full_body(data.trajectory_settings.q0.data, trajectories.operational.trajectory, trajectories.operational.d_trajectory, data.trajectory_settings.h, data.trajectory_settings.parameters);

[q, dq, ddq] = inverse_ds_ss_jacobian_quat(data.trajectory_settings.q0.data, trajectories.operational.trajectory, trajectories.operational.d_trajectory, data.trajectory_settings.h, data.trajectory_settings.parameters);
%[q, dq, ddq] = inverse_ds_ss_jacobian_quat_second_order(data.trajectory_settings.q0.data, trajectories.operational.trajectory, trajectories.operational.d_trajectory, trajectories.operational.dd_trajectory, data.trajectory_settings.h, data.trajectory_settings.parameters);

trajectories.joints.q = q;
trajectories.joints.dq = dq;
trajectories.joints.ddq = ddq;
set(handles.label_points,'String','Inverse kinematics Complete!');
set(handles.label_points,'BackGroundColor',[0.702 0.78 1]);
%set(handles.save_operational_button,'ForegroundColor','red');
set([handles.save_joint_button handles.pushbutton_matlab_visualization handles.pushbutton_ros_visualization],'Enable','on');


function humanoid_part_panel_SelectionChangeFcn(hObject, eventdata, handles)
global data
% guidata(hObject)
% Remember previous humanoid_part selected and the new one
new_humanoid_part = get(eventdata.NewValue,'String');
old_humanoid_part = get(eventdata.OldValue,'String');
%set(handles.data_panel,'Visible','on');

% Update previous humanoid_part value (If there was)
if ~(isempty(old_humanoid_part)),
  switch old_humanoid_part
    case 'Right Hand'
      old_humanoid_part = 'RH';
    case 'Left Hand'
      old_humanoid_part = 'LH';
    case 'Right Foot'
      old_humanoid_part = 'RF';
    case 'Left Foot'
      old_humanoid_part = 'LF';
    case 'Chest'
      old_humanoid_part = 'CoM';
  end
  % Save old_points
  update_points(old_humanoid_part, handles);  
  update_support_foot(handles);
end

% Update operational panel
initial_val_function (hObject, handles);

% save current humanoid_part selected
switch new_humanoid_part
  case 'Right Hand'
    data.trajectory_points.current_humanoid_part = 'RH';

  case 'Left Hand'
    data.trajectory_points.current_humanoid_part = 'LH'; 

  case 'Right Foot'
    data.trajectory_points.current_humanoid_part = 'RF';

  case 'Left Foot'
    data.trajectory_points.current_humanoid_part = 'LF';

  case 'Chest'
    data.trajectory_points.current_humanoid_part = 'CoM';
end

messages_master(hObject, handles);
show_selected_values(hObject, handles);


% guidata(hObject, handles);


function data_panel_SelectionChangeFcn(hObject, eventdata, handles)

global joint_val
new_val = get(eventdata.NewValue,'String');
old_val = get(eventdata.OldValue,'String');
new_part = handles.new_humanoid_part;
current = getappdata(handles.trajectory_generation,'Current');
humanoid_robot = getappdata(handles.trajectory_generation,'Robot');
current.datafrom = new_val;
current.endeffector = new_part;
switch new_val
    case 'File'
        
    case 'Default'
        current.q0 = humanoid_robot.default.q0;
        current.time.T0 = humanoid_robot.default.t0;
        current.time.Ts = humanoid_robot.default.Ts;
        current.time.T = humanoid_robot.default.T;
    case 'Joint Insert'
        joint_insert(new_part,humanoid_robot)
        uiwait
        
        current.q0 = joint_val;
        current.time.T0 = humanoid_robot.default.t0;
        current.time.Ts = humanoid_robot.default.Ts;
        current.time.T = humanoid_robot.default.T;
        
end
switch new_part
    case 'RH'
       PO = pose_quat2rpy(humanoid_robot.CoM_T_RH(current.q0)); 
    case 'LH'
       PO = pose_quat2rpy(humanoid_robot.h.CoM_T_LH(current.q0)); 
    case 'RLF'
       PO = zeros(6,1);
       %PO = pose_quat2rpy(humanoid_robot.h.CoM_T_RF(current.q0)); 
    case 'LLF'
       PO = zeros(6,1);
       %PO = pose_quat2rpy(humanoid_robot.h.CoM_T_LF(current.q0)); 
    case 'RLS'
       PO = pose_quat2rpy(humanoid_robot.h.RF_T_CoM(current.q0)); 
    case 'LLS'
       PO = pose_quat2rpy(humanoid_robot.h.LF_T_CoM(current.q0));
end
current.position.p0 = PO(1:3,1);
current.orientation.a0 = PO(4:6,1);
set(handles.operational_panel,'Visible','on');
% set(handles.save_operational_button ,'Visible','on');
set(handles.gen_joint_button ,'Visible','on');

guidata(hObject,handles)


function pos_diffabs_panel_SelectionChangeFcn(hObject, eventdata, handles)
global data
set([handles.xf_val handles.yf_val handles.zf_val],'String','0');

data.trajectory_points.pos_diff.(data.trajectory_points.current_humanoid_part){data.stretchs.number} = get(eventdata.NewValue,'String');

switch data.trajectory_points.pos_diff.(data.trajectory_points.current_humanoid_part){data.stretchs.number}
    case 'Diff'
        set(handles.xf_text ,'String','delta x');set(handles.yf_text,'String','delta y');set(handles.zf_text,'String','delta z');
    case 'Abs'
        set(handles.xf_text ,'String','x f');set(handles.yf_text,'String','y f');set(handles.zf_text,'String','z f');
end

guidata(hObject,handles)


function orient_diffabs_panel_SelectionChangeFcn(hObject, eventdata, handles)
global data
set([handles.rollf_val handles.pitchf_val handles.yawf_val],'String','0');

data.trajectory_points.orient_diff.(data.trajectory_points.current_humanoid_part){data.stretchs.number} = get(eventdata.NewValue,'String');

switch data.trajectory_points.orient_diff.(data.trajectory_points.current_humanoid_part){data.stretchs.number}
    case 'Diff'
        set(handles.rollf_text ,'String','delta roll');set(handles.pitchf_text,'String','delta pitch');set(handles.yawf_text,'String','delta yaw');
    case 'Abs'
        set(handles.rollf_text ,'String','roll f');set(handles.pitchf_text,'String','pitch f');set(handles.yawf_text,'String','yaw f');
end
handles.diffabs = get(eventdata.NewValue,'String');
guidata(hObject,handles)


function time_diffabs_panel_SelectionChangeFcn(hObject, eventdata, handles)
global data
set(handles.T_val,'String','0');

data.trajectory_points.time_diff.(data.trajectory_points.current_humanoid_part){data.stretchs.number} = get(eventdata.NewValue,'String');

switch data.trajectory_points.time_diff.(data.trajectory_points.current_humanoid_part){data.stretchs.number}
    case 'Diff'
        set(handles.T_text ,'String','Delta Time');
    case 'Abs'
        set(handles.T_text ,'String','Final Time');
end
handles.diffabs = get(eventdata.NewValue,'String');
guidata(hObject,handles)


function vel_acel_func(hOject,handles,caso,diffabs,interpola)

global dp_ini dp_end ddp_ini ddp_end
current = getappdata(handles.trajectory_generation,'Current');
switch caso
  case 1
    current.position.diffabs = diffabs;

    current.position.dp0 = dp_ini;
    current.position.ddp0 = ddp_ini;

    current.position.dpf = dp_end;
    current.position.ddpf = ddp_end;
    current.interpola.pos = interpola;

  case 2
    current.orientation.diffabs = diffabs;

    current.orientation.da0 = dp_ini;
    current.orientation.dda0 = ddp_ini;

    current.orientation.daf = dp_end;
    current.orientation.ddaf = ddp_end;
    current.interpola.orient = interpola;
end
setappdata(handles.trajectory_generation,'Current',current);


function traj_name_text_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function initial_val_function (hObject, handles)
% Disable gui objects
set(handles.traj_name_text, 'String', 'teo_movement');
set(handles.save_operational_button, 'ForegroundColor', 'black');
set([handles.default_button handles.file_button handles.insert_joint_button...
    handles.diff_pos_button handles.abs_pos_button handles.diff_orient_button...
    handles.abs_orient_button handles.diff_time_button handles.abs_time_button], 'Value', 0);
  
guidata(hObject,handles)


function main_menu_Callback(hObject, eventdata, handles)
% hObject    handle to main_menu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% cd ..
TEOTraGen
close trajectory_generation


function trajectory_settings_Callback(hObject, eventdata, handles)
% settings_trajectory_generation
close (gcf)
settings_trajectory_generation


function show_selected_values(hObject, handles)
global data

% Enable Operational Panel
set(handles.operational_panel, 'Visible', 'on');

% data.stretchs.trajectory
% data.trajectory_points.current_humanoid_part
% data.stretchs.trajectory{data.stretchs.number}
% ['Operational Space - ' '"Stretch ' data.stretchs.trajectory{data.stretchs.number} '" - ' data.trajectory_points.current_humanoid_part ' Pose']

set(handles.operational_panel, 'Title', ['Operational Space - ' '"Stretch ' data.stretchs.trajectory{data.stretchs.number} '" - ' data.trajectory_points.current_humanoid_part ' Pose']);

add_default_data(hObject, handles);
    
% Initial Point
if data.stretchs.number == 1,
%   if strcmp(data.trajectory_settings.units.def, 'Inertial'),
%     initial_trajectory = 'Iinitial';
%   elseif strcmp(data.trajectory_settings.units.def, 'Relative'),
%     initial_trajectory = 'initial';
%   else
%     error('myApp:inputChk', 'Wrong frames definition provided')
%   end
%   
%  
%   set(handles.xi_val,'String', num2str(data.trajectory_settings.(initial_trajectory).(data.trajectory_points.current_humanoid_part)(1)));
%   set(handles.yi_val,'String', num2str(data.trajectory_settings.(initial_trajectory).(data.trajectory_points.current_humanoid_part)(2)));
%   set(handles.zi_val,'String', num2str(data.trajectory_settings.(initial_trajectory).(data.trajectory_points.current_humanoid_part)(3)));
%   set(handles.rolli_val,'String', num2str(data.trajectory_settings.(initial_trajectory).(data.trajectory_points.current_humanoid_part)(4)));
%   set(handles.pitchi_val,'String', num2str(data.trajectory_settings.(initial_trajectory).(data.trajectory_points.current_humanoid_part)(5)));
%   set(handles.yawi_val,'String', num2str(data.trajectory_settings.(initial_trajectory).(data.trajectory_points.current_humanoid_part)(6)));

  set(handles.xi_val,'String', num2str(0));
  set(handles.yi_val,'String', num2str(0));
  set(handles.zi_val,'String', num2str(0));
  set(handles.rolli_val,'String', num2str(0));
  set(handles.pitchi_val,'String', num2str(0));
  set(handles.yawi_val,'String', num2str(0));

else
  if strcmp(data.trajectory_points.input_type,'Poses')
    % Show positions
    if strcmp(data.trajectory_points.pos_diff.(data.trajectory_points.current_humanoid_part){data.stretchs.number-1}, 'Diff')
      set(handles.xi_val,'String', num2str(data.trajectory_points.initial_point.(data.trajectory_points.current_humanoid_part)(1,data.stretchs.number-1) + data.trajectory_points.final_point.(data.trajectory_points.current_humanoid_part)(1,data.stretchs.number-1)));
      set(handles.yi_val,'String', num2str(data.trajectory_points.initial_point.(data.trajectory_points.current_humanoid_part)(2,data.stretchs.number-1) + data.trajectory_points.final_point.(data.trajectory_points.current_humanoid_part)(2,data.stretchs.number-1)));
      set(handles.zi_val,'String', num2str(data.trajectory_points.initial_point.(data.trajectory_points.current_humanoid_part)(3,data.stretchs.number-1) + data.trajectory_points.final_point.(data.trajectory_points.current_humanoid_part)(3,data.stretchs.number-1)));
    elseif strcmp(data.trajectory_points.pos_diff.(data.trajectory_points.current_humanoid_part){data.stretchs.number-1}, 'Abs')
      set(handles.xi_val,'String', num2str(data.trajectory_points.final_point.(data.trajectory_points.current_humanoid_part)(1,data.stretchs.number-1)));
      set(handles.yi_val,'String', num2str(data.trajectory_points.final_point.(data.trajectory_points.current_humanoid_part)(2,data.stretchs.number-1)));
      set(handles.zi_val,'String', num2str(data.trajectory_points.final_point.(data.trajectory_points.current_humanoid_part)(3,data.stretchs.number-1)));
    else
      disp('ERROR: No diff/abs position option')
    end
    % Show orientations
    if strcmp(data.trajectory_points.orient_diff.(data.trajectory_points.current_humanoid_part){data.stretchs.number-1}, 'Diff')
      set(handles.rolli_val,'String', num2str(data.trajectory_points.initial_point.(data.trajectory_points.current_humanoid_part)(4,data.stretchs.number-1)+data.trajectory_points.final_point.(data.trajectory_points.current_humanoid_part)(4,data.stretchs.number-1)));
      set(handles.pitchi_val,'String', num2str(data.trajectory_points.initial_point.(data.trajectory_points.current_humanoid_part)(5,data.stretchs.number-1)+data.trajectory_points.final_point.(data.trajectory_points.current_humanoid_part)(5,data.stretchs.number-1)));
      set(handles.yawi_val,'String', num2str(data.trajectory_points.initial_point.(data.trajectory_points.current_humanoid_part)(6,data.stretchs.number-1)+data.trajectory_points.final_point.(data.trajectory_points.current_humanoid_part)(6,data.stretchs.number-1)));
    elseif strcmp(data.trajectory_points.orient_diff.(data.trajectory_points.current_humanoid_part){data.stretchs.number-1}, 'Abs')
      set(handles.rolli_val,'String', num2str(data.trajectory_points.final_point.(data.trajectory_points.current_humanoid_part)(4,data.stretchs.number-1)));
      set(handles.pitchi_val,'String', num2str(data.trajectory_points.final_point.(data.trajectory_points.current_humanoid_part)(5,data.stretchs.number-1)));
      set(handles.yawi_val,'String', num2str(data.trajectory_points.final_point.(data.trajectory_points.current_humanoid_part)(6,data.stretchs.number-1)));
    else
      disp('ERROR: No diff/abs orientation option')
    end
  elseif strcmp(data.trajectory_points.input_type,'Velocities')
    % Show linear velocities
    if strcmp(data.trajectory_points.pos_diff.(data.trajectory_points.current_humanoid_part){data.stretchs.number-1}, 'Diff')
      set(handles.xi_val,'String', num2str(data.trajectory_points.initial_velocity.(data.trajectory_points.current_humanoid_part)(1,data.stretchs.number-1) + data.trajectory_points.final_velocity.(data.trajectory_points.current_humanoid_part)(1,data.stretchs.number-1)));
      set(handles.yi_val,'String', num2str(data.trajectory_points.initial_velocity.(data.trajectory_points.current_humanoid_part)(2,data.stretchs.number-1) + data.trajectory_points.final_velocity.(data.trajectory_points.current_humanoid_part)(2,data.stretchs.number-1)));
      set(handles.zi_val,'String', num2str(data.trajectory_points.initial_velocity.(data.trajectory_points.current_humanoid_part)(3,data.stretchs.number-1) + data.trajectory_points.final_velocity.(data.trajectory_points.current_humanoid_part)(3,data.stretchs.number-1)));
    elseif strcmp(data.trajectory_points.pos_diff.(data.trajectory_points.current_humanoid_part){data.stretchs.number-1}, 'Abs')
      set(handles.xi_val,'String', num2str(data.trajectory_points.final_velocity.(data.trajectory_points.current_humanoid_part)(1,data.stretchs.number-1)));
      set(handles.yi_val,'String', num2str(data.trajectory_points.final_velocity.(data.trajectory_points.current_humanoid_part)(2,data.stretchs.number-1)));
      set(handles.zi_val,'String', num2str(data.trajectory_points.final_velocity.(data.trajectory_points.current_humanoid_part)(3,data.stretchs.number-1)));
    else
      disp('ERROR: No diff/abs position option')
    end
    % Show angular velocities
    if strcmp(data.trajectory_points.orient_diff.(data.trajectory_points.current_humanoid_part){data.stretchs.number-1}, 'Diff')
      set(handles.rolli_val,'String', num2str(data.trajectory_points.initial_velocity.(data.trajectory_points.current_humanoid_part)(4,data.stretchs.number-1)+data.trajectory_points.final_velocity.(data.trajectory_points.current_humanoid_part)(4,data.stretchs.number-1)));
      set(handles.pitchi_val,'String', num2str(data.trajectory_points.initial_velocity.(data.trajectory_points.current_humanoid_part)(5,data.stretchs.number-1)+data.trajectory_points.final_velocity.(data.trajectory_points.current_humanoid_part)(5,data.stretchs.number-1)));
      set(handles.yawi_val,'String', num2str(data.trajectory_points.initial_velocity.(data.trajectory_points.current_humanoid_part)(6,data.stretchs.number-1)+data.trajectory_points.final_velocity.(data.trajectory_points.current_humanoid_part)(6,data.stretchs.number-1)));
    elseif strcmp(data.trajectory_points.orient_diff.(data.trajectory_points.current_humanoid_part){data.stretchs.number-1}, 'Abs')
      set(handles.rolli_val,'String', num2str(data.trajectory_points.final_velocity.(data.trajectory_points.current_humanoid_part)(4,data.stretchs.number-1)));
      set(handles.pitchi_val,'String', num2str(data.trajectory_points.final_velocity.(data.trajectory_points.current_humanoid_part)(5,data.stretchs.number-1)));
      set(handles.yawi_val,'String', num2str(data.trajectory_points.final_velocity.(data.trajectory_points.current_humanoid_part)(6,data.stretchs.number-1)));
    else
      disp('ERROR: No diff/abs orientation option')
    end
  elseif strcmp(data.trajectory_points.input_type,'Accelerations')
    % Show linear acceleration
    if strcmp(data.trajectory_points.pos_diff.(data.trajectory_points.current_humanoid_part){data.stretchs.number-1}, 'Diff')
      set(handles.xi_val,'String', num2str(data.trajectory_points.initial_acceleration.(data.trajectory_points.current_humanoid_part)(1,data.stretchs.number-1) + data.trajectory_points.final_acceleration.(data.trajectory_points.current_humanoid_part)(1,data.stretchs.number-1)));
      set(handles.yi_val,'String', num2str(data.trajectory_points.initial_acceleration.(data.trajectory_points.current_humanoid_part)(2,data.stretchs.number-1) + data.trajectory_points.final_acceleration.(data.trajectory_points.current_humanoid_part)(2,data.stretchs.number-1)));
      set(handles.zi_val,'String', num2str(data.trajectory_points.initial_acceleration.(data.trajectory_points.current_humanoid_part)(3,data.stretchs.number-1) + data.trajectory_points.final_acceleration.(data.trajectory_points.current_humanoid_part)(3,data.stretchs.number-1)));
    elseif strcmp(data.trajectory_points.pos_diff.(data.trajectory_points.current_humanoid_part){data.stretchs.number-1}, 'Abs')
      set(handles.xi_val,'String', num2str(data.trajectory_points.final_acceleration.(data.trajectory_points.current_humanoid_part)(1,data.stretchs.number-1)));
      set(handles.yi_val,'String', num2str(data.trajectory_points.final_acceleration.(data.trajectory_points.current_humanoid_part)(2,data.stretchs.number-1)));
      set(handles.zi_val,'String', num2str(data.trajectory_points.final_acceleration.(data.trajectory_points.current_humanoid_part)(3,data.stretchs.number-1)));
    else
      disp('ERROR: No diff/abs position option')
    end
    % Show angular acceleration
    if strcmp(data.trajectory_points.orient_diff.(data.trajectory_points.current_humanoid_part){data.stretchs.number-1}, 'Diff')
      set(handles.rolli_val,'String', num2str(data.trajectory_points.initial_acceleration.(data.trajectory_points.current_humanoid_part)(4,data.stretchs.number-1)+data.trajectory_points.final_acceleration.(data.trajectory_points.current_humanoid_part)(4,data.stretchs.number-1)));
      set(handles.pitchi_val,'String', num2str(data.trajectory_points.initial_acceleration.(data.trajectory_points.current_humanoid_part)(5,data.stretchs.number-1)+data.trajectory_points.final_acceleration.(data.trajectory_points.current_humanoid_part)(5,data.stretchs.number-1)));
      set(handles.yawi_val,'String', num2str(data.trajectory_points.initial_acceleration.(data.trajectory_points.current_humanoid_part)(6,data.stretchs.number-1)+data.trajectory_points.final_acceleration.(data.trajectory_points.current_humanoid_part)(6,data.stretchs.number-1)));
    elseif strcmp(data.trajectory_points.orient_diff.(data.trajectory_points.current_humanoid_part){data.stretchs.number-1}, 'Abs')
      set(handles.rolli_val,'String', num2str(data.trajectory_points.final_acceleration.(data.trajectory_points.current_humanoid_part)(4,data.stretchs.number-1)));
      set(handles.pitchi_val,'String', num2str(data.trajectory_points.final_acceleration.(data.trajectory_points.current_humanoid_part)(5,data.stretchs.number-1)));
      set(handles.yawi_val,'String', num2str(data.trajectory_points.final_acceleration.(data.trajectory_points.current_humanoid_part)(6,data.stretchs.number-1)));
    else
      disp('ERROR: No diff/abs orientation option')
    end    
  else
    error('TeoTraGen:wrongOption', 'Wrong option. Input_type has to be: positions, velocities or accelerations')
  end
end

% Final Point
if strcmp(data.trajectory_points.input_type,'Poses')
  set(handles.xf_val,'String', num2str(data.trajectory_points.final_point.(data.trajectory_points.current_humanoid_part)(1,data.stretchs.number)));
  set(handles.yf_val,'String', num2str(data.trajectory_points.final_point.(data.trajectory_points.current_humanoid_part)(2,data.stretchs.number)));
  set(handles.zf_val,'String', num2str(data.trajectory_points.final_point.(data.trajectory_points.current_humanoid_part)(3,data.stretchs.number)));
  set(handles.rollf_val,'String', num2str(data.trajectory_points.final_point.(data.trajectory_points.current_humanoid_part)(4,data.stretchs.number)));
  set(handles.pitchf_val,'String', num2str(data.trajectory_points.final_point.(data.trajectory_points.current_humanoid_part)(5,data.stretchs.number)));
  set(handles.yawf_val,'String', num2str(data.trajectory_points.final_point.(data.trajectory_points.current_humanoid_part)(6,data.stretchs.number))); 
elseif strcmp(data.trajectory_points.input_type,'Velocities')
  set(handles.xf_val,'String', num2str(data.trajectory_points.final_velocity.(data.trajectory_points.current_humanoid_part)(1,data.stretchs.number)));
  set(handles.yf_val,'String', num2str(data.trajectory_points.final_velocity.(data.trajectory_points.current_humanoid_part)(2,data.stretchs.number)));
  set(handles.zf_val,'String', num2str(data.trajectory_points.final_velocity.(data.trajectory_points.current_humanoid_part)(3,data.stretchs.number)));
  set(handles.rollf_val,'String', num2str(data.trajectory_points.final_velocity.(data.trajectory_points.current_humanoid_part)(4,data.stretchs.number)));
  set(handles.pitchf_val,'String', num2str(data.trajectory_points.final_velocity.(data.trajectory_points.current_humanoid_part)(5,data.stretchs.number)));
  set(handles.yawf_val,'String', num2str(data.trajectory_points.final_velocity.(data.trajectory_points.current_humanoid_part)(6,data.stretchs.number)));   
elseif strcmp(data.trajectory_points.input_type,'Accelerations')
  set(handles.xf_val,'String', num2str(data.trajectory_points.final_acceleration.(data.trajectory_points.current_humanoid_part)(1,data.stretchs.number)));
  set(handles.yf_val,'String', num2str(data.trajectory_points.final_acceleration.(data.trajectory_points.current_humanoid_part)(2,data.stretchs.number)));
  set(handles.zf_val,'String', num2str(data.trajectory_points.final_acceleration.(data.trajectory_points.current_humanoid_part)(3,data.stretchs.number)));
  set(handles.rollf_val,'String', num2str(data.trajectory_points.final_acceleration.(data.trajectory_points.current_humanoid_part)(4,data.stretchs.number)));
  set(handles.pitchf_val,'String', num2str(data.trajectory_points.final_acceleration.(data.trajectory_points.current_humanoid_part)(5,data.stretchs.number)));
  set(handles.yawf_val,'String', num2str(data.trajectory_points.final_acceleration.(data.trajectory_points.current_humanoid_part)(6,data.stretchs.number)));   
end
  
  
% Diff or Abs
if strcmp(data.trajectory_points.pos_diff.(data.trajectory_points.current_humanoid_part)(data.stretchs.number),'Diff')
  set (handles.diff_pos_button,'Value',1.0);
else
  set (handles.abs_pos_button,'Value',1.0);
end
if strcmp(data.trajectory_points.orient_diff.(data.trajectory_points.current_humanoid_part)(data.stretchs.number),'Diff')
  set (handles.diff_orient_button,'Value',1.0);
else
  set (handles.abs_orient_button,'Value',1.0);
end
if strcmp(data.trajectory_points.time_diff.(data.trajectory_points.current_humanoid_part)(data.stretchs.number),'Diff')
  set (handles.diff_time_button,'Value',1.0);
else
  set (handles.abs_time_button,'Value',1.0);
end
  

% Times
set (handles.T_val,'String',num2str(data.trajectory_points.T_val.(data.trajectory_points.current_humanoid_part)(data.stretchs.number)))
if data.stretchs.number == 1
  set (handles.t0_val,'String',num2str(data.trajectory_points.t0_val.(data.trajectory_points.current_humanoid_part)(data.stretchs.number)));
  set (handles.Ts_val,'String',num2str(data.trajectory_points.Ts_val.(data.trajectory_points.current_humanoid_part)(data.stretchs.number)));
else
  set (handles.Ts_val,'String',num2str(data.trajectory_points.Ts_val.(data.trajectory_points.current_humanoid_part)(data.stretchs.number-1))) 
  if strcmp(data.trajectory_points.orient_diff.(data.trajectory_points.current_humanoid_part){data.stretchs.number-1},'Diff')
    set (handles.t0_val,'String',num2str(data.trajectory_points.t0_val.(data.trajectory_points.current_humanoid_part)(data.stretchs.number-1)+data.trajectory_points.T_val.(data.trajectory_points.current_humanoid_part)(data.stretchs.number-1)));
  elseif strcmp(data.trajectory_points.orient_diff.(data.trajectory_points.current_humanoid_part){data.stretchs.number-1},'Abs')
    set (handles.t0_val,'String',num2str(data.trajectory_points.T_val.(data.trajectory_points.current_humanoid_part)(data.stretchs.number-1)));
  else
    disp('ERROR: No diff/abs time option')
  end  
end


% Interpolations
switch data.trajectory_points.interpola_pos.(data.trajectory_points.current_humanoid_part){data.stretchs.number}
  case 'Linear'
    set(handles.interpola_pos_popup,'Value',1);
  case 'Circular'
    set(handles.interpola_pos_popup,'Value',2);
  case 'Spline'
    set(handles.interpola_pos_popup,'Value',3);
  case 'Polynomial3'
    set(handles.interpola_pos_popup,'Value',4);
  case 'Cubic Spline'
    set(handles.interpola_pos_popup,'Value',5);
  case 'Polynomial5'
    set(handles.interpola_pos_popup,'Value',6);
  case 'Polynomial7'
    set(handles.interpola_pos_popup,'Value',7);
end

switch data.trajectory_points.interpola_orient.(data.trajectory_points.current_humanoid_part){data.stretchs.number}
  case 'Linear'
    set(handles.interpola_orient_popup,'Value',1);
  case 'Spline'
    set(handles.interpola_orient_popup,'Value',2);
  case 'Polynomial3'
    set(handles.interpola_orient_popup,'Value',3);
  case 'Cubic Spline'
    set(handles.interpola_orient_popup,'Value',4);
  case 'Polynomial5'
    set(handles.interpola_orient_popup,'Value',5);
  case 'Polynomial7'
    set(handles.interpola_orient_popup,'Value',6);
end

% Support foot
switch data.trajectory_points.support_foot.(data.trajectory_points.current_humanoid_part)(data.stretchs.number)
  case 0
    set(handles.support_popup,'Value',1);
  case -1
    set(handles.support_popup,'Value',2);
  case 1
    set(handles.support_popup,'Value',3);
end
    
    
function update_points(humanoid_part, handles)
global data

if strcmp(data.trajectory_points.input_type,'Poses')
  data.trajectory_points.initial_point.(humanoid_part)(1,data.stretchs.number) = str2num(get(handles.xi_val,'String'));
  data.trajectory_points.initial_point.(humanoid_part)(2,data.stretchs.number) = str2num(get(handles.yi_val,'String'));
  data.trajectory_points.initial_point.(humanoid_part)(3,data.stretchs.number) = str2num(get(handles.zi_val,'String'));
  data.trajectory_points.initial_point.(humanoid_part)(4,data.stretchs.number) = str2num(get(handles.rolli_val,'String'));
  data.trajectory_points.initial_point.(humanoid_part)(5,data.stretchs.number) = str2num(get(handles.pitchi_val,'String'));
  data.trajectory_points.initial_point.(humanoid_part)(6,data.stretchs.number) = str2num(get(handles.yawi_val,'String'));
  data.trajectory_points.final_point.(humanoid_part)(1,data.stretchs.number) = str2num(get(handles.xf_val,'String'));
  data.trajectory_points.final_point.(humanoid_part)(2,data.stretchs.number) = str2num(get(handles.yf_val,'String'));
  data.trajectory_points.final_point.(humanoid_part)(3,data.stretchs.number) = str2num(get(handles.zf_val,'String'));
  data.trajectory_points.final_point.(humanoid_part)(4,data.stretchs.number) = str2num(get(handles.rollf_val,'String'));
  data.trajectory_points.final_point.(humanoid_part)(5,data.stretchs.number) = str2num(get(handles.pitchf_val,'String'));
  data.trajectory_points.final_point.(humanoid_part)(6,data.stretchs.number) = str2num(get(handles.yawf_val,'String'));
  
elseif strcmp(data.trajectory_points.input_type,'Velocities')
  data.trajectory_points.initial_velocity.(humanoid_part)(1,data.stretchs.number) = str2num(get(handles.xi_val,'String'));
  data.trajectory_points.initial_velocity.(humanoid_part)(2,data.stretchs.number) = str2num(get(handles.yi_val,'String'));
  data.trajectory_points.initial_velocity.(humanoid_part)(3,data.stretchs.number) = str2num(get(handles.zi_val,'String'));
  data.trajectory_points.initial_velocity.(humanoid_part)(4,data.stretchs.number) = str2num(get(handles.rolli_val,'String'));
  data.trajectory_points.initial_velocity.(humanoid_part)(5,data.stretchs.number) = str2num(get(handles.pitchi_val,'String'));
  data.trajectory_points.initial_velocity.(humanoid_part)(6,data.stretchs.number) = str2num(get(handles.yawi_val,'String'));
  data.trajectory_points.final_velocity.(humanoid_part)(1,data.stretchs.number) = str2num(get(handles.xf_val,'String'));
  data.trajectory_points.final_velocity.(humanoid_part)(2,data.stretchs.number) = str2num(get(handles.yf_val,'String'));
  data.trajectory_points.final_velocity.(humanoid_part)(3,data.stretchs.number) = str2num(get(handles.zf_val,'String'));
  data.trajectory_points.final_velocity.(humanoid_part)(4,data.stretchs.number) = str2num(get(handles.rollf_val,'String'));
  data.trajectory_points.final_velocity.(humanoid_part)(5,data.stretchs.number) = str2num(get(handles.pitchf_val,'String'));
  data.trajectory_points.final_velocity.(humanoid_part)(6,data.stretchs.number) = str2num(get(handles.yawf_val,'String'));
  
elseif strcmp(data.trajectory_points.input_type,'Accelerations')
  data.trajectory_points.initial_acceleration.(humanoid_part)(1,data.stretchs.number) = str2num(get(handles.xi_val,'String'));
  data.trajectory_points.initial_acceleration.(humanoid_part)(2,data.stretchs.number) = str2num(get(handles.yi_val,'String'));
  data.trajectory_points.initial_acceleration.(humanoid_part)(3,data.stretchs.number) = str2num(get(handles.zi_val,'String'));
  data.trajectory_points.initial_acceleration.(humanoid_part)(4,data.stretchs.number) = str2num(get(handles.rolli_val,'String'));
  data.trajectory_points.initial_acceleration.(humanoid_part)(5,data.stretchs.number) = str2num(get(handles.pitchi_val,'String'));
  data.trajectory_points.initial_acceleration.(humanoid_part)(6,data.stretchs.number) = str2num(get(handles.yawi_val,'String'));
  data.trajectory_points.final_acceleration.(humanoid_part)(1,data.stretchs.number) = str2num(get(handles.xf_val,'String'));
  data.trajectory_points.final_acceleration.(humanoid_part)(2,data.stretchs.number) = str2num(get(handles.yf_val,'String'));
  data.trajectory_points.final_acceleration.(humanoid_part)(3,data.stretchs.number) = str2num(get(handles.zf_val,'String'));
  data.trajectory_points.final_acceleration.(humanoid_part)(4,data.stretchs.number) = str2num(get(handles.rollf_val,'String'));
  data.trajectory_points.final_acceleration.(humanoid_part)(5,data.stretchs.number) = str2num(get(handles.pitchf_val,'String'));
  data.trajectory_points.final_acceleration.(humanoid_part)(6,data.stretchs.number) = str2num(get(handles.yawf_val,'String'));
end


data.trajectory_points.t0_val.(humanoid_part)(data.stretchs.number) = str2num(get(handles.t0_val,'String'));
data.trajectory_points.Ts_val.(humanoid_part)(data.stretchs.number) = str2num(get(handles.Ts_val,'String'));
% All humanoid_parts will have the same T value
for jj = 1:(length(data.trajectory_settings.humanoid_fields)-1),
  data.trajectory_points.T_val.(data.trajectory_settings.humanoid_fields(jj).name)(data.stretchs.number) = str2num(get(handles.T_val,'String'));
end
    

function num_points_text_Callback(hObject, eventdata, handles)
global data
total_points = str2double(get(hObject,'String'));

% Check the correct number of points
if ((total_points<1)||(round(total_points) ~= total_points))
  warndlg('ERROR: Number of points has to be an integer greater than 1','Points Error');
  if isnan(total_points),
    total_points = data.trajectory_settings.total_points; % Use previous total_points
  else
    total_points = floor(total_points);
  end
  
  if total_points<1
      total_points = 1;
  else
      set(hObject,'String',num2str(total_points));
  end
end

prev_total_points = data.trajectory_settings.total_points;

data.trajectory_settings.total_points = total_points;
set(hObject,'String',num2str(data.trajectory_settings.total_points));
message_num_points(data.trajectory_settings.total_points, handles);

if prev_total_points < total_points
  add_default_data(hObject, handles);
elseif prev_total_points > total_points
  remove_default_data(hObject, handles);  
end

stretch_trajectory_popup_Callback(handles.stretch_trajectory_popup, eventdata, handles);


guidata(hObject,handles)
update_stretch_popup(hObject, handles)


function num_points_text_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
  set(hObject, 'BackgroundColor', 'white');
end


function stretch_trajectory_popup_Callback(hObject, eventdata, handles)
global data
if isfield(data.trajectory_points, 'current_humanoid_part')
  % Update points and support foot
  update_points(data.trajectory_points.current_humanoid_part, handles);
  update_support_foot(handles);

  data.stretchs.number = get(hObject,'Value');
  messages_master(hObject, handles);
  set(handles.humanoid_part_panel, 'Title', ['Poses ' data.trajectory_points.input_type ' - "Stretch ' data.stretchs.trajectory{data.stretchs.number} '"']);
  show_selected_values(hObject, handles)
else
  message_no_points(hObject, handles);
  set(hObject,'Value', 1); % Return to previous stretch
end


function stretch_trajectory_popup_CreateFcn(hObject, eventdata, handles)
  if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
      set(hObject,'BackgroundColor','white');
  end

    
function add_default_data(hObject, handles)
global data

% Get fields that needs add default data
for jj = 1:(length(data.trajectory_settings.humanoid_fields) - 1),
  actual_size = size(data.trajectory_points.initial_point.(data.trajectory_settings.humanoid_fields(jj).name), 2);
  if actual_size < data.trajectory_settings.total_points
    for ii = 1:(data.trajectory_settings.total_points - actual_size),
      data.trajectory_points.initial_point.(data.trajectory_settings.humanoid_fields(jj).name)(:,end+1) = data.trajectory_points.final_point.(data.trajectory_settings.humanoid_fields(jj).name)(:,end);
      data.trajectory_points.initial_velocity.(data.trajectory_settings.humanoid_fields(jj).name)(:,end+1) = data.trajectory_points.final_velocity.(data.trajectory_settings.humanoid_fields(jj).name)(:,end);
      data.trajectory_points.initial_acceleration.(data.trajectory_settings.humanoid_fields(jj).name)(:,end+1) = data.trajectory_points.final_acceleration.(data.trajectory_settings.humanoid_fields(jj).name)(:,end);
      data.trajectory_points.final_point.(data.trajectory_settings.humanoid_fields(jj).name)(:,end+1) = zeros(data.trajectory_settings.humanoid_fields(jj).size, 1);
      data.trajectory_points.final_velocity.(data.trajectory_settings.humanoid_fields(jj).name)(:,end+1) = zeros(data.trajectory_settings.humanoid_fields(jj).size, 1);
      data.trajectory_points.final_acceleration.(data.trajectory_settings.humanoid_fields(jj).name)(:,end+1) = zeros(data.trajectory_settings.humanoid_fields(jj).size, 1);
      data.trajectory_points.t0_val.(data.trajectory_settings.humanoid_fields(jj).name)(end+1) = 0;
      data.trajectory_points.T_val.(data.trajectory_settings.humanoid_fields(jj).name)(end+1) = 0;
      data.trajectory_points.Ts_val.(data.trajectory_settings.humanoid_fields(jj).name)(end+1) = data.trajectory_settings.parameters.Ts;
      data.trajectory_points.pos_diff.(data.trajectory_settings.humanoid_fields(jj).name){end+1} = 'Diff';
      data.trajectory_points.orient_diff.(data.trajectory_settings.humanoid_fields(jj).name){end+1} = 'Diff';
      data.trajectory_points.time_diff.(data.trajectory_settings.humanoid_fields(jj).name){end+1} = 'Diff';
      data.trajectory_points.interpola_pos.(data.trajectory_settings.humanoid_fields(jj).name){end+1} = 'Linear';
      data.trajectory_points.interpola_orient.(data.trajectory_settings.humanoid_fields(jj).name){end+1} = 'Linear';  
      data.trajectory_points.support_foot.(data.trajectory_settings.humanoid_fields(jj).name)(end+1) = 0;
    end
  end
end





% for ii = 1:(data.trajectory_settings.total_points - actual_size);    
%     for jj = 1:(length(data.trajectory_settings.humanoid_fields)-1)
%         data.trajectory_points.initial_point.(data.trajectory_settings.humanoid_fields(jj).name)(:,end+1) = data.trajectory_points.final_point.(data.trajectory_settings.humanoid_fields(jj).name)(:,end);
% %         data.trajectory_points.final_point.(data.trajectory_settings.humanoid_fields(jj).name)(:,end+1) = zeros(data.trajectory_settings.humanoid_fields(jj).size, 1);
%         data.trajectory_points.t0_val.(data.trajectory_settings.humanoid_fields(jj).name)(end+1) = 0;
%         data.trajectory_points.T_val.(data.trajectory_settings.humanoid_fields(jj).name)(end+1) = 0;
%         data.trajectory_points.Ts_val.(data.trajectory_settings.humanoid_fields(jj).name)(end+1) = data.trajectory_settings.parameters.Ts;
%         data.trajectory_points.pos_diff.(data.trajectory_settings.humanoid_fields(jj).name){end+1} = 'Diff';
%         data.trajectory_points.orient_diff.(data.trajectory_settings.humanoid_fields(jj).name){end+1} = 'Diff';
%         data.trajectory_points.time_diff.(data.trajectory_settings.humanoid_fields(jj).name){end+1} = 'Diff';
%         data.trajectory_points.interpola_pos.(data.trajectory_settings.humanoid_fields(jj).name){end+1} = 'Linear';
%         data.trajectory_points.interpola_orient.(data.trajectory_settings.humanoid_fields(jj).name){end+1} = 'Linear';  
%         data.trajectory_points.support_foot.(data.trajectory_settings.humanoid_fields(jj).name)(end+1) = 0;
%     end
% end
guidata(hObject, handles)


function remove_default_data(hObject, handles)
global data

% Get fields that needs add default data
for jj = 1:(length(data.trajectory_settings.humanoid_fields) - 1),
  actual_size = size(data.trajectory_points.initial_point.(data.trajectory_settings.humanoid_fields(jj).name), 2);
  if actual_size > data.trajectory_settings.total_points
    for ii = 1:(actual_size - data.trajectory_settings.total_points),
      data.trajectory_points.initial_point.(data.trajectory_settings.humanoid_fields(jj).name)(:,end) = [];
      data.trajectory_points.initial_velocity.(data.trajectory_settings.humanoid_fields(jj).name)(:,end) = [];
      data.trajectory_points.initial_acceleration.(data.trajectory_settings.humanoid_fields(jj).name)(:,end) = [];
      data.trajectory_points.final_point.(data.trajectory_settings.humanoid_fields(jj).name)(:,end) = [];
      data.trajectory_points.final_velocity.(data.trajectory_settings.humanoid_fields(jj).name)(:,end) = [];
      data.trajectory_points.final_acceleration.(data.trajectory_settings.humanoid_fields(jj).name)(:,end) = [];
      data.trajectory_points.t0_val.(data.trajectory_settings.humanoid_fields(jj).name)(end) = [];
      data.trajectory_points.T_val.(data.trajectory_settings.humanoid_fields(jj).name)(end) = [];
      data.trajectory_points.Ts_val.(data.trajectory_settings.humanoid_fields(jj).name)(end) = [];
      data.trajectory_points.pos_diff.(data.trajectory_settings.humanoid_fields(jj).name){end} = [];
      data.trajectory_points.orient_diff.(data.trajectory_settings.humanoid_fields(jj).name){end} = [];
      data.trajectory_points.time_diff.(data.trajectory_settings.humanoid_fields(jj).name){end} = [];
      data.trajectory_points.interpola_pos.(data.trajectory_settings.humanoid_fields(jj).name){end} = [];
      data.trajectory_points.interpola_orient.(data.trajectory_settings.humanoid_fields(jj).name){end} = [];
      data.trajectory_points.support_foot.(data.trajectory_settings.humanoid_fields(jj).name)(end) = [];
    end
  end
end

guidata(hObject, handles)



function update_support_foot(handles)
global data

support_foot = get(handles.support_popup,'Value');

% Double or Simple Support
% 0--> Double. -1--> Right Foot. 1-->Left Support
% All humanoid_parts have the same Support value
for jj = 1:(length(data.trajectory_settings.humanoid_fields) - 1),
  switch support_foot
    case 1
      data.trajectory_points.support_foot.(data.trajectory_settings.humanoid_fields(jj).name)(data.stretchs.number) = 0;
    case 2
      data.trajectory_points.support_foot.(data.trajectory_settings.humanoid_fields(jj).name)(data.stretchs.number) = -1;
    case 3
      data.trajectory_points.support_foot.(data.trajectory_settings.humanoid_fields(jj).name)(data.stretchs.number) = 1;
  end
end


function messages_master(hObject, handles)
global data
% Show some messages if current options are wrong
if (strcmp(data.trajectory_points.current_humanoid_part,'RF')&&(data.trajectory_points.support_foot.RF(data.stretchs.number)==0))
  set(handles.label_points,'String','Right Foot will not move in Double Support');
  set(handles.label_points,'BackGroundColor',[1 1 0.2]); % Yellow
elseif (strcmp(data.trajectory_points.current_humanoid_part,'RF')&&(data.trajectory_points.support_foot.RF(data.stretchs.number)==-1))
  set(handles.label_points,'String','Right Foot will not move in Right Leg Support');
  set(handles.label_points,'BackGroundColor',[1 1 0.2]); % Yellow
elseif (strcmp(data.trajectory_points.current_humanoid_part,'LF')&&(data.trajectory_points.support_foot.RF(data.stretchs.number)==0))
  set(handles.label_points,'String','Left Foot will not move in Double Support');
  set(handles.label_points,'BackGroundColor',[1 1 0.2]); % Yellow
elseif (strcmp(data.trajectory_points.current_humanoid_part,'LF')&&(data.trajectory_points.support_foot.RF(data.stretchs.number)==1))
  set(handles.label_points,'String','Left Foot will not move in Left Leg Support');
  set(handles.label_points,'BackGroundColor',[1 1 0.2]); % Yellow
else
  %Clear label_points
  set(handles.label_points,'String','');
  set(handles.label_points,'BackGroundColor',[0.702 0.78 1]); % Blue
end


function message_no_points(hObject, handles)
  set(handles.label_points,'String','Select any body part to generate stretchs');
  set(handles.label_points,'BackGroundColor',[1 1 0.2]);

    
function save_joint_button_Callback(hObject, eventdata, handles)
global trajectories
traj_name_string = get(handles.traj_name_text,'String');
save_trajectory_window(traj_name_string, trajectories.joints.q, trajectories.joints.dq, trajectories.joints.ddq, 'joints');


function axis_movement_pushbutton_Callback(hObject, eventdata, handles)
global data trajectories
pose_rpy = (trajectories.operational.trajectory.(data.trajectory_points.current_humanoid_part))';
pose_tr=zeros(4,4,size(pose_rpy,1));

for i = 1:size(pose_rpy,1)
  pose_tr(:,:,i) = transl(pose_rpy(i,1:3)) * rpy2tr(pose_rpy(i,4:6));
end
try
  set(handles.label_points,'String','');
  set(handles.label_points,'BackGroundColor',[0.702 0.78 1]);
  figure(60),title(['Humanoid Axes: ' data.trajectory_points.current_humanoid_part '   ---   Ts = ' num2str(data.trajectory_settings.parameters.Ts)]),tranimate(pose_tr);
catch
  set(handles.label_points,'String','Visualize Axis Movement aborted!');
  set(handles.label_points,'BackGroundColor',[1 1 0.2]); 
end


function pushbutton_matlab_visualization_Callback(hObject, eventdata, handles)
global trajectories data
try
  set(handles.label_points,'String','');
  set(handles.label_points,'BackGroundColor',[0.702 0.78 1]);
  matlab_visualization (trajectories.operational.trajectory, trajectories.joints.q, data.trajectory_settings.TEO, data.trajectory_settings.h)
catch
  set(handles.label_points,'String','Visualize Humanoid Movement aborted!');
  set(handles.label_points,'BackGroundColor',[1 1 0.2]);     
end


function pushbutton9_Callback(hObject, eventdata, handles)


function check_times_data
for pp=1:total_points
    for ff=1:(length(data.trajectory_settings.humanoid_fields)-1)
    end
% set (handles.T_val,'String',num2str(data.trajectory_points.T_val.(humanoid_part)(data.stretchs.number)))
end


% --------------------------------------------------------------------
function Back_Callback(hObject, eventdata, handles)
% DO NOT REMOVE THIS FUNCTION. JUST LEAVE IT EMPTY


function input_type_popup_Callback(hObject, eventdata, handles)
global data
contents = cellstr(get(hObject,'String'));
data.trajectory_points.input_type = contents{get(hObject,'Value')};
stretch_trajectory_popup_Callback(handles.stretch_trajectory_popup, eventdata, handles);


function input_type_popup_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function yf_val_Callback(hObject, eventdata, handles)


function xf_val_Callback(hObject, eventdata, handles)


function zf_val_Callback(hObject, eventdata, handles)


function rollf_val_Callback(hObject, eventdata, handles)


function pitchf_val_Callback(hObject, eventdata, handles)


function yawf_val_Callback(hObject, eventdata, handles)


function T_val_Callback(hObject, eventdata, handles)


function pb_num_points_inc_Callback(hObject, eventdata, handles)
global data
% Add one point
data.trajectory_settings.total_points = data.trajectory_settings.total_points + 1;
set (handles.num_points_text, 'String', num2str(data.trajectory_settings.total_points));
message_num_points(data.trajectory_settings.total_points, handles);

add_default_data(hObject, handles);
stretch_trajectory_popup_Callback(handles.stretch_trajectory_popup, eventdata, handles);

guidata(hObject,handles)
update_stretch_popup(hObject, handles);


function pb_num_points_dec_Callback(hObject, eventdata, handles)
global data
% Substract one point
if data.trajectory_settings.total_points > 1
  data.trajectory_settings.total_points = data.trajectory_settings.total_points - 1;
  set (handles.num_points_text, 'String', num2str(data.trajectory_settings.total_points));
  message_num_points(data.trajectory_settings.total_points, handles);  
else
  message_num_points(0, handles);
end

remove_default_data(hObject, handles);
stretch_trajectory_popup_Callback(handles.stretch_trajectory_popup, eventdata, handles);

guidata(hObject,handles);
update_stretch_popup(hObject, handles);


function message_num_points(num_points, handles)
if num_points == 0
  warndlg('ERROR: Number of points has to be an integer greater than 1','Points Error');
  message_string = 'ERROR: Trajectory needs at least another point!';
  message_color = [1 0.2 0.2];
else
  message_string = ['INFO: Set ', num2str(num_points), ' points'];
  message_color = [0.702 0.78 1];
end

set(handles.label_points, 'String', message_string);
set(handles.label_points, 'BackGroundColor', message_color);


function update_stretch_popup(hObject, handles)
global data
% Set the values of the stretch_trajectory_popup values
total_points = data.trajectory_settings.total_points;

data.stretchs.trajectory = cell(1, total_points);
for jj = 1:total_points,
    data.stretchs.trajectory{jj} = [num2str(jj-1) ' - ' num2str(jj)];
end

set(handles.stretch_trajectory_popup, 'String', data.stretchs.trajectory)
set(handles.stretch_trajectory_popup, 'Value', 1);
guidata(hObject,handles);


function checkbox1_Callback(hObject, eventdata, handles)


function humanoid_part_panel_CreateFcn(hObject, eventdata, handles)


function input_type_popup_KeyPressFcn(hObject, eventdata, handles)


function pushbutton_ros_visualization_Callback(hObject, eventdata, handles)
global data trajectories
ros_visualization(trajectories.joints.q, trajectories.operational.trajectory.SF, data.trajectory_settings.parameters.Ts);


function edit_e0_Callback(hObject, eventdata, handles)


function edit_e0_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function T_values_ok = check_T_values(handles)
global data;
% if data.trajectory_points.T_val.CoM(1) == 0
%   msg = ['ERROR: Thre is not time in Stretch 0 - 1'];
%   warndlg(msg,'Times values Error');
%   T_values_ok = 0;
%   return;
% end
for i = 1:data.trajectory_settings.total_points-1,
%   t0 = data.trajectory_points.t0_val.CoM(i);
%   t0next = data.trajectory_points.t0_val.CoM(i+1);
% 	if data.trajectory_points.T_val.(data.trajectory_settings.humanoid_fields(jj).name)(data.stretchs.number)
  if data.trajectory_points.t0_val.CoM(i) >= data.trajectory_points.t0_val.CoM(i+1)
    msg = ['ERROR: There is not time in Stretch ', num2str(i-1), ' - ', num2str(i), ' to generate the trajectory'];
    warndlg(msg,'Times values Error');
    T_values_ok = 0;
    return;
  end
end
if data.trajectory_points.T_val.CoM(data.trajectory_settings.total_points) == 0,
  msg = ['ERROR: Last stretch number cannot be 0'];
  warndlg(msg,'Times values Error');
  T_values_ok = 0;
else
  T_values_ok = 1;
end


function ik_gains_Callback(hObject, eventdata, handles)
% Change the values of gains
global data
  [data.trajectory_settings.parameters.kp data.trajectory_settings.parameters.ko] = change_ik_gains(data.trajectory_settings.parameters.kp, data.trajectory_settings.parameters.ko);
% guidata(hObject,handles);


function change_parameters_Callback(hObject, eventdata, handles)


function pushbutton_plots_Callback(hObject, eventdata, handles)
global trajectories data
trajectory_plots(trajectories.operational, trajectories.joints, data.trajectory_settings.TEO);


function initial_configuration_Callback(hObject, eventdata, handles)
global data
data.trajectory_settings.q0.data  = change_configuration(data.trajectory_settings.q0.data, data.trajectory_settings.TEO);
guidata(hObject,handles);


function xi_val_Callback(hObject, eventdata, handles)

function traj_name_text_Callback(hObject, eventdata, handles)


% --------------------------------------------------------------------
function tools_Callback(hObject, eventdata, handles)
% hObject    handle to tools (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


function relative_position_conversion_Callback(hObject, eventdata, handles)
global data trajectories

prevq = trajectories.joints.q;
prevdq = trajectories.joints.dq;
prevddq = trajectories.joints.ddq;

[newq, newdq, newddq, support_foot] = joints_space_interpolation(data.trajectory_settings.TEO, data.trajectory_settings.h, trajectories.joints.q(:,1), zeros(26,1), data.trajectory_settings.parameters.Ts);

if (~isempty(newq) && ~isempty(newdq) && ~isempty(newddq))
  trajectories.joints.q = [newq prevq];
  trajectories.joints.dq = [newdq prevdq];
  trajectories.joints.ddq = [newddq prevddq];
  
  trajectories.operational.trajectory.SF = [support_foot trajectories.operational.trajectory.SF];
  trajectories.operational.trajectory.time = 0:data.trajectory_settings.parameters.Ts:(data.trajectory_settings.parameters.Ts*size(trajectories.joints.q,2)-data.trajectory_settings.parameters.Ts);
end


% --- Executes on button press in pushbutton_reset_trajectories.
function pushbutton_reset_trajectories_Callback(hObject, eventdata, handles)

global data trajectories

trajectories.operational = data.trajectory_settings.initial;
trajectories.Inertialtrajectories = data.trajectory_settings.Iinitial;

% Default layout
set([handles.data_panel handles.operational_panel], 'Visible','off');
% set([handles.save_operational_button handles.gen_joint_button ],'Enable','off');
set([handles.diff_pos_button handles.abs_pos_button handles.diff_orient_button handles.abs_orient_button handles.diff_time_button handles.abs_time_button],'Value',0);
set(handles.Ts_val,'String', data.trajectory_settings.parameters.Ts);
set([handles.save_operational_button handles.gen_joint_button handles.axis_movement_pushbutton handles.pushbutton_plots handles.save_joint_button handles.pushbutton_matlab_visualization handles.pushbutton_ros_visualization],'Enable','off')

% Enable/Disable generation for corresponding humanoid part
if data.trajectory_settings.body_parts.CoM == 0
    set(handles.CoM_button,'Enable','off');
    disp('Disable CoM generation');
end
if data.trajectory_settings.body_parts.RF == 0
    set(handles.RF_button,'Enable','off');
    disp('Disable RF generation');
end
if data.trajectory_settings.body_parts.LF == 0
    set(handles.LF_button,'Enable','off');
    disp('Disable LF generation');
end
if data.trajectory_settings.body_parts.RH == 0
    set(handles.RH_button,'Enable','off');
    disp('Disable RH generation');
end
if data.trajectory_settings.body_parts.LH == 0
    set(handles.LH_button,'Enable','off');
    disp('Disable LH generation');
end

set(handles.Waist_button,'Enable','off'); % Disable for now...

% Enable default GUI objects
set ([handles.diff_pos_button, handles.diff_orient_button, handles.diff_time_button handles.support_popup], 'Value', 1.0)
set(handles.CoM_button, 'Value', 0)  % Avoid CoM_button be pressed

% At least one one point in total_points
data.trajectory_settings.total_points = 1;
set (handles.num_points_text, 'String', num2str(data.trajectory_settings.total_points))

data.trajectory_points.input_type = 'Poses';
% humanoid_part_to_plot = 'RH';

% Create initial values
for jj = 1:(length(data.trajectory_settings.humanoid_fields)-1)
  data.trajectory_points.initial_point.(data.trajectory_settings.humanoid_fields(jj).name) = zeros(data.trajectory_settings.humanoid_fields(jj).size, 1);
  data.trajectory_points.initial_velocity.(data.trajectory_settings.humanoid_fields(jj).name) = zeros(data.trajectory_settings.humanoid_fields(jj).size, 1);
  data.trajectory_points.initial_acceleration.(data.trajectory_settings.humanoid_fields(jj).name) = zeros(data.trajectory_settings.humanoid_fields(jj).size, 1);
  data.trajectory_points.final_point.(data.trajectory_settings.humanoid_fields(jj).name) = zeros(data.trajectory_settings.humanoid_fields(jj).size, 1);
  data.trajectory_points.final_velocity.(data.trajectory_settings.humanoid_fields(jj).name) = zeros(data.trajectory_settings.humanoid_fields(jj).size, 1);
  data.trajectory_points.final_acceleration.(data.trajectory_settings.humanoid_fields(jj).name) = zeros(data.trajectory_settings.humanoid_fields(jj).size, 1);  
  data.trajectory_points.t0_val.(data.trajectory_settings.humanoid_fields(jj).name) = 0;
  data.trajectory_points.T_val.(data.trajectory_settings.humanoid_fields(jj).name) = 0;
  data.trajectory_points.Ts_val.(data.trajectory_settings.humanoid_fields(jj).name) = data.trajectory_settings.parameters.Ts;
  data.trajectory_points.pos_diff.(data.trajectory_settings.humanoid_fields(jj).name){1} = 'Diff';
  data.trajectory_points.orient_diff.(data.trajectory_settings.humanoid_fields(jj).name){1} = 'Diff';
  data.trajectory_points.time_diff.(data.trajectory_settings.humanoid_fields(jj).name){1} = 'Diff';
  data.trajectory_points.interpola_pos.(data.trajectory_settings.humanoid_fields(jj).name){1} = 'Linear';
  data.trajectory_points.interpola_orient.(data.trajectory_settings.humanoid_fields(jj).name){1} = 'Linear';
  data.trajectory_points.support_foot.(data.trajectory_settings.humanoid_fields(jj).name) = 0; % Double support
end


% Update handles structure   
guidata(hObject, handles);

num_points_text_Callback(handles.num_points_text, eventdata, handles);

% Update handles structure   
guidata(hObject, handles);
