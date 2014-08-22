function varargout = trajectory_plots(varargin)
% TRAJECTORY_PLOTS MATLAB code for trajectory_plots.fig
%      TRAJECTORY_PLOTS, by itself, creates a new TRAJECTORY_PLOTS or raises the existing
%      singleton*.
%
%      H = TRAJECTORY_PLOTS returns the handle to a new TRAJECTORY_PLOTS or the handle to
%      the existing singleton*.
%
%      TRAJECTORY_PLOTS('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in TRAJECTORY_PLOTS.M with the given input arguments.
%
%      TRAJECTORY_PLOTS('Property','Value',...) creates a new TRAJECTORY_PLOTS or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before trajectory_plots_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to trajectory_plots_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help trajectory_plots

% Last Modified by GUIDE v2.5 03-Apr-2014 22:53:39

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @trajectory_plots_OpeningFcn, ...
                   'gui_OutputFcn',  @trajectory_plots_OutputFcn, ...
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


% --- Executes just before trajectory_plots is made visible.
function trajectory_plots_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to trajectory_plots (see VARARGIN)

% Choose default command line output for trajectory_plots
handles.output = hObject;

if isempty(varargin)
  warning('WarnTEOTraGen:noInputGUI', 'No inputs for trajectory_points');
  handles.operational_space_traj = 0;
  handles.joints_space_traj = 0;
  handles.humanoid_description = 0;
elseif numel(varargin) > 4
  warning('WarnTEOTraGen:wrongInputGUI', 'Unnecessary inputs for trajectory_points');
  handles.operational_space_traj = 0;
  handles.joints_space_traj = 0;
  handles.humanoid_description = 0;
else   
  % Get trajectories data
  handles.operational_space_traj = varargin{1};
  if numel(varargin) < 3
    handles.humanoid_description = 0;
    handles.joints_space_traj = varargin{2};
  elseif numel(varargin) < 2
    handles.humanoid_description = 0;
    handles.joints_space_traj = 0;  
  else
    handles.joints_space_traj = varargin{2};
    handles.humanoid_description = varargin{3};
  end
end

% Show message based on input data
if ~isfield(handles.operational_space_traj,'trajectory') && ~isfield(handles.joints_space_traj,'q')
  set(handles.message_label,'String', 'TEOTraGen ERROR: No input data. Nothing to plot');
  set(handles.message_label,'BackGroundColor',[1 0.2 0.2]); % Red
elseif isfield(handles.operational_space_traj,'trajectory') && ~isfield(handles.joints_space_traj,'q')
  set(handles.message_label,'String', 'TEOTraGen WARNING: It will only plot operational space trajectories');
  set(handles.message_label,'BackGroundColor',[1 1 0.2]); % Yellow
  set(handles.pushbutton_operational,'Enable','on');
elseif isfield(handles.operational_space_traj,'trajectory') && isfield(handles.joints_space_traj,'q')
  if ~isfield(handles.humanoid_description,'legs')
    set(handles.message_label,'String', 'TEOTraGen WARNING: Joint space trajectories plots will not have joints limits');
    set(handles.message_label,'BackGroundColor',[1 1 0.2]); % Yellow
  end
  set(handles.pushbutton_operational,'Enable','on');
  set(handles.pushbutton_joints,'Enable','on');
else
  error('ErrorTEOTraGen:wrongInputGUI', 'Wrong inputs option');
end

% Default plotting values
handles.space = 'operational';
handles.humanoid_part = 'CoM';
handles.trajectory = 'pose';
guidata(hObject, handles);
update_button_color(handles.pushbutton_operational);
update_button_color(handles.pushbutton_com);
update_button_color(handles.pushbutton_pose);
update_plots(handles)

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes trajectory_plots wait for user response (see UIRESUME)
% uiwait(handles.figure1);

function update_plots(handles)
message = ['TEOTraGen INFO: Plotting ' handles.space ' ' handles.trajectory 'trajectory for ' handles.humanoid_part ' part ...'];
set(handles.message_label,'String', message);
set(handles.message_label,'BackGroundColor',[1 1 0.2]); % Yellow

if strcmp(handles.space,'operational')
  plot_operational(handles);
else
  plot_joints(handles);
end
message = ['TEOTraGen INFO: ' [upper(handles.space(1)) handles.space(2:end)] ' ' handles.trajectory ' for ' handles.humanoid_part ' part OK'];
set(handles.message_label,'String', message);
set(handles.message_label,'BackGroundColor',[0.702 0.78 1]); % Blue

function plot_operational(handles)
plots_titles = {'X' 'Y' 'Z' 'Roll' 'Pitch' 'Yaw'};
titles_y = {'[m]' '[m]' '[m]' '[rad]' '[rad]' '[rad]'...
    '[m/s]' '[m/s]' '[m/s]' '[rad/s]' '[rad/s]' '[rad/s]'...
    '[m/s^2]' '[m/s^2]' '[m/s^2]' '[rad/s^2]' '[rad/s^2]' '[rad/s^2]'};
axes_order = {'1' '3' '5' '2' '4' '6'};
  
if strcmp(handles.trajectory,'pose')
  traj_type = 'trajectory';
  titles_y_offset = 0;
elseif strcmp(handles.trajectory,'velocity')
  traj_type = 'd_trajectory';
  titles_y_offset = 6;
elseif strcmp(handles.trajectory,'acceleration')
  traj_type = 'dd_trajectory';
  titles_y_offset = 12;
else
  error('ErrorTEOTraGen:wrongVariableOption', 'Wrong handles.trajectory value');
end
  
for jj = 1:6
  axes(handles.(strcat('axes',char(axes_order(jj)))))
  cla(handles.(strcat('axes',char(axes_order(jj)))),'reset')
  plot(handles.(strcat('axes',char(axes_order(jj)))), handles.operational_space_traj.(traj_type).time, ...
                        handles.operational_space_traj.(traj_type).(handles.humanoid_part)(jj,:));
  hold on;
  title(handles.(strcat('axes',char(axes_order(jj)))),plots_titles(jj),'color','w')
  xlabel(handles.(strcat('axes',char(axes_order(jj)))),'t [s]')
  ylabel(handles.(strcat('axes',char(axes_order(jj)))),titles_y(jj+titles_y_offset),'color','w')
  hold off;
    
end  

function plot_joints(handles)
plots_titles = {'Hip Yaw' 'Hip Roll' 'Hip Pitch' 'Knee Pitch' 'Ankle Pitch' 'Ankle Roll'...
                'Torso Yaw' 'Torso Pitch' ...
                'Shoulder Pitch' 'Shoulder Roll' 'Shoulder Yaw' 'Elbow Pitch' 'Wrist Yaw' 'Wrist Pitch'};
titles_y = {'[rad]' '[rad]' '[rad]' '[rad]' '[rad]' '[rad]'...
    '[rad/s]' '[rad/s]' '[rad/s]' '[rad/s]' '[rad/s]' '[rad/s]'...
    '[rad/s^2]' '[rad/s^2]' '[rad/s^2]' '[rad/s^2]' '[rad/s^2]' '[rad/s^2]'};
axes_order = {'1' '2' '3' '4' '5' '6'};

if strcmp(handles.trajectory,'pose')
  traj_type = 'q';
  titles_y_offset = 0;
elseif strcmp(handles.trajectory,'velocity')
  traj_type = 'dq';
  titles_y_offset = 6;
elseif strcmp(handles.trajectory,'acceleration')
  traj_type = 'ddq';
  titles_y_offset = 12;
else
  error('ErrorTEOTraGen:wrongVariableOption', 'Wrong handles.trajectory value');
end

switch handles.humanoid_part
  case 'RF'
    traj_element_offset = 0;
    plots_title_offset = 0;
    j_limit1 = 'legs';j_limit2 = 'right';
  case 'LF'
    traj_element_offset = 6;
    plots_title_offset = 0;
    j_limit1 = 'legs';j_limit2 = 'left';
  case 'CoM'
    traj_element_offset = 12;
    plots_title_offset = 6;
    j_limit1 = 'waist';
  case 'RH'
    traj_element_offset = 14;
    plots_title_offset = 8;
    j_limit1 = 'arms'; j_limit2 = 'right';
  case 'LH'
    traj_element_offset = 20;
    plots_title_offset = 8;
    j_limit1 = 'arms'; j_limit2 = 'left';
  otherwise
    error('ErrorTEOTraGen:wrongVariableOption', 'Wrong handles.humanoid_part value');
end

for jj = 1:6, 
  axes(handles.(strcat('axes',num2str(jj))))
  cla(handles.(strcat('axes',num2str(jj))),'reset')
  if ~(strcmp(handles.humanoid_part, 'CoM') && (jj > 2)) % Torso only has 2 joints    
    plot(handles.(strcat('axes',num2str(jj))), handles.operational_space_traj.trajectory.time, ...
      handles.joints_space_traj.(traj_type)(jj+traj_element_offset,:));
    hold on
      title(handles.(strcat('axes',char(axes_order(jj)))),plots_titles(jj+plots_title_offset),'color','w')
      xlabel(handles.(strcat('axes',char(axes_order(jj)))),'t [s]')
      ylabel(handles.(strcat('axes',char(axes_order(jj)))),titles_y(jj+titles_y_offset),'color','w');
      if strcmp(traj_type, 'q')
        if strcmp(handles.humanoid_part,'CoM')
          limits =  handles.humanoid_description.(j_limit1).joint(jj).angle_limits;
        else
          limits =  handles.humanoid_description.(j_limit1).(j_limit2).joint(jj).angle_limits;
        end   
        plot(handles.operational_space_traj.trajectory.time,limits(1,1)*ones(1,size(handles.joints_space_traj.(traj_type),2)),'r');
        plot(handles.operational_space_traj.trajectory.time,limits(1,2)*ones(1,size(handles.joints_space_traj.(traj_type),2)),'r');
      end
    hold off
  end
%   plot(handles.(strcat('axes',char(axes_order(jj)))), handles.operational_space_traj.(traj_type).time, ...
%                     handles.operational_space_traj.(traj_type)(jj+traj_element_offset,:));
end
  

function reset_part_button_color(handles)
%Grey: [0.702 0.702 0.702]
set(handles.pushbutton_com,'BackgroundColor', [0.702 0.702 0.702]);
set(handles.pushbutton_right_leg,'BackgroundColor', [0.702 0.702 0.702]);
set(handles.pushbutton_left_leg,'BackgroundColor', [0.702 0.702 0.702]);
set(handles.pushbutton_right_arm,'BackgroundColor', [0.702 0.702 0.702]);
set(handles.pushbutton_left_arm,'BackgroundColor', [0.702 0.702 0.702]);

function reset_space_button_color(handles)
%Grey: [0.702 0.702 0.702]
set(handles.pushbutton_operational,'BackgroundColor', [0.702 0.702 0.702]);
set(handles.pushbutton_joints,'BackgroundColor', [0.702 0.702 0.702]);

function reset_trajectory_button_color(handles)
%Grey: [0.702 0.702 0.702]
set(handles.pushbutton_pose,'BackgroundColor', [0.702 0.702 0.702]);
set(handles.pushbutton_velocity,'BackgroundColor', [0.702 0.702 0.702]);
set(handles.pushbutton_acceleration,'BackgroundColor', [0.702 0.702 0.702]);

function update_button_color(hObject)
%Green: [0 0.6 0]
  set(hObject,'BackgroundColor',[0 0.6 0]);

% --- Outputs from this function are returned to the command line.
function varargout = trajectory_plots_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


function pushbutton_com_Callback(hObject, eventdata, handles)
handles.humanoid_part = 'CoM';
guidata(hObject, handles);
reset_part_button_color(handles)
update_button_color(hObject)
update_plots(handles)


function pushbutton_right_leg_Callback(hObject, eventdata, handles)
handles.humanoid_part = 'RF';
guidata(hObject, handles);
reset_part_button_color(handles)
update_button_color(hObject)
update_plots(handles)


function pushbutton_left_leg_Callback(hObject, eventdata, handles)
handles.humanoid_part = 'LF';
guidata(hObject, handles);
reset_part_button_color(handles)
update_button_color(hObject)
update_plots(handles)


function pushbutton_right_arm_Callback(hObject, eventdata, handles)
handles.humanoid_part = 'RH';
guidata(hObject, handles);
reset_part_button_color(handles)
update_button_color(hObject)
update_plots(handles)


function pushbutton_left_arm_Callback(hObject, eventdata, handles)
handles.humanoid_part = 'LH';
guidata(hObject, handles);
reset_part_button_color(handles)
update_button_color(hObject)
update_plots(handles)


function pushbutton_operational_Callback(hObject, eventdata, handles)
handles.space = 'operational';
guidata(hObject, handles);
set(handles.pushbutton_com,'String','CoM');
reset_space_button_color(handles)
update_button_color(hObject)
update_plots(handles)


function pushbutton_joints_Callback(hObject, eventdata, handles)
handles.space = 'joints';
guidata(hObject, handles);
set(handles.pushbutton_com,'String','Torso');
reset_space_button_color(handles)
update_button_color(hObject)
update_plots(handles)


function pushbutton_pose_Callback(hObject, eventdata, handles)
handles.trajectory = 'pose';
guidata(hObject, handles);
reset_trajectory_button_color(handles)
update_button_color(hObject)
update_plots(handles)


function pushbutton_velocity_Callback(hObject, eventdata, handles)
handles.trajectory = 'velocity';
guidata(hObject, handles);
reset_trajectory_button_color(handles)
update_button_color(hObject)
update_plots(handles)


function pushbutton_acceleration_Callback(hObject, eventdata, handles)
handles.trajectory = 'acceleration';
guidata(hObject, handles);
reset_trajectory_button_color(handles)
update_button_color(hObject)
update_plots(handles)
