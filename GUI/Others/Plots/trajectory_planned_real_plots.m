function varargout = trajectory_planned_real_plots(varargin)
% TRAJECTORY_PLANNED_REAL_PLOTS MATLAB code for trajectory_planned_real_plots.fig
%      TRAJECTORY_PLANNED_REAL_PLOTS, by itself, creates a new TRAJECTORY_PLANNED_REAL_PLOTS or raises the existing
%      singleton*.
%
%      H = TRAJECTORY_PLANNED_REAL_PLOTS returns the handle to a new TRAJECTORY_PLANNED_REAL_PLOTS or the handle to
%      the existing singleton*.
%
%      TRAJECTORY_PLANNED_REAL_PLOTS('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in TRAJECTORY_PLANNED_REAL_PLOTS.M with the given input arguments.
%
%      TRAJECTORY_PLANNED_REAL_PLOTS('Property','Value',...) creates a new TRAJECTORY_PLANNED_REAL_PLOTS or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before trajectory_planned_real_plots_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to trajectory_planned_real_plots_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help trajectory_planned_real_plots

% Last Modified by GUIDE v2.5 10-Feb-2015 18:56:20

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @trajectory_planned_real_plots_OpeningFcn, ...
                   'gui_OutputFcn',  @trajectory_planned_real_plots_OutputFcn, ...
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


% --- Executes just before trajectory_planned_real_plots is made visible.
function trajectory_planned_real_plots_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to trajectory_planned_real_plots (see VARARGIN)

% Choose default command line output for trajectory_planned_real_plots
handles.output = hObject;

if isempty(varargin)
  warning('WarnTEOTraGen:noInputGUI', 'No inputs for trajectory_planned_real_plots');
  handles.operational_space_traj = 0;
  handles.joints_space_traj = 0;
  handles.humanoid_description = 0;
elseif numel(varargin) > 4
  warning('WarnTEOTraGen:wrongInputGUI', 'Unnecessary inputs for trajectory_planned_real_plots');
  handles.operational_space_traj = 0;
  handles.joints_space_traj = 0;
  handles.humanoid_description = 0;
elseif numel(varargin) > 3
  warning('WarnTEOTraGen:wrongInputGUI', 'No enough inputs for trajectory_planned_real_plots');
  handles.operational_space_traj = 0;
  handles.joints_space_traj = 0;
  handles.humanoid_description = 0;
else   
  % Get trajectories data
  handles.planned_traj = varargin{1};
  handles.real_traj = varargin{2};
  handles.humanoid_description = varargin{3};
end

% Show message based on input data
if ~isfield(handles.planned_traj,'trajectory') && ~isfield(handles.real_traj,'trajectory')
  set(handles.message_label,'String', 'TEOTraGen ERROR: No input data. Nothing to plot');
  set(handles.message_label,'BackGroundColor',[1 0.2 0.2]); % Red
elseif isfield(handles.planned_traj,'trajectory') && ~isfield(handles.real_traj,'trajectory')
  set(handles.message_label,'String', 'TEOTraGen WARNING: It will only plot planned operational space trajectories');
  set(handles.message_label,'BackGroundColor',[1 1 0.2]); % Yellow
  set(handles.pushbutton_planned,'Enable','on');
elseif isfield(handles.planned_traj,'trajectory') && isfield(handles.real_traj,'trajectory')
  if ~isfield(handles.humanoid_description,'legs')
    set(handles.message_label,'String', 'TEOTraGen WARNING: It will only plot real operational space trajectories');
    set(handles.message_label,'BackGroundColor',[1 1 0.2]); % Yellow
  end
  set(handles.pushbutton_planned,'Enable','on');
  set(handles.pushbutton_real,'Enable','on');
else
  error('ErrorTEOTraGen:wrongInputGUI', 'Wrong inputs option');
end

% Default plotting values
handles.show_planned = 1;
handles.show_real = 0;
handles.humanoid_part = 'CoM';
handles.trajectory = 'pose';
handles.plot_matrix = zeros(2,6);

guidata(hObject, handles);
update_button_color(handles.pushbutton_planned);
update_button_color(handles.pushbutton_com);
update_button_color(handles.pushbutton_pose);

handles = init_plots(hObject, handles);

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes trajectory_planned_real_plots wait for user response (see UIRESUME)
% uiwait(handles.figure1);

function handles = init_plots(hObject, handles)
message = ['TEOTraGen INFO: Plotting Trajectories ' handles.trajectory 'trajectory for ' handles.humanoid_part ' part ...'];
set(handles.message_label,'String', message);
set(handles.message_label,'BackGroundColor',[1 1 0.2]); % Yellow

% Plot trajectories
handles = plot_operational(handles);
handles = filter_plots(handles);
  
message = ['TEOTraGen INFO: ' handles.trajectory ' for ' handles.humanoid_part ' part ... OK'];
set(handles.message_label,'String', message);
set(handles.message_label,'BackGroundColor',[0.702 0.78 1]); % Blue

function handles = plot_operational(handles)
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
  handles.plot_matrix(1,jj) = plot(handles.(strcat('axes',char(axes_order(jj)))), handles.planned_traj.(traj_type).time, ...
                        handles.planned_traj.(traj_type).(handles.humanoid_part)(jj,:));
  hold on;
  handles.plot_matrix(2,jj) = plot(handles.(strcat('axes',char(axes_order(jj)))), handles.real_traj.(traj_type).time, ...
                        handles.real_traj.(traj_type).(handles.humanoid_part)(jj,:),'r');
  title(handles.(strcat('axes',char(axes_order(jj)))),plots_titles(jj))
  xlabel(handles.(strcat('axes',char(axes_order(jj)))),'t [s]')
  ylabel(handles.(strcat('axes',char(axes_order(jj)))),titles_y(jj+titles_y_offset))
  hold off;
end

function handles = filter_plots(handles)
if ~handles.show_planned
  for jj = 1:6,
    set(handles.plot_matrix(1,jj),'Visible','off');  %# Make it invisible
  end
else
  for jj = 1:6,
    set(handles.plot_matrix(1,jj),'Visible','on');   %# Make it visible
  end
end
if ~handles.show_real
  for jj = 1:6,
    set(handles.plot_matrix(2,jj),'Visible','off');  %# Make it invisible
  end
else
  for jj = 1:6,
    set(handles.plot_matrix(2,jj),'Visible','on');   %# Make it visible
  end
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
set(handles.pushbutton_planned,'BackgroundColor', [0.702 0.702 0.702]);
set(handles.pushbutton_real,'BackgroundColor', [0.702 0.702 0.702]);

function reset_trajectory_button_color(handles)
%Grey: [0.702 0.702 0.702]
set(handles.pushbutton_pose,'BackgroundColor', [0.702 0.702 0.702]);
set(handles.pushbutton_velocity,'BackgroundColor', [0.702 0.702 0.702]);
set(handles.pushbutton_acceleration,'BackgroundColor', [0.702 0.702 0.702]);

function update_button_color(hObject)
%Green: [0 0.6 0]
set(hObject,'BackgroundColor',[0 0.6 0]);

function update_real_button_color(hObject, handles)
if handles.show_real, %Green: [0 0.6 0]
  set(hObject,'BackgroundColor',[0 0.6 0]);
else %Grey: [0.702 0.702 0.702]
  set(hObject,'BackgroundColor', [0.702 0.702 0.702]);
end

function update_planned_button_color(hObject, handles)
if handles.show_planned, %Green: [0 0.6 0]
  set(hObject,'BackgroundColor',[0 0.6 0]);
else %Grey: [0.702 0.702 0.702]
  set(hObject,'BackgroundColor', [0.702 0.702 0.702]);
end
  
  
% --- Outputs from this function are returned to the command line.
function varargout = trajectory_planned_real_plots_OutputFcn(hObject, eventdata, handles) 
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
handles = init_plots(hObject, handles);
guidata(hObject, handles);


function pushbutton_right_leg_Callback(hObject, eventdata, handles)
handles.humanoid_part = 'RF';
guidata(hObject, handles);
reset_part_button_color(handles)
update_button_color(hObject)
handles = init_plots(hObject, handles);
guidata(hObject, handles);


function pushbutton_left_leg_Callback(hObject, eventdata, handles)
handles.humanoid_part = 'LF';
guidata(hObject, handles);
reset_part_button_color(handles)
update_button_color(hObject)
handles = init_plots(hObject, handles);
guidata(hObject, handles);


function pushbutton_right_arm_Callback(hObject, eventdata, handles)
handles.humanoid_part = 'RH';
guidata(hObject, handles);
reset_part_button_color(handles)
update_button_color(hObject)
handles = init_plots(hObject, handles);
guidata(hObject, handles);


function pushbutton_left_arm_Callback(hObject, eventdata, handles)
handles.humanoid_part = 'LH';
guidata(hObject, handles);
reset_part_button_color(handles)
update_button_color(hObject)
handles = init_plots(hObject, handles);
guidata(hObject, handles);


function pushbutton_planned_Callback(hObject, eventdata, handles)
handles.show_planned = ~handles.show_planned;
guidata(hObject, handles);

update_planned_button_color(hObject, handles)
filter_plots(handles);


function pushbutton_real_Callback(hObject, eventdata, handles)
handles.show_real = ~handles.show_real;
guidata(hObject, handles);

update_real_button_color(hObject, handles)
filter_plots(handles);


function pushbutton_pose_Callback(hObject, eventdata, handles)
handles.trajectory = 'pose';
guidata(hObject, handles);
reset_trajectory_button_color(handles)
update_button_color(hObject)
handles = init_plots(hObject, handles);
guidata(hObject, handles);





function pushbutton_velocity_Callback(hObject, eventdata, handles)
handles.trajectory = 'velocity';
guidata(hObject, handles);
reset_trajectory_button_color(handles)
update_button_color(hObject)
handles = init_plots(hObject, handles);
guidata(hObject, handles);


function pushbutton_acceleration_Callback(hObject, eventdata, handles)
handles.trajectory = 'acceleration';
guidata(hObject, handles);
reset_trajectory_button_color(handles)
update_button_color(hObject)
handles = init_plots(hObject, handles);
guidata(hObject, handles);


function pushbutton_open_axes1_Callback(hObject, eventdata, handles)
h2copy = allchild(handles.axes1);
figure;
hParent = axes;
copyobj(h2copy, hParent)

function pushbutton_open_axes2_Callback(hObject, eventdata, handles)
h2copy = allchild(handles.axes2);
figure;
hParent = axes;
copyobj(h2copy, hParent)

function pushbutton_open_axes3_Callback(hObject, eventdata, handles)
h2copy = allchild(handles.axes3);
figure;
hParent = axes;
copyobj(h2copy, hParent)

function pushbutton_open_axes4_Callback(hObject, eventdata, handles)
h2copy = allchild(handles.axes4);
figure;
hParent = axes;
copyobj(h2copy, hParent)

function pushbutton_open_axes5_Callback(hObject, eventdata, handles)
h2copy = allchild(handles.axes5);
figure;
hParent = axes;
copyobj(h2copy, hParent)

function pushbutton_open_axes6_Callback(hObject, eventdata, handles)
h2copy = allchild(handles.axes6);
figure;
hParent = axes;
copyobj(h2copy, hParent)