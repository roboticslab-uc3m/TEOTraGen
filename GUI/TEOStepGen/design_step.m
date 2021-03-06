function varargout = design_step(varargin)
% DESIGN_STEP GUI
%
%   This GUI allows user edit the variation of CoM, Right Foot, Left Foot,
%   Right Arm and Left Arm. The user can also choose the type of
%   interpolation.
%
%   Author: Domingo Esteban
%   RoboticsLab - Universidad Carlos III de Madrid
%   $Revision: 1.0 $  $Date: 2013/08/05 $

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @design_step_OpeningFcn, ...
                   'gui_OutputFcn',  @design_step_OutputFcn, ...
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

% --- Executes just before design_step is made visible.
function design_step_OpeningFcn(hObject, eventdata, handles, varargin)
  global data

  % Hide plot_options panel
  set(handles.plot_options, 'Visible', 'Off')

  % Choose default command line output for design_step
  handles.output = hObject;

  % Assign Input Data
  if isempty(varargin)

    % Get Data
    handles.GUIConfig = TEOStepGenConfig();

    % Check Values of the Configuration File
    if isfield(handles.GUIConfig, 'q0'),
      handles.Input_data.q0 = handles.GUIConfig.q0;
    else
      handles.Input_data.q0 =   [ ...
                                  % Right Leg
                                  0;                    ... % Right Hip Yaw
                                  0;                    ... % Right Hip Roll
                                  -0.2701;              ... % Right Hip Pitch
                                  0.5680;               ... % Right Knee Pitch
                                  -0.2979;              ... % Right Ankle Pitch
                                  0;                    ... % Right Ankle Roll
                                  % Left Leg 
                                  0;                    ... % Left Hip Yaw
                                  0;                    ... % Left Hip Roll 
                                  -0.2701;              ... % Left Hip Pitch
                                  0.5680;               ... % Left Knee Pitch
                                  -0.2979;              ... % Left Ankle Pitch
                                  0;                    ... % Left Ankle Roll
                                  % Torso
                                  0;                    ... % Torso Yaw
                                  0;                    ... % Torso Pitch
                                  % Right Arm
                                  0.420000000000000;    ... % Right Shoulder Pitch
                                  -0.167017153300893;   ... % Right Shoulder Roll
                                  0;                    ... % Right Shoulder Yaw
                                  -1.250000000000000;   ... % Right Elbow Pitch
                                  0;                    ... % Right Wrist Yaw
                                  0;                    ... % Right Wrist Pitch
                                  % Left Arm
                                  0.420000000000000;    ... % Left Shoulder Pitch
                                  0.167017153300893;    ... % Left Shoulder Roll
                                  0;                    ... % Left Shoulder Yaw
                                  -1.250000000000000;   ... % Left Elbow Pitch
                                  0;                    ... % Left Wrist Yaw
                                  0                     ... % Left Wrist Pitch
                                  ];
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
      handles.Input_data.L_val = 0.1;
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
      handles.Input_data.kp = handles.GUIConfig.kp;
    else
%       handles.Input_data.kp = 0.01;
      handles.Input_data.kp = 20;
    end
    
    if isfield(handles.GUIConfig, 'ko'),
      handles.Input_data.ko = handles.GUIConfig.ko;
    else
%       handles.Input_data.ko = pi/8;
      handles.Input_data.ko = 1;
    end
        
    
  else
  	handles.Input_data = varargin{:};
  end

  % Define some default values
  handles.interpola_DS1     = 'Polynomial5';
  handles.interpola_DS2     = 'Polynomial5';
  handles.interpola_SS_com  = 'Polynomial5';
  handles.interpola_SS_ff   = 'Polynomial5';
  handles.interpola_RH      = 'Polynomial5';
  handles.interpola_LH      = 'Polynomial5';
  
  handles.result = 0;

  % Load TEO Kinematics Library and Structure
  handles.h   = TEO_kinematics_library;
  handles.humanoid_structure = TEO_structure('numeric', 'rad', 'm');
  data = []; % Its value change when a step is generated
  
  % Update handles structure
  guidata(hObject, handles);

  % Update window title
  set(handles.text_title, 'String', strcat('DESIGN STEP - ', ' ', handles.Input_data.SupportLeg, ' Support'));
  
  % Assign support_foot variable
  switch handles.Input_data.SupportLeg
    case 'Right' % Support on right foot
      support_foot = 'RF';
  
    case 'Right Leg'
      support_foot = 'RF';
      
    case 'Left' % Support on left foot
      support_foot = 'LF';
    
    case 'Left Leg'
      support_foot = 'LF';
      
    otherwise
      error('TEOStepGen:Error','Wrong support_foot option');
      
  end
  handles.support_foot = support_foot;
  
  % Update handles structure
  guidata(hObject, handles);

  % Pre-evaluate delta x and delta y CoM in Double Support
  pre_evaluate_delta_com_ds(hObject, handles);

  % Pre-evaluate foot deltas in Simple and Double Support
  pre_evaluate_delta_ss(hObject, handles);

  % Pre-evaluate delta x and delta y Arm in Simple Support
  pre_evaluate_delta_arm_ss(hObject, handles);

  
  % Plots panel
  graphstab = uitabpanel(...
    'Parent',handles.panel_graphics,...
    'Style','popup',...
    'Units','normalized',...
    'Position',[0,0,1,1],...
    'FrameBackgroundColor',[0.078,0.169,0.549],...
    'FrameBorderType','etchedin',...
    'Title',{'Right Leg','Left Leg','Torso','Right Arm','Left Arm'},...%,'CoM'},...
    'PanelHeights',[59.5,59.5,50,59.5,59.5],...%,50],...
    'HorizontalAlignment','left',...
    'FontWeight','bold',...
    'TitleBackgroundColor',[0.078,0.169,0.549],...
    'TitleForegroundColor',[1 1 1],...
    'PanelBackgroundColor',[0.702,0.78,1],...
    'PanelBorderType','line');%,'SelectedItem',1);

  hpanel = getappdata(graphstab,'panels');

	handles.sd_graph = axes('Parent',getappdata(graphstab,'status'), ...
                          'Units','normalized',...
                          'Position',[0.3,0.1,0.4,0.9],'Box','on'); 
  
  % Right leg joints
  handles.axes1 = axes('Parent',hpanel(1),'Position',[.1 .6 .25 .25]);
  handles.axes2 = axes('Parent',hpanel(1),'Position',[.4 .6 .25 .25]);
  handles.axes3 = axes('Parent',hpanel(1),'Position',[.7 .6 .25 .25]);
  handles.axes4 = axes('Parent',hpanel(1),'Position',[.1 .15 .25 .25]);
  handles.axes5 = axes('Parent',hpanel(1),'Position',[.4 .15 .25 .25]);
  handles.axes6 = axes('Parent',hpanel(1),'Position',[.7 .15 .25 .25]);
   
  % Left leg joints
  handles.axes7 = axes('Parent',hpanel(2),'Position',[.1 .6 .25 .25]);
  handles.axes8 = axes('Parent',hpanel(2),'Position',[.4 .6 .25 .25]);
  handles.axes9 = axes('Parent',hpanel(2),'Position',[.7 .6 .25 .25]);
  handles.axes10 = axes('Parent',hpanel(2),'Position',[.1 .15 .25 .25]);
  handles.axes11 = axes('Parent',hpanel(2),'Position',[.4 .15 .25 .25]);
  handles.axes12 = axes('Parent',hpanel(2),'Position',[.7 .15 .25 .25]);
   
  % Torso joints
  handles.axes13 = axes('Parent',hpanel(3),'Position',[.1 .6 .25 .25]);
  handles.axes14 = axes('Parent',hpanel(3),'Position',[.7 .6 .25 .25]);   
   
  % Right arm joints
  handles.axes15 = axes('Parent',hpanel(4),'Position',[.1 .6 .25 .25]);
  handles.axes16 = axes('Parent',hpanel(4),'Position',[.4 .6 .25 .25]);
  handles.axes17 = axes('Parent',hpanel(4),'Position',[.7 .6 .25 .25]);
  handles.axes18 = axes('Parent',hpanel(4),'Position',[.1 .15 .25 .25]);
  handles.axes19 = axes('Parent',hpanel(4),'Position',[.4 .15 .25 .25]);
  handles.axes20 = axes('Parent',hpanel(4),'Position',[.7 .15 .25 .25]);
   
  % Left arm joints
  handles.axes21 = axes('Parent',hpanel(5),'Position',[.1 .6 .25 .25]);
  handles.axes22 = axes('Parent',hpanel(5),'Position',[.4 .6 .25 .25]);
  handles.axes23 = axes('Parent',hpanel(5),'Position',[.7 .6 .25 .25]);
  handles.axes24 = axes('Parent',hpanel(5),'Position',[.1 .15 .25 .25]);
  handles.axes25 = axes('Parent',hpanel(5),'Position',[.4 .15 .25 .25]);
  handles.axes26 = axes('Parent',hpanel(5),'Position',[.7 .15 .25 .25]);

  % CoM value
  % handles.axes_zmp = axes('Parent',hpanel(6),'Position',[.2 .2 .6 .6]);
  % handles.axes_robot_plot = axes('Parent',hpanel(6),'Position',[.2 .2 .6 .6]);
   
  % Axes to TEO picture visualization
  axes(handles.sd_graph)
  [r,map] = imread('TEO-face-background.png','png');
  image(r); colormap(map); 
  text(150,200,'Please, click the humanoid part','HorizontalAlignment','center','Color','w','BackgroundColor',[.0 .0 .0]);
  text(150,250,'to display/hide the joint plots','HorizontalAlignment','center','Color','w','BackgroundColor',[.0 .0 .0]);
  axis off

  % Update handles structure
  guidata(hObject, handles);
  % UIWAIT makes design_step wait for user response (see UIRESUME)
  % uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = design_step_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


function interpolaDS1_Callback(hObject, eventdata, handles)
contents = cellstr(get(hObject,'String'));
handles.interpola_DS1 = contents{get(hObject,'Value')};
guidata(hObject,handles)

function interpolaDS1_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function delta_x_CoM_DS1_Callback(hObject, eventdata, handles)

function delta_x_CoM_DS1_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function delta_y_CoM_DS1_Callback(hObject, eventdata, handles)

function delta_y_CoM_DS1_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function delta_z_CoM_DS1_Callback(hObject, eventdata, handles)

function delta_z_CoM_DS1_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function pre_evaluate_delta_com_ds(hObject, handles)
% PRE_EVALUATE_DELTA_COM_DS Pre-evaluate delta x and delta y CoM in Double
% Support. The CoM is translated to the center of the support foot in
% CoM_DS1 phase. The opposite translation is realized in the CoM_DS2 phase.

support_foot = handles.support_foot;
h = handles.h;
TEO = handles.humanoid_structure;
q = handles.Input_data.q0;

switch support_foot
  case 'RF' % Support on right foot
    delta = h.CoM_T_RF(q);
%     delta_P(1,:) = (2*delta(1) + (TEO.legs.right.foot.limits.x(1) + TEO.legs.right.foot.limits.x(2)));
%     delta_P(2,:) = (2*delta(2) + (TEO.legs.right.foot.limits.y(1) + TEO.legs.right.foot.limits.y(2)));
%     delta_P(1,:) = delta(1) + TEO.legs.right.foot.limits.x(1) + TEO.legs.right.foot.limits.x(2);
    delta_P(2,:) = delta(2) + TEO.legs.right.foot.limits.y(1) + TEO.legs.right.foot.limits.y(2);

  case 'LF' % Support on left foot
    delta = h.CoM_T_LF(q);
%     delta_P(1,:) = (2*delta(1) + (TEO.legs.left.foot.limits.x(1) + TEO.legs.left.foot.limits.x(2)));
%     delta_P(2,:) = (2*delta(2) + (TEO.legs.left.foot.limits.y(1) + TEO.legs.left.foot.limits.y(2)));
%     delta_P(1,:) = delta(1) + TEO.legs.left.foot.limits.x(1) + TEO.legs.left.foot.limits.x(2);
    delta_P(2,:) = delta(2) + TEO.legs.left.foot.limits.y(1) + TEO.legs.left.foot.limits.y(2);

end

% delta_P = delta_P/2;
% set(handles.delta_x_CoM_DS1,'String',num2str(delta_P(1)),'BackgroundColor','red');
set(handles.delta_y_CoM_DS1,'String',num2str(delta_P(2)),'BackgroundColor','red');
% set(handles.delta_x_CoM_DS2,'String',num2str(-delta_P(1)),'BackgroundColor','red');
set(handles.delta_y_CoM_DS2,'String',num2str(-delta_P(2)),'BackgroundColor','red');

guidata(hObject, handles)


function pre_evaluate_delta_ss(hObject, handles)
% PRE_EVALUATE_DELTA_SS Pre-evaluate delta x, delta y, and delta z of CoM
% in Single Support.

L = handles.Input_data.L_val; % Floating foot X variation
H = handles.Input_data.H_val; % Floating foot Z variation
gamma_com = handles.Input_data.gamma_com; % Percentage of X variation fue to CoM variation

X1_SF = L*gamma_com/2;
set(handles.delta_x_CoM_SS1,'String',num2str(X1_SF),'BackgroundColor','red');
X2_SF = L*gamma_com;
set(handles.delta_x_CoM_SS2,'String',num2str(X2_SF),'BackgroundColor','red');

X1_FF = L*(1-gamma_com)/2;
set(handles.delta_x_FF_SS1,'String',num2str(X1_FF),'BackgroundColor','red');
set(handles.delta_z_FF_SS1,'String',num2str(H),'BackgroundColor','red');

X2_FF = L*(1-gamma_com);
set(handles.delta_x_FF_SS2,'String',num2str(X2_FF),'BackgroundColor','red');
guidata(hObject, handles)


function pre_evaluate_delta_arm_ss(hObject, handles)

support_foot = handles.support_foot;

% Get floating foot deltas
floating_delta_x = str2double(get(handles.delta_x_FF_SS2,'String'));

if strcmp(support_foot,'RF')
  set(handles.delta_x_RH,'String',num2str(floating_delta_x),'BackgroundColor','red');
  set(handles.delta_x_LH,'String',num2str(-floating_delta_x),'BackgroundColor','red');
else
  set(handles.delta_x_RH,'String',num2str(-floating_delta_x),'BackgroundColor','red');
  set(handles.delta_x_LH,'String',num2str(floating_delta_x),'BackgroundColor','red');  
end
guidata(hObject, handles)



function pushbutton_generate_step_Callback(hObject, eventdata, handles)
global q dq ddq trajectory d_trajectory dd_trajectory
global data

h = handles.h;

% Wait bar
waitbar1 = waitbar(0,'Please wait...');

% Steps Data
if isempty(data)
  data.t0 = 0;                                    % Initial time
  data.Tend = data.t0 + handles.Input_data.T_val;
  data.q = handles.Input_data.q0;
  data.nsteps = 0;
end

data.Ts = handles.Input_data.Ts_val;            % Sampling Time
data.q0 = handles.Input_data.q0;
data.T = handles.Input_data.T_val;              % Total time. Time of the step

data.alpha_ds = handles.Input_data.alpha_ds;    % Percentage of the total time for double support
data.gamma_com = handles.Input_data.gamma_com;  % Percentage of the total time for support foot ???
data.L = handles.Input_data.L_val;              % Length of the step (X direction)
data.H = handles.Input_data.H_val;              % Height of the step (Z direction)

data.kp = handles.Input_data.kp;
data.ko = handles.Input_data.ko;

    
% Delta Data
global delta
  delta.delta_CoM_DS1 = [str2double(get(handles.delta_x_CoM_DS1,'String'));...    % Variation of the CoM in the Double Support phase 1
      str2double(get(handles.delta_y_CoM_DS1,'String'));... 
      str2double(get(handles.delta_z_CoM_DS1,'String'));...
      zeros(2,1);...
      str2double(get(handles.delta_yaw_CoM_DS1,'String'))];
  delta.interpola_CoM_DS1 = handles.interpola_DS1;                                % Interpolation for the CoM in the Double Support phase 1

  delta.delta_CoM_SS1 = [str2double(get(handles.delta_x_CoM_SS1,'String'));...    % Variation of the CoM in the Simple Support phase 1
      str2double(get(handles.delta_y_CoM_SS1,'String'));...
      str2double(get(handles.delta_z_CoM_SS1,'String'));
      zeros(2,1);...
      str2double(get(handles.delta_yaw_CoM_SS1,'String'))];
  delta.delta_CoM_SS2 = [str2double(get(handles.delta_x_CoM_SS2,'String'));...    % Variation of the CoM in the Simple Support phase 2
      str2double(get(handles.delta_y_CoM_SS2,'String'));...
      str2double(get(handles.delta_z_CoM_SS2,'String'));
      zeros(2,1);...
      str2double(get(handles.delta_yaw_CoM_SS2,'String'))];
  delta.interpola_CoM_SS = handles.interpola_SS_com;                              % Interpolation for the CoM in the Simple Support phase (1 and 2)

  delta.delta_FF_SS1 = [str2double(get(handles.delta_x_FF_SS1,'String'));...      % Variation of the Floating Foot in the Simple Support phase 1  
      str2double(get(handles.delta_y_FF_SS1,'String'));...
      str2double(get(handles.delta_z_FF_SS1,'String'));
      zeros(2,1);...
      str2double(get(handles.delta_yaw_FF_SS1,'String'))];
    
  delta.delta_FF_SS2 = [str2double(get(handles.delta_x_FF_SS2,'String'));...      % Variation of the Floating Foot in the Simple Support phase 1  
      str2double(get(handles.delta_y_FF_SS2,'String'));...
      str2double(get(handles.delta_z_FF_SS2,'String'));...
      zeros(2,1);...
      str2double(get(handles.delta_yaw_FF_SS2,'String'))];
  delta.interpola_FF_SS = handles.interpola_SS_ff;                                % Interpolation for the Floating Foot in the Simple Support phase (1 and 2)

  delta.delta_CoM_DS2 = [str2double(get(handles.delta_x_CoM_DS2,'String'));...    % Variation of the CoM in the Double Support phase 2
      str2double(get(handles.delta_y_CoM_DS2,'String'));... 
      str2double(get(handles.delta_z_CoM_DS2,'String'));
      zeros(2,1);...
      str2double(get(handles.delta_yaw_CoM_DS2,'String'))];
  delta.interpola_CoM_DS2 = handles.interpola_DS2;                                % Interpolation for the CoM in the Double Support phase 2

  delta.delta_RH = [str2double(get(handles.delta_x_RH,'String'));...              % Variation of the Right Hand in the DS and SS phase
      str2double(get(handles.delta_y_RH,'String'));...
      str2double(get(handles.delta_z_RH,'String'));zeros(3,1)];
  delta.interpola_RH = handles.interpola_RH;                                      % Interpolation for the Right Hand in the DS and SS phase

  delta.delta_LH = [str2double(get(handles.delta_x_LH,'String'));...              % Variation of the Left Hand in the DS and SS phase
      str2double(get(handles.delta_y_LH,'String'));...
      str2double(get(handles.delta_z_LH,'String'));zeros(3,1)];
  delta.interpola_LH = handles.interpola_LH;                                      % Interpolation for the Left Hand in the DS and SS phase


waitbar(0.1)

% Support Foot
support_foot = handles.support_foot;
    

% Create trajectory structures if there isn't
if ~isstruct(handles.result)
  % CREATE TRAJECTORY
    % Fields corresponding to the operational space of TEO
    TEO_fields = humanoid_operational_fields();

    % Creates the trajectories structures
    trajectory = create_trajectory_template (TEO_fields, data.Ts);
    d_trajectory = create_trajectory_template (TEO_fields, data.Ts);
    dd_trajectory = create_trajectory_template (TEO_fields, data.Ts);
else
  trajectory = handles.result.trajectory;
  d_trajectory = handles.result.d_trajectory;
  dd_trajectory = handles.result.dd_trajectory;
end

waitbar(0.2)


% First step has as X Delta the half of a complete step
if data.nsteps == 0 
  delta.delta_LH(1) = delta.delta_LH(1)/2;
  delta.delta_RH(1) = delta.delta_RH(1)/2;
  delta.delta_CoM_DS1(1) = delta.delta_CoM_DS1(1)/2;
  delta.delta_CoM_DS2(1) = delta.delta_CoM_DS2(1)/2;
  delta.delta_CoM_SS1(1) = delta.delta_CoM_SS1(1)/2;
  delta.delta_CoM_SS2(1) = delta.delta_CoM_SS2(1)/2;
  delta.delta_FF_SS1(1) = delta.delta_FF_SS1(1)/2;
  delta.delta_FF_SS2(1) = delta.delta_FF_SS2(1)/2;
end

% Double Step and Simple Step for TEO Robot

[q, dq, ddq, trajectory, d_trajectory, dd_trajectory] = ds_ss_step_TEO(h, delta, data, support_foot, trajectory, d_trajectory, dd_trajectory);


waitbar(0.7)

% borrar el grafico actual de axes_zmp (imagen de TEO):
% cla(handles.axes_zmp)

handles.result.q = q;
handles.result.dq = dq;
handles.result.ddq = ddq;
handles.result.trajectory = trajectory;
handles.result.d_trajectory = d_trajectory;
handles.result.dd_trajectory = dd_trajectory;

waitbar(0.8)

% Show plots
set(handles.plot_options, 'Visible', 'On')
plot_all_graphs(hObject,handles,'PLOT SPACE') % Show joints space by default

waitbar(0.9)

% Update number of steps
data.nsteps = data.nsteps + 1;

% Disable generate_step button
disable_generate_step_button(hObject, eventdata, handles)

waitbar(1)
% Close wait bar when the generation has finished
close(waitbar1)

% Update handles structure
guidata(hObject, handles);


function disable_generate_step_button(hObject, eventdata, handles)
set(handles.pushbutton_new_step,'Enable','On')
set(handles.pushbutton_reset_trajectory,'Enable','On')
set(handles.pushbutton_matlab_vis,'Enable','On')
set(handles.pushbutton_ros_vis,'Enable','On')
set(handles.pushbutton_generate_step,'Enable','Off')
update_nsteps(hObject, eventdata, handles)


function enable_generate_step_button(hObject, eventdata, handles)
set(handles.pushbutton_new_step,'Enable','Off')
set(handles.pushbutton_reset_trajectory,'Enable','Off')
set(handles.pushbutton_matlab_vis,'Enable','Off')
set(handles.pushbutton_ros_vis,'Enable','Off')
set(handles.pushbutton_generate_step,'Enable','On')
update_nsteps(hObject, eventdata, handles)

function update_nsteps(hObject, eventdata, handles)
global data
if isstruct(data)
  nsteps = data.nsteps;
else
  nsteps = 0;
end
set(handles.edit_nsteps,'String',num2str(nsteps))


function interpolaSSff_Callback(hObject, eventdata, handles)
contents = cellstr(get(hObject,'String'));
handles.interpola_SS_ff = contents{get(hObject,'Value')};
guidata(hObject,handles)

function interpolaSSff_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function delta_x_FF_SS2_Callback(hObject, eventdata, handles)

function delta_x_FF_SS2_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function delta_y_FF_SS2_Callback(hObject, eventdata, handles)

function delta_y_FF_SS2_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function delta_z_FF_SS2_Callback(hObject, eventdata, handles)

function delta_z_FF_SS2_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function delta_x_CoM_SS2_Callback(hObject, eventdata, handles)

function delta_x_CoM_SS2_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function delta_y_CoM_SS2_Callback(hObject, eventdata, handles)

function delta_y_CoM_SS2_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function delta_z_CoM_SS2_Callback(hObject, eventdata, handles)

function delta_z_CoM_SS2_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function delta_x_FF_SS1_Callback(hObject, eventdata, handles)

function delta_x_FF_SS1_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function delta_y_FF_SS1_Callback(hObject, eventdata, handles)

function delta_y_FF_SS1_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function delta_z_FF_SS1_Callback(hObject, eventdata, handles)

function delta_z_FF_SS1_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function interpolaSScom_Callback(hObject, eventdata, handles)

function interpolaSScom_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function delta_x_CoM_SS1_Callback(hObject, eventdata, handles)

function delta_x_CoM_SS1_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function delta_y_CoM_SS1_Callback(hObject, eventdata, handles)

function delta_y_CoM_SS1_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function delta_z_CoM_SS1_Callback(hObject, eventdata, handles)

function delta_z_CoM_SS1_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function delta_x_RH_Callback(hObject, eventdata, handles)

function delta_x_RH_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function delta_y_RH_Callback(hObject, eventdata, handles)

function delta_y_RH_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function delta_z_RH_Callback(hObject, eventdata, handles)

function delta_z_RH_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function interpolaLH_Callback(hObject, eventdata, handles)
contents = cellstr(get(hObject,'String'));
handles.interpola_LH = contents{get(hObject,'Value')};
guidata(hObject,handles)

function interpolaLH_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function delta_x_LH_Callback(hObject, eventdata, handles)

function delta_x_LH_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function delta_y_LH_Callback(hObject, eventdata, handles)

function delta_y_LH_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function delta_z_LH_Callback(hObject, eventdata, handles)

function delta_z_LH_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function interpolaRH_Callback(hObject, eventdata, handles)
contents = cellstr(get(hObject,'String'));
handles.interpola_RH = contents{get(hObject,'Value')};

guidata(hObject,handles)


function interpolaRH_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function plot_options_SelectionChangeFcn(hObject, eventdata, handles)
  handles.plot_new = get(eventdata.NewValue,'String');
  old_val = get(eventdata.OldValue,'String');

  for jj = 1:6
    cla(handles.(strcat('axes',num2str(jj))))
    cla(handles.(strcat('axes',num2str(jj+6))))

  end
  for jj = 1:5
    cla(handles.(strcat('axes',num2str(jj+13))))
    cla(handles.(strcat('axes',num2str(jj+18)))) 
  end

  plot_all_graphs(hObject,handles,handles.plot_new)

  guidata(hObject,handles)


function pushbutton3_Callback(hObject, eventdata, handles)

function pushbutton_matlab_vis_Callback(hObject, eventdata, handles)
global trajectory q
matlab_visualization (trajectory, q, handles.humanoid_structure, handles.h)


% --------------------------------------------------------------------
function menu_steps_control_Callback(hObject, eventdata, handles)
steps_control
guidata(hObject,handles)
close(handles.figure1)


% --------------------------------------------------------------------
function menu_main_TEO_Callback(hObject, eventdata, handles)
  TEOTraGen
  guidata(hObject,handles)
  close(handles.figure1)


% --------------------------------------------------------------------
function Untitled_3_Callback(hObject, eventdata, handles)


% --------------------------------------------------------------------
function Untitled_8_Callback(hObject, eventdata, handles)


function change_parameters_Callback(hObject, eventdata, handles)

function ik_gains_Callback(hObject, eventdata, handles)
% Change the values of gains
[handles.Input_data.kp handles.Input_data.ko] = change_ik_gains(handles.Input_data.kp, handles.Input_data.ko);
guidata(hObject,handles);


function initial_configuration_Callback(hObject, eventdata, handles)
handles.Input_data.q0  = change_configuration(handles.Input_data.q0, handles.humanoid_structure);
guidata(hObject, handles);


function interpolaDS2_Callback(hObject, eventdata, handles)
contents = cellstr(get(hObject,'String'));
handles.interpola_DS2 = contents{get(hObject,'Value')};
guidata(hObject,handles)


function interpolaDS2_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function delta_x_CoM_DS2_Callback(hObject, eventdata, handles)

function delta_x_CoM_DS2_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function delta_y_CoM_DS2_Callback(hObject, eventdata, handles)

function delta_y_CoM_DS2_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function delta_z_CoM_DS2_Callback(hObject, eventdata, handles)

function delta_z_CoM_DS2_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function pushbutton_ros_vis_Callback(hObject, eventdata, handles)
if isstruct(handles.result)
  ros_visualization(handles.result.q, handles.result.trajectory.SF, handles.Input_data.Ts_val);
else
  errordlg('There is not any Joint Angles data to visualize in ROS','ROS visualization Error')
  return
end

function pushbutton_reset_trajectory_Callback(hObject, eventdata, handles)
global data
if isstruct(handles.result)
  choice = questdlg('Do you want to delete the current trajectory?', ...
    'Reset Trajectory', ...
    'Yes','No','No');
  % Handle response
  if strcmp(choice,'Yes')
    % Reset data
    handles.result = 0;
    data = [];
    % Disable other buttons
    enable_generate_step_button(hObject, eventdata, handles)
  else
    return;
  end
else
  errordlg('There is not any trajectory data to reset','Reset Trajectory Error')
  return
end

guidata(hObject,handles)

function pushbutton_new_step_Callback(hObject, eventdata, handles)
global data
if isstruct(handles.result)
  q_end = handles.result.q(:,end);
  % Change support_foot
  if strcmp(handles.support_foot, 'RF')
    support_foot = 'LF';
    Leg = 'Left';
  else
    support_foot = 'RF';
    Leg = 'Right';
  end

  handles.support_foot = support_foot;
  
  % Update handles structure
  guidata(hObject, handles);
  
  pre_evaluate_delta_com_ds(hObject, handles);
  
  pre_evaluate_delta_arm_ss(hObject, handles);
  
  % Change Yaw rotation of CoM, the inverse than previous step
  set(handles.delta_yaw_CoM_DS1,'String', num2str(-str2num(get(handles.delta_yaw_CoM_DS1,'String'))));
  set(handles.delta_yaw_CoM_DS2,'String', num2str(-str2num(get(handles.delta_yaw_CoM_DS2,'String'))));
  set(handles.delta_yaw_CoM_SS1,'String', num2str(-str2num(get(handles.delta_yaw_CoM_SS1,'String'))));
  set(handles.delta_yaw_CoM_SS2,'String', num2str(-str2num(get(handles.delta_yaw_CoM_SS2,'String'))));
  set(handles.delta_yaw_FF_SS1,'String', num2str(-str2num(get(handles.delta_yaw_FF_SS1,'String'))));
  set(handles.delta_yaw_FF_SS2,'String', num2str(-str2num(get(handles.delta_yaw_FF_SS2,'String'))));
  
  
  if ~isempty(data)
    data.t0 = data.Tend; % Initial time
    data.Tend = data.t0 + data.T;
%     data.q = q_end;
  end
  
  % Update title
  set(handles.text_title,'String',strcat('DESIGN STEP - ', ' ', Leg,' Support'));
  
  %Update handles field
  handles.support_foot = support_foot;
else
  errordlg('There is not any trajectory data to add a step','Add Step Error')
  return
end

% Enable generate_step_button
enable_generate_step_button(hObject, eventdata, handles)

guidata(hObject,handles)



function edit_nsteps_Callback(hObject, eventdata, handles)

function edit_nsteps_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --------------------------------------------------------------------
function menu_others_Callback(hObject, eventdata, handles)
% hObject    handle to menu_others (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



% --------------------------------------------------------------------
function whole_body_plot_Callback(hObject, eventdata, handles)

global trajectory d_trajectory dd_trajectory
global q dq ddq

trajectories.operational.trajectory = trajectory;
trajectories.operational.d_trajectory = d_trajectory;
trajectories.operational.dd_trajectory = dd_trajectory;

trajectories.joints.q = q;
trajectories.joints.dq = dq;
trajectories.joints.ddq = ddq;

trajectory_plots(trajectories.operational, trajectories.joints, handles.humanoid_structure);


function Untitled_9_Callback(hObject, eventdata, handles)


function relative_position_Callback(hObject, eventdata, handles)
global trajectory d_trajectory dd_trajectory
global q dq ddq
global data

prevq = q;
prevdq = dq;
prevddq = ddq;

[newq, newdq, newddq, support_foot] = joints_space_interpolation(handles.humanoid_structure, handles.h, q(:,1), zeros(26,1), data.Ts);

if (~isempty(newq) && ~isempty(newdq) && ~isempty(newddq))
  q = [newq prevq];
  dq = [newdq prevdq];
  ddq = [newddq prevddq];
  handles.result.q = q;
  handles.result.dq = dq;
  handles.result.ddq = ddq;
  
  handles.result.trajectory.SF = [support_foot handles.result.trajectory.SF];
  handles.result.trajectory.time = 0:data.Ts:(data.Ts*size(q,2)-data.Ts);
  
  
  guidata(hObject,handles)
  
  % Show plots
  set(handles.plot_options, 'Visible', 'On')
  plot_all_graphs(hObject,handles,'PLOT SPACE') % Show joints space by default
end


function delta_yaw_CoM_DS1_Callback(hObject, eventdata, handles)

function delta_yaw_CoM_DS1_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function delta_yaw_CoM_SS2_Callback(hObject, eventdata, handles)

function delta_yaw_CoM_SS2_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function delta_yaw_CoM_SS1_Callback(hObject, eventdata, handles)

function delta_yaw_CoM_SS1_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function delta_yaw_CoM_DS2_Callback(hObject, eventdata, handles)

function delta_yaw_CoM_DS2_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function delta_yaw_FF_SS2_Callback(hObject, eventdata, handles)

function delta_yaw_FF_SS2_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function delta_yaw_FF_SS1_Callback(hObject, eventdata, handles)

function delta_yaw_FF_SS1_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function joint_angles_menu_Callback(hObject, eventdata, handles)
if isstruct(handles.result)
  save_trajectory_window('teo_step', handles.result.q, '', '', 'joints');
else
  errordlg('There is not any Joint Angles data to save','Save Error')
  return
end

function ang_vel_acc_menu_Callback(hObject, eventdata, handles)
if isstruct(handles.result)
  save_trajectory_window('teo_step', handles.result.q, handles.result.dq, handles.result.ddq, 'joints');
else
  errordlg('There is not any Joint Angles data to save','Save Error')
  return
end
