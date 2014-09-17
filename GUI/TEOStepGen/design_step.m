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
% End initialization code - DO NOT EDIT


% --- Executes just before design_step is made visible.
function design_step_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to design_step (see VARARGIN)
% Choose default command line output for design_step
global h TEO data

% Hide plot_options panel
set(handles.plot_options, 'Visible', 'Off')

% Choose default command line output for design_step
handles.output = hObject;

handles.interpola_DS1='Polynomial5';
handles.interpola_DS2='Polynomial5';
handles.interpola_SS_com='Polynomial5';
handles.interpola_SS_ff='Polynomial5';
handles.interpola_RH='Polynomial5';
handles.interpola_LH='Polynomial5';


if isempty(varargin)
    % Default values
    handles.Input_data.Ts_val = 0.02;
    handles.Input_data.T_val = 4;  
    handles.Input_data.alpha_ds = 1/3;
    handles.Input_data.alpha_sf = 0.2;
    handles.Input_data.DS_or_SS = 'Double and Simple';
    handles.Input_data.Leg = 'Right Leg';
    handles.Input_data.L_val = 0.05;
    handles.Input_data.H_val = 0.03;
    
   handles.Input_data.q0=[0; 0.00325683448936741; -0.308647699300050; 0.796421295515307; -0.487773596215257; 0.0278918646012491;... % Right Leg
     0; 0.00325683448936741; -0.311486990906165; 0.796421295515307; -0.484850796032492; -0.0354911450764397;...                     % Left Leg
     0.0349065850398866; 0;...                                                                                                      % Waist
     0.420000000000000; -0.167017153300893; 0; -1.250000000000000; 0; 0; ...                                                        % Right Arm
     0.420000000000000; 0.167017153300893; 0; -1.250000000000000; 0; 0];                                                            % Left Arm 
%      1.57079632679490; -0.167017153300893; 0; -0.734875474523928; 0; 0;                                                             % Right Arm
%      1.57079632679490; 0.167017153300893; 0;  -0.734875474523928; 0; 0];                                                            % Left Arm   
    
%     handles.Input_data.q0=[0; 0.00325683448936741; -0.308647699300050;...
%             0.796421295515307; -0.487773596215257; 0.0278918646012491;...
%             0; 0.00325683448936741; -0.311486990906165;...
%             0.796421295515307;...
%             -0.484850796032492; -0.0354911450764397; 0.0349065850398866;...
%             1.57079632679490; -0.167017153300893; 0; -0.734875474523928;...
%             0; 1.57079632679490; 0.167017153300893; 0;...
%              -0.734875474523928; 0;0;0;0];

else
    handles.Input_data = varargin{:};
end

% Define some default values
handles.parameters.kp = 0.01;
handles.parameters.ko = pi/8;
handles.result = 0;


% Assign Input Data
% L = handles.Input_data.L_val;
% H = handles.Input_data.H_val;
q0 = handles.Input_data.q0;
Leg = handles.Input_data.Leg;
%set(handles.text1_design,'String',Leg);
DS_or_SS = handles.Input_data.DS_or_SS;
% alpha_sf = handles.Input_data.alpha_sf;

% Load TEO Kinematics Library and Structure
h = TEO_kinematics_library;
TEO = TEO_structure('numeric', 'rad', 'm'); 
handles.humanoid_structure = TEO;
data = []; % Its value change when a step is generated

if strcmp(DS_or_SS,'Simple')
    %set(handles.DS1_data_panel,'Visible','off');
    set(handles.delta_x_CoM_DS1,'Enable','inactive');
    set(handles.delta_y_CoM_DS1,'Enable','inactive');
    set(handles.delta_z_CoM_DS1,'Enable','inactive');
    set(handles.interpolaDS1,'Enable','inactive');
    set(handles.interpolaDS2,'Enable','inactive');
    %children = get(handles.uipanelDelta_CoM_DS1,'Children');
    %children = get(handles.DS1_data_panel,'Children');
    %set(children,'Enable','inactive');
    %children = get(handles.uipanelDS1_interpolation,'Children');
    %set(children,'Enable','inactive');
    %set(handles.text2_design,'String','No Double Support');
    set(handles.DS1_data_panel,'Title','No Double Support');
else
    set(handles.DS1_data_panel,'Visible','on');
end




% Assign support_foot variable
switch Leg
  case 'Right Leg' % Support on right foot
    support_foot = 'RF';

  case 'Left Leg' % Support on left foot
    support_foot = 'LF';    
end

% Pre-evaluate delta x and delta y CoM in Double Support
pre_evaluate_delta_com_ds(hObject, handles, support_foot, h, TEO, q0);

% Update title
set(handles.text_title,'String',strcat('DESIGN STEP - ', ' ', Leg,' Support'));

handles.support_foot = support_foot;


% Pre-evaluate foot deltas in Simple and Double Support
pre_evaluate_delta_foot(hObject, handles);
% L = handles.Input_data.L_val;
% H = handles.Input_data.H_val;
% alpha_sf = handles.Input_data.alpha_sf;
% 
% X1_SF = L*alpha_sf/2;
% set(handles.delta_x_CoM_SS1,'String',num2str(X1_SF),'BackgroundColor','red');
% X2_SF = L*alpha_sf;
% set(handles.delta_x_CoM_SS2,'String',num2str(X2_SF),'BackgroundColor','red');
% 
% X1_FF = L*(1-alpha_sf)/2;
% set(handles.delta_x_FF_SS1,'String',num2str(X1_FF),'BackgroundColor','red');
% set(handles.delta_z_FF_SS1,'String',num2str(H),'BackgroundColor','red');
% 
% X2_FF = L*(1-alpha_sf);
% set(handles.delta_x_FF_SS2,'String',num2str(X2_FF),'BackgroundColor','red');


% Pre-evaluate delta x and delta y Arm in Simple Support
pre_evaluate_delta_arm_ss(hObject, handles, support_foot);


graphstab = uitabpanel(...
  'Parent',handles.panel_graphics,...
  'Style','popup',...
  'Units','normalized',...
  'Position',[0,0,1,1],...
  'FrameBackgroundColor',[0.078,0.169,0.549],...
  'FrameBorderType','etchedin',...
  'Title',{'Right Leg','Left Leg','Torso','Right Arm','Left Arm','CoM'},...
  'PanelHeights',[59.5,59.5,50,59.5,59.5,50],...
  'HorizontalAlignment','left',...
  'FontWeight','bold',...
  'TitleBackgroundColor',[0.078,0.169,0.549],...
  'TitleForegroundColor',[1 1 1],...
  'PanelBackgroundColor',[0.702,0.78,1],...
  'PanelBorderType','line','SelectedItem',6);

hpanel = getappdata(graphstab,'panels');



% *******************************

   handles.sd_graph = axes('Parent',getappdata(graphstab,'status'),...
    'Units','normalized',...
      'Position',[0.3,0.1,0.4,0.9],'Box','on'); 
  
   % Ejes para mostrar el valor de las articulaciones de la pierna derecha
   handles.axes1 = axes('Parent',hpanel(1),'Position',[.1 .6 .25 .25]);
   handles.axes2 = axes('Parent',hpanel(1),'Position',[.4 .6 .25 .25]);
   handles.axes3 = axes('Parent',hpanel(1),'Position',[.7 .6 .25 .25]);
   handles.axes4 = axes('Parent',hpanel(1),'Position',[.1 .15 .25 .25]);
   handles.axes5 = axes('Parent',hpanel(1),'Position',[.4 .15 .25 .25]);
   handles.axes6 = axes('Parent',hpanel(1),'Position',[.7 .15 .25 .25]);
   
   % Ejes para mostrar el valor de las articulaciones de la pierna
   % izquierda
   handles.axes7 = axes('Parent',hpanel(2),'Position',[.1 .6 .25 .25]);
   handles.axes8 = axes('Parent',hpanel(2),'Position',[.4 .6 .25 .25]);
   handles.axes9 = axes('Parent',hpanel(2),'Position',[.7 .6 .25 .25]);
   handles.axes10 = axes('Parent',hpanel(2),'Position',[.1 .15 .25 .25]);
   handles.axes11 = axes('Parent',hpanel(2),'Position',[.4 .15 .25 .25]);
   handles.axes12 = axes('Parent',hpanel(2),'Position',[.7 .15 .25 .25]);
   
   % Ejes para mostrar el valor de las articulaciones del torso
   handles.axes13 = axes('Parent',hpanel(3),'Position',[.1 .6 .25 .25]);
   handles.axes14 = axes('Parent',hpanel(3),'Position',[.7 .6 .25 .25]);   
   
   % Ejes para mostrar el valor de las articulaciones del brazo derecho
   handles.axes15 = axes('Parent',hpanel(4),'Position',[.1 .6 .25 .25]);
   handles.axes16 = axes('Parent',hpanel(4),'Position',[.4 .6 .25 .25]);
   handles.axes17 = axes('Parent',hpanel(4),'Position',[.7 .6 .25 .25]);
   handles.axes18 = axes('Parent',hpanel(4),'Position',[.1 .15 .25 .25]);
   handles.axes19 = axes('Parent',hpanel(4),'Position',[.4 .15 .25 .25]);
   handles.axes20 = axes('Parent',hpanel(4),'Position',[.7 .15 .25 .25]);
   
   % Ejes para mostrar el valor de las articulaciones del brazo izquierdo
   handles.axes21 = axes('Parent',hpanel(5),'Position',[.1 .6 .25 .25]);
   handles.axes22 = axes('Parent',hpanel(5),'Position',[.4 .6 .25 .25]);
   handles.axes23 = axes('Parent',hpanel(5),'Position',[.7 .6 .25 .25]);
   handles.axes24 = axes('Parent',hpanel(5),'Position',[.1 .15 .25 .25]);
   handles.axes25 = axes('Parent',hpanel(5),'Position',[.4 .15 .25 .25]);
   handles.axes26 = axes('Parent',hpanel(5),'Position',[.7 .15 .25 .25]);

   
   % Ejes para mostrar el valor del CoM
   handles.axes_zmp = axes('Parent',hpanel(6),'Position',[.2 .2 .6 .6]);
   %handles.axes_robot_plot = axes('Parent',hpanel(6),'Position',[.2 .2 .6 .6]);
   
   % Ejes para mostrar la imagen de TEO
   axes(handles.sd_graph)
   [r,map] = imread('teo_photo2.jpg','jpg');
   image(r);colormap(map);axis off
   
   axes(handles.axes_zmp)
   [r,map] = imread('teo_photo2.jpg','jpg');
   image(r);colormap(map);axis off

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


function pre_evaluate_delta_com_ds(hObject, handles, support_foot, h, TEO, q)
switch support_foot
  case 'RF' % Support on right foot
    delta = h.CoM_T_RF(q);
    delta_P(1,:) = (2*delta(1) + (TEO.legs.right.foot.limits.x(1) + TEO.legs.right.foot.limits.x(2)));
    delta_P(2,:) = (2*delta(2) + (TEO.legs.right.foot.limits.y(1) + TEO.legs.right.foot.limits.y(2)));

  case 'LF' % Support on left foot
    delta = h.CoM_T_LF(q);
    delta_P(1,:) = (2*delta(1) + (TEO.legs.left.foot.limits.x(1) + TEO.legs.left.foot.limits.x(2)));
    delta_P(2,:) = (2*delta(2) + (TEO.legs.left.foot.limits.y(1) + TEO.legs.left.foot.limits.y(2)));

end

delta_P = delta_P/2;
set(handles.delta_x_CoM_DS1,'String',num2str(delta_P(1)),'BackgroundColor','red');
set(handles.delta_y_CoM_DS1,'String',num2str(delta_P(2)),'BackgroundColor','red');
set(handles.delta_x_CoM_DS2,'String',num2str(-delta_P(1)),'BackgroundColor','red');
set(handles.delta_y_CoM_DS2,'String',num2str(-delta_P(2)),'BackgroundColor','red');

guidata(hObject, handles)


function pre_evaluate_delta_foot(hObject, handles)

L = handles.Input_data.L_val;
H = handles.Input_data.H_val;
alpha_sf = handles.Input_data.alpha_sf;

X1_SF = L*alpha_sf/2;
set(handles.delta_x_CoM_SS1,'String',num2str(X1_SF),'BackgroundColor','red');
X2_SF = L*alpha_sf;
set(handles.delta_x_CoM_SS2,'String',num2str(X2_SF),'BackgroundColor','red');

X1_FF = L*(1-alpha_sf)/2;
set(handles.delta_x_FF_SS1,'String',num2str(X1_FF),'BackgroundColor','red');
set(handles.delta_z_FF_SS1,'String',num2str(H),'BackgroundColor','red');

X2_FF = L*(1-alpha_sf);
set(handles.delta_x_FF_SS2,'String',num2str(X2_FF),'BackgroundColor','red');
guidata(hObject, handles)

function pre_evaluate_delta_arm_ss(hObject, handles, support_foot)

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


% Wait bar
waitbar1= waitbar(0,'Please wait...');

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
data.alpha_sf = handles.Input_data.alpha_sf;    % Percentage of the total time for support foot ???
data.DS_or_SS = handles.Input_data.DS_or_SS;    %
data.L = handles.Input_data.L_val;              % Length of the step (X direction)
data.H = handles.Input_data.H_val;              % Height of the step (Z direction)
                % Initial pose
    
% Delta Data
global delta
  delta.delta_CoM_DS1 = [str2double(get(handles.delta_x_CoM_DS1,'String'));...      % Variation of the CoM in the Double Support phase 1
      str2double(get(handles.delta_y_CoM_DS1,'String'));... 
      str2double(get(handles.delta_z_CoM_DS1,'String'));zeros(3,1)];
  delta.interpola_CoM_DS1 = handles.interpola_DS1;                                  % Interpolation for the CoM in the Double Support phase 1

  delta.delta_CoM_SS1 = [str2double(get(handles.delta_x_CoM_SS1,'String'));...    % Variation of the CoM in the Simple Support phase 1
      str2double(get(handles.delta_y_CoM_SS1,'String'));...
      str2double(get(handles.delta_z_CoM_SS1,'String'));zeros(3,1)];
  delta.delta_CoM_SS2 = [str2double(get(handles.delta_x_CoM_SS2,'String'));...    % Variation of the CoM in the Simple Support phase 2
      str2double(get(handles.delta_y_CoM_SS2,'String'));...
      str2double(get(handles.delta_z_CoM_SS2,'String'));zeros(3,1)];
  delta.interpola_CoM_SS = handles.interpola_SS_com;                              % Interpolation for the CoM in the Simple Support phase (1 and 2)

  delta.delta_FF_SS1 = [str2double(get(handles.delta_x_FF_SS1,'String'));...      % Variation of the Floating Foot in the Simple Support phase 1  
      str2double(get(handles.delta_y_FF_SS1,'String'));...
      str2double(get(handles.delta_z_FF_SS1,'String'));zeros(3,1)];
  delta.delta_FF_SS2 = [str2double(get(handles.delta_x_FF_SS2,'String'));...      % Variation of the Floating Foot in the Simple Support phase 1  
      str2double(get(handles.delta_y_FF_SS2,'String'));...
      str2double(get(handles.delta_z_FF_SS2,'String'));zeros(3,1)];
  delta.interpola_FF_SS = handles.interpola_SS_ff;                                % Interpolation for the Floating Foot in the Simple Support phase (1 and 2)

  delta.delta_CoM_DS2 = [str2double(get(handles.delta_x_CoM_DS2,'String'));...      % Variation of the CoM in the Double Support phase 2
      str2double(get(handles.delta_y_CoM_DS2,'String'));... 
      str2double(get(handles.delta_z_CoM_DS2,'String'));zeros(3,1)];
  delta.interpola_CoM_DS2 = handles.interpola_DS2;                                  % Interpolation for the CoM in the Double Support phase 2

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
  delta.delta_CoM_SS2(1) = delta.delta_CoM_SS1(2)/2;
  delta.delta_FF_SS1(1) = delta.delta_FF_SS1(1)/2;
  delta.delta_FF_SS2(1) = delta.delta_FF_SS2(1)/2;
end

% Double Step and Simple Step for TEO Robot
[q, dq, ddq, trajectory, d_trajectory, dd_trajectory] = ds_ss_step_TEO(delta, data, support_foot, trajectory, d_trajectory, dd_trajectory);


waitbar(0.7)

% borrar el grafico actual de axes_zmp (imagen de TEO):
cla(handles.axes_zmp)

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

% Disable button
disable_generate_step_button(hObject, eventdata, handles)

waitbar(1)
%Close wait bar when ds_ss_step_TEO has finished
close(waitbar1)

guidata(hObject,handles)


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

for jj=1:6
  cla(handles.(strcat('axes',num2str(jj))))
  cla(handles.(strcat('axes',num2str(jj+6))))
  
end
for jj=1:5
  cla(handles.(strcat('axes',num2str(jj+13))))
  cla(handles.(strcat('axes',num2str(jj+18)))) 
end

plot_all_graphs(hObject,handles,handles.plot_new)

guidata(hObject,handles)


function pushbutton3_Callback(hObject, eventdata, handles)

function pushbutton_matlab_vis_Callback(hObject, eventdata, handles)
global trajectory q TEO h
matlab_visualization (trajectory, q, TEO, h)


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
% hObject    handle to Untitled_3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function ang_vel_acc_save_txt_menu_Callback(hObject, eventdata, handles)
if isstruct(handles.result)
  Q = handles.result.q;
  dQ = handles.result.dq;
  ddQ = handles.result.ddq;
  [m,n] = size(Q);
  %Qtext = [Q(1:6,:);Q(14:17,:);Q(7:12,:);Q(19:22,:);Q(13,:);zeros(1,n);Q(18,:)];
  %dQtext = [dQ(1:6,:);dQ(14:17,:);dQ(7:12,:);dQ(19:22,:);dQ(13,:);zeros(1,n);dQ(18,:)];
  %ddQtext = [ddQ(1:6,:);ddQ(14:17,:);ddQ(7:12,:);ddQ(19:22,:);ddQ(13,:);zeros(1,n);ddQ(18,:)];
  try
    [file1,path] = uiputfile('*.txt','Save Joint Angles as');
    dlmwrite(file1,Q,'delimiter','\t','precision','%.6f')
  catch
     disp('Save joint angles *.txt aborted');
  end
  try
    [file2,path] = uiputfile('*.txt','Save Joint Velocities as');
    dlmwrite(file2,dQ,'delimiter','\t','precision','%.6f')
  catch
    disp('Save joint velocities *.txt aborted');
  end
  try
    [file3,path] = uiputfile('*.txt','Save Joint Accelerations as');
    dlmwrite(file3,ddQ,'delimiter','\t','precision','%.6f')
  catch
    disp('Save joint acceleration *.txt aborted');
  end
else
  errordlg('There is not any Joint Angles data to save','Save Error')
  return
end
guidata(hObject,handles)


% --------------------------------------------------------------------
function ang_vel_acc_save_csv_menu_Callback(hObject, eventdata, handles)
if isstruct(handles.result)
  Q = handles.result.q;
  dQ = handles.result.dq;
  ddQ = handles.result.ddq;
  [m,n] = size(Q);
  %Qtext = [Q(1:6,:);Q(14:17,:);Q(7:12,:);Q(19:22,:);Q(13,:);zeros(1,n);Q(18,:)];
  %dQtext = [dQ(1:6,:);dQ(14:17,:);dQ(7:12,:);dQ(19:22,:);dQ(13,:);zeros(1,n);dQ(18,:)];
  %ddQtext = [ddQ(1:6,:);ddQ(14:17,:);ddQ(7:12,:);ddQ(19:22,:);ddQ(13,:);zeros(1,n);ddQ(18,:)];
  try
  [file1,path] = uiputfile('*.csv','Save Joint Angles as');
  csvid = fopen(file1, 'w');
  fprintf(csvid, '%1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f \n', Q');
  fclose(csvid);
  catch
      disp('Save joint angles *.csv aborted');
  end

  try
    [file2,path] = uiputfile('*.csv','Save Joint Velocities as');
    csvid = fopen(file2, 'w');
    fprintf(csvid, '%1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f \n', dQ');
    fclose(csvid);
  catch
    disp('Save joint velocities *.csv aborted');
  end

  try
    [file3,path] = uiputfile('*.csv','Save Joint Accelerations as');
    csvid = fopen(file3, 'w');
    fprintf(csvid, '%1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f \n', ddQ');
    fclose(csvid);
  catch
    disp('Save joint accelerations *.csv aborted');
  end
else
  errordlg('There is not any Joint Angles data to save','Save Error')
  return
end


% --------------------------------------------------------------------
function ang_vel_acc_csv_menu_Callback(hObject, eventdata, handles)
% hObject    handle to ang_vel_acc_csv_menu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function angles_save_txt_menu_Callback(hObject, eventdata, handles)
if isstruct(handles.result)
  Q = handles.result.q;
  [m,n] = size(Q);
  %Qtext = [Q(1:6,:);Q(14:17,:);Q(7:12,:);Q(19:22,:);Q(13,:);zeros(1,n);Q(18,:)];
  try
  [file,path] = uiputfile('*.txt','Save Joint Angles as');
  dlmwrite(file,Q,'delimiter','\t','precision','%.6f')
  catch
      disp('Save joint angles *.txt aborted');
  end
else
  errordlg('There is not any Joint Angles data to save','Save Error')
  return
end
guidata(hObject,handles)


% --------------------------------------------------------------------
function angles_save_csv_menu_Callback(hObject, eventdata, handles)
if isstruct(handles.result)
  Q = handles.result.q;
  [m,n] = size(Q);
  %Qtext = [Q(1:6,:);Q(14:17,:);Q(7:12,:);Q(19:22,:);Q(13,:);zeros(1,n);Q(18,:)];
  try
  [file,path] = uiputfile('*.csv','Save Joint Angles as');
  csvid = fopen(file, 'w');
  fprintf(csvid, '%1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f, %1.4f \n', Q);
  fclose(csvid);
  catch
      disp('Save joint angles *.csv aborted');
  end
else
  errordlg('There is not any Joint Angles data to save','Save Error')
  return
end
guidata(hObject,handles)


% --------------------------------------------------------------------
function Untitled_8_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


function change_parameters_Callback(hObject, eventdata, handles)
% hObject    handle to change_parameters (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


function ik_gains_Callback(hObject, eventdata, handles)
% Change the values of gains
[handles.parameters.kp handles.parameters.ko] = change_ik_gains(handles.parameters.kp, handles.parameters.ko);
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
global TEO h data
if isstruct(handles.result)
  q = handles.result.q(:,end);
  % Change support_foot
  if strcmp(handles.support_foot, 'RF')
    support_foot = 'LF';
    Leg = 'Left Leg';
  else
    support_foot = 'RF';
    Leg = 'Right Leg';
  end

  pre_evaluate_delta_com_ds(hObject, handles, support_foot, h, TEO, q);
  
  pre_evaluate_delta_arm_ss(hObject, handles, support_foot);
  
  if ~isempty(data)
    data.t0 = data.Tend;                                    % Initial time
    data.Tend = data.t0 + data.T;
    data.q = q;
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
% hObject    handle to edit_nsteps (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_nsteps as text
%        str2double(get(hObject,'String')) returns contents of edit_nsteps as a double


% --- Executes during object creation, after setting all properties.
function edit_nsteps_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_nsteps (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
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
global TEO

global trajectory d_trajectory dd_trajectory
global q dq ddq

trajectories.operational.trajectory = trajectory;
trajectories.operational.d_trajectory = d_trajectory;
trajectories.operational.dd_trajectory = dd_trajectory;

trajectories.joints.q = q;
trajectories.joints.dq = dq;
trajectories.joints.ddq = ddq;

trajectory_plots(trajectories.operational, trajectories.joints, TEO);


function Untitled_9_Callback(hObject, eventdata, handles)


function relative_position_Callback(hObject, eventdata, handles)
global TEO h 
global trajectory d_trajectory dd_trajectory
global q dq ddq
global data

prevq = q;
prevdq = dq;
prevddq = ddq;

[newq, newdq, newddq, support_foot] = joints_space_interpolation(TEO, h, q(:,1), zeros(26,1), data.Ts);

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


% --------------------------------------------------------------------
function ang_vel_acc_save_teo_csv_menu_Callback(hObject, eventdata, handles)
if isstruct(handles.result)
  Q = handles.result.q;
  dQ = handles.result.dq;
  ddQ = handles.result.ddq;
  [m,n] = size(Q);
  %Qtext = [Q(1:6,:);Q(14:17,:);Q(7:12,:);Q(19:22,:);Q(13,:);zeros(1,n);Q(18,:)];
  %dQtext = [dQ(1:6,:);dQ(14:17,:);dQ(7:12,:);dQ(19:22,:);dQ(13,:);zeros(1,n);dQ(18,:)];
  %ddQtext = [ddQ(1:6,:);ddQ(14:17,:);ddQ(7:12,:);ddQ(19:22,:);ddQ(13,:);zeros(1,n);ddQ(18,:)];
  try
  [file1,path] = uiputfile('*.csv','Save Joint Angles as');
  csvid = fopen(file1, 'w');
    fprintf(csvid, '%1.2f %1.2f %1.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %1.2f\n',...
                                                                                                                                   [Q(6,:); Q(5,:); Q(4,:); Q(3,:); Q(2,:); Q(1,:); Q(7:end,:)]);
  fclose(csvid);
  catch
      disp('Save joint angles *.csv aborted');
  end

  try
    [file2,path] = uiputfile('*.csv','Save Joint Velocities as');
    csvid = fopen(file2, 'w');
    fprintf(csvid, '%1.2f %1.2f %1.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %1.2f\n',...
                                                                                                                                 [dQ(6,:); dQ(5,:); dQ(4,:); dQ(3,:); dQ(2,:); dQ(1,:); dQ(7:end,:)]);
    fclose(csvid);
  catch
    disp('Save joint velocities *.csv aborted');
  end

  try
    [file3,path] = uiputfile('*.csv','Save Joint Accelerations as');
    csvid = fopen(file3, 'w');
    fprintf(csvid, '%1.2f %1.2f %1.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %1.2f\n',...
                                                                                                                                 [ddQ(6,:); ddQ(5,:); ddQ(4,:); ddQ(3,:); ddQ(2,:); ddQ(1,:); ddQ(7:end,:)]);
    fclose(csvid);
  catch
    disp('Save joint accelerations *.csv aborted');
  end
else
  errordlg('There is not any Joint Angles data to save','Save Error')
  return
end


% --------------------------------------------------------------------
function angles_save_teo_csv_menu_Callback(hObject, eventdata, handles)
if isstruct(handles.result)
  Q = handles.result.q;
  [m,n] = size(Q);
  %Qtext = [Q(1:6,:);Q(14:17,:);Q(7:12,:);Q(19:22,:);Q(13,:);zeros(1,n);Q(18,:)];
  try
  [file,path] = uiputfile('*.csv','Save Joint Angles as');
  csvid = fopen(file, 'w');
  fprintf(csvid, '%1.2f %1.2f %1.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %1.2f\n',...
                                                                                                                                   [Q(6,:); Q(5,:); Q(4,:); Q(3,:); Q(2,:); Q(1,:); Q(7:end,:)]);
  fclose(csvid);
  catch
      disp('Save joint angles *.csv aborted');
  end
else
  errordlg('There is not any Joint Angles data to save','Save Error')
  return
end
guidata(hObject,handles)
