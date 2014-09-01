%% Footprint Generation
% *Footprint generation*

function varargout = footprint_generation(varargin)
% FOOTPRINT_GENERATION MATLAB code for footprint_generation.fig
%      FOOTPRINT_GENERATION, by itself, creates a new FOOTPRINT_GENERATION or raises the existing
%      singleton*.
%
%      H = FOOTPRINT_GENERATION returns the handle to a new FOOTPRINT_GENERATION or the handle to
%      the existing singleton*.
%
%      FOOTPRINT_GENERATION('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in FOOTPRINT_GENERATION.M with the given input arguments.
%
%      FOOTPRINT_GENERATION('Property','Value',...) creates a new FOOTPRINT_GENERATION or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before footprint_generation_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to footprint_generation_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help footprint_generation

% Last Modified by GUIDE v2.5 08-Jun-2014 21:15:40

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @footprint_generation_OpeningFcn, ...
                   'gui_OutputFcn',  @footprint_generation_OutputFcn, ...
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


% --- Executes just before footprint_generation is made visible.
function footprint_generation_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to footprint_generation (see VARARGIN)

clear P p Points
global data trajectories

global P p 
global Points thetas

Points = [0 0 0];
thetas = 0;
P = [];
p = [];

trajectories.joints = [];


waitbarOpening = waitbar(0,'Please wait...');

% DEFAULT VALUES
data.step_parameters.step_length = 0.20;
data.step_parameters.nsteps = 0;
data.step_parameters.t_step = 4;
data.step_parameters.step_height = 0.10;
data.step_parameters.step_d = 0.1073;

data.step_parameters.init_floating_foot = 'Right';
data.step_parameters.finish_both_feet = 1;
data.step_parameters.ds_percentage = 0.20;

waitbar(0.1);

data.trajectory_settings.parameters.g = 9.1;
data.trajectory_settings.parameters.kp = 0.01;
data.trajectory_settings.parameters.ko = pi/8;
data.cog_parameters.zc = 1.0464;
data.cog_parameters.lambda = 0.7;
data.cog_parameters.alpha = 0.1;
data.cog_parameters.beta = 0.3;

waitbar(0.3);
data.trajectory_settings.humanoid_fields = humanoid_operational_fields ();

% Initial Configuration
data.trajectory_settings.q0 = [ 0.000000002196439; 0.010832397471816; -0.551079679399186; 1.164867614763564; -0.613787939728849; -0.010832398780057; ... % Right Leg
                                0.000000000586019; -0.010832395710800; -0.551079681180760; 1.164867615158001; -0.613787930690068; 0.010832389542969; ... % Left Leg
                                0; 0;...                                                                                   % Waist
                                0.420000000000000; -0.167017153300893; 0; -1.250000000000000; 0; 0; ...
                                0.420000000000000; 0.167017153300893; 0; -1.250000000000000; 0; 0];                                
%                                -0.261799387799149; -0.167017153300893; 0; -0.734875474523928; 0; 0;...                                      % Right Arm
%                                -0.261799387799149; 0.167017153300893; 0;  -0.734875474523928; 0; 0];                                        % Left Arm ;

waitbar(0.4);
% TEO Kinematics Library
data.trajectory_settings.h = TEO_kinematics_library;

waitbar(0.6);
% TEO fields
data.trajectory_settings.TEO = TEO_structure('numeric', 'rad', 'm');

% Default Sample Time
data.trajectory_settings.parameters.Ts = 0.02;

% Choose default command line output for footprint_generation
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

waitbar(1.0);
close(waitbarOpening)

% Initial Floor Limits
handles.floor_options.xmin = -1;
handles.floor_options.xmax = 2;
handles.floor_options.ymin = -1;
handles.floor_options.ymax = 1;
handles.floor_options.xscale = 0.2;
handles.floor_options.yscale = 0.2;
guidata(hObject, handles);

% Axes limits
axis(handles.axes_flat_floor, [handles.floor_options.xmin handles.floor_options.xmax handles.floor_options.ymin handles.floor_options.ymax]);

% Panel visualization
grid(handles.axes_flat_floor,'on');
set(handles.axes_flat_floor,'XTick',handles.floor_options.xmin:handles.floor_options.xscale:handles.floor_options.xmax);
set(handles.axes_flat_floor,'YTick',handles.floor_options.ymin:handles.floor_options.yscale:handles.floor_options.ymax);

% Plot world Frame
plot_world_frame(handles)

% Plot footprints
posfootprintRF.inertial = [0, -0.1073, 0];
posfootprintLF.inertial = [0, 0.1073, 0];
% plot(plot::Circle2d(0.1, [0, -0.1186]), plot::Circle2d(0.2, [0, 0.1186]));
% radius = 0.03;
% footprintRF = plot2dcircle(0, -0.1186, radius);
% footprintLF = plot2dcircle(0, 0.1186, radius);

plot_footprint(posfootprintRF.inertial, 0, 'Right');
plot_footprint(posfootprintLF.inertial, 0, 'Left');


function varargout = footprint_generation_OutputFcn(hObject, eventdata, handles) 
varargout{1} = handles.output;



function pushbutton_generate_nsteps_Callback(hObject, eventdata, handles)
global median_zmp
global footprints_poses
global data

% Get step parameters
data.step_parameters.nsteps = str2double(get(handles.edit_nsteps,'String'));
data.step_parameters.step_length = str2double(get(handles.edit_step_length,'String'));
data.step_parameters.step_d = str2double(get(handles.edit_step_d,'String'));
data.step_parameters.t_step = str2double(get(handles.edit_time_step,'String'));

% Check introduced values
if ~check_input_value(data.step_parameters.nsteps, 'Number of Steps')
  return;
end
if ~check_input_value(data.step_parameters.step_length, 'Step Length', 'nan')
  return;
end
if ~check_input_value(data.step_parameters.step_d, 'Step Length')
  return;
end

if ~check_input_value(data.step_parameters.t_step, 'Step Time','nan')
  return;
end

% Calculate the number of footprints
if data.step_parameters.finish_both_feet,
    nfootprints = data.step_parameters.nsteps + 3;
else
    nfootprints = data.step_parameters.nsteps + 2;
end

% Fill footprints_poses.local and footprints_poses.inertial structures
footprints_poses.inertial = zeros(6, nfootprints);

% Calculate first footprints position
%TODO: Calcular la mitad usando la librerias
posfootprintRF.inertial = [0; -data.step_parameters.step_d; 0];
posfootprintLF.inertial = [0; data.step_parameters.step_d; 0];

% Get start foot and assign to flag
if strcmp(data.step_parameters.init_floating_foot,'Right')
    startLF = 1;
else
    startLF = 0;
end

% Get theta parameter
theta = str2double(get(handles.edit_theta_variation,'String'));
if ~check_input_value(theta, 'Theta Variation','nan')
  return;
end

% Calculate First two foot prints poses
if startLF == 0,
    footprints_poses.inertial(1:3,1) = posfootprintLF.inertial;
    footprints_poses.inertial(1:3,2) = posfootprintRF.inertial;
else
    footprints_poses.inertial(1:3,1) = posfootprintRF.inertial;
    footprints_poses.inertial(1:3,2) = posfootprintLF.inertial;
end


% Calculate N footprints
for ii = 1:data.step_parameters.nsteps,
  % Get the footprint that corresponds with the support foot
  poscurrentfootprint.inertial = footprints_poses.inertial(1:3,ii+1);
  
  % Get the variation
  if ii == 1,
%     xvar = data.step_parameters.step_length/4; % El primer paso solo sera la mitad de un paso completo
    xvar = data.step_parameters.step_length/2;
    thetaincr = theta/2;
  else
    xvar = data.step_parameters.step_length/2;
    thetaincr = theta/2;
  end
  yvar = data.step_parameters.step_d*2;
  zvar = 0; % Because it is just footprints

  % Get the inertial theta
  theta_inertial = footprints_poses.inertial(6,ii+1) + thetaincr;
  
  footprints_poses.inertial(1:3,ii+2) = poscurrentfootprint.inertial + rotz(theta_inertial)*[xvar; -(-1)^(ii+startLF)*yvar; zvar];
  footprints_poses.inertial(6,ii+2) = theta_inertial;
end

% Last footprint (only if user selects finish with both feet)
if data.step_parameters.finish_both_feet
  poscurrentfootprint.inertial = footprints_poses.inertial(1:3,end-1);
  xvar = 0;
  theta_inertial = footprints_poses.inertial(6,end-1); % Same orientation than last footprint
  footprints_poses.inertial(1:3,end) = poscurrentfootprint.inertial + rotz(theta_inertial)*[xvar; -(-1)^(ii+startLF+1)*yvar; zvar];
  footprints_poses.inertial(6,end) = theta_inertial;
end


side_foot = data.step_parameters.init_floating_foot;

% Plot all footprints (except first two)
for jj = 3:size(footprints_poses.inertial,2)    
  plot_footprint(footprints_poses.inertial(1:3,jj), footprints_poses.inertial(6,jj), side_foot);

  if strcmp(side_foot,'Right')    
    side_foot = 'Left';
  else
    side_foot = 'Right';
  end
end

% D.E.: Calcular median_zmp.local
median_zmp.inertial = calculate_median_zmp (footprints_poses.inertial);
% median_zmp.local = calculate_median_zmp (footprints_poses.local, 'local');
plot_median_zmp (median_zmp)

function median_zmp = calculate_median_zmp (footprints, poses_type)
if nargin < 2
  poses_type = 'inertial';
end

if strcmp(poses_type,'inertial')
  median_zmp = zeros(2,size(footprints,2)-1);

  for n = 1:size(median_zmp,2)
      median_zmp(:,n) = (footprints(1:2,n) + footprints(1:2,n+1))/2;
  end
  
else
  median_zmp = footprints(1:2,:)/2;
end
    

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Step generation functions %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function pushbutton_local_motion_feet_Callback(hObject, eventdata, handles)
global footprints_poses
global trajectories
global data

% Check inputs values before assign it to variable
if ~check_input_value(str2double(get(handles.edit_double_support_percentage,'String')), 'Double Step (DS) percentage')
  return;
end
data.step_parameters.ds_percentage = str2double(get(handles.edit_double_support_percentage,'String'))/100;

% Check input value before assign it to variable
if ~check_input_value(str2double(get(handles.edit_ts,'String')), 'Sample Time (Ts)')
  return;
end
data.trajectory_settings.parameters.Ts = str2double(get(handles.edit_ts,'String'));


% Calculate double and single step time
ds_time = round_to_Ts(data.step_parameters.ds_percentage*data.step_parameters.t_step, data.trajectory_settings.parameters.Ts);
ss_time = round_to_Ts(data.step_parameters.t_step - ds_time, data.trajectory_settings.parameters.Ts);
current_time = 0;

%DEBUG
global pp_FF dpp_FF ddpp_FF datads1 Tds1

% RF_poses
% LF_poses
[trajectories.operational.inertial.trajectory trajectories.operational.inertial.d_trajectory trajectories.operational.inertial.dd_trajectory] = create_TEO_structures(data.trajectory_settings.parameters.Ts);

floating_foot = data.step_parameters.init_floating_foot;

% for ii=1:size(footprints_poses.inertial,2)-1
for ii = 2:size(footprints_poses.inertial,2) - 1 %DEBE SER -1, HAY QUE AGREGAR LA HUELLA DE DOBLE SOPORTE
  % Double Support Phase 1 (half of Double Support phase)
  initial_time = current_time;
  final_time = initial_time + ds_time/2;

  if ii == 2, % DS1 for first step
    if strcmp(floating_foot, 'Left')
        pLF = footprints_poses.inertial(:,ii);
        pRF = footprints_poses.inertial(:,ii-1);
    else
        pLF = footprints_poses.inertial(:,ii);
        pRF = footprints_poses.inertial(:,ii-1);
    end   
    
    % Left Foot
    P1_LF = set_trajectory_condition(initial_time, pLF, zeros(6,1), zeros(6,1));
    P2_LF = set_trajectory_condition(final_time, pLF, zeros(6,1), zeros(6,1));
    P_LF = [P1_LF P2_LF];
    [pp_LF, dpp_LF, ddpp_LF] = poly5_viapoints_trajectory (P_LF, data.trajectory_settings.parameters.Ts);
    trajectories.operational.inertial.trajectory   = insert_trajectory(trajectories.operational.inertial.trajectory,   data.trajectory_settings.humanoid_fields, pp_LF,  'LF');
    trajectories.operational.inertial.d_trajectory = insert_trajectory(trajectories.operational.inertial.d_trajectory, data.trajectory_settings.humanoid_fields, dpp_LF, 'LF');
    trajectories.operational.inertial.dd_trajectory = insert_trajectory(trajectories.operational.inertial.dd_trajectory, data.trajectory_settings.humanoid_fields, ddpp_LF, 'LF');
    
    % Right Foot
    P1_RF = set_trajectory_condition(initial_time, pRF, zeros(6,1), zeros(6,1));
    P2_RF = set_trajectory_condition(final_time, pRF, zeros(6,1), zeros(6,1));
    P_RF = [P1_RF P2_RF];
    [pp_RF, dpp_RF, ddpp_RF] = poly5_viapoints_trajectory (P_RF, data.trajectory_settings.parameters.Ts);
    trajectories.operational.inertial.trajectory   = insert_trajectory(trajectories.operational.inertial.trajectory,   data.trajectory_settings.humanoid_fields, pp_RF,  'RF');
    trajectories.operational.inertial.d_trajectory = insert_trajectory(trajectories.operational.inertial.d_trajectory, data.trajectory_settings.humanoid_fields, dpp_RF, 'RF');
    trajectories.operational.inertial.dd_trajectory = insert_trajectory(trajectories.operational.inertial.dd_trajectory, data.trajectory_settings.humanoid_fields, ddpp_RF, 'RF');
    
    %DEBUG
    datads1 = size(pp_RF.data);
    Tds1 = pp_RF.T;
    
  else % DS1 other steps (Repeat last value)
    SF = 0;
    time_DS = initial_time:final_time;
    pp_SF  = create_trajectory_structure(SF*ones(size(time_DS)), data.trajectory_settings.parameters.Ts, time_DS);        
    trajectories.operational.inertial.trajectory   = insert_trajectory(trajectories.operational.inertial.trajectory,   data.trajectory_settings.humanoid_fields, pp_SF,  'SF');
    trajectories.operational.inertial.d_trajectory = insert_trajectory(trajectories.operational.inertial.d_trajectory, data.trajectory_settings.humanoid_fields, pp_SF, 'SF');
    trajectories.operational.inertial.dd_trajectory = insert_trajectory(trajectories.operational.inertial.dd_trajectory, data.trajectory_settings.humanoid_fields, pp_SF, 'SF'); 

    %DEBUG
    datads1 = size(pp_RF.data);
    Tds1 = pp_RF.T;
  end

  current_time = final_time;

  % Single Support Phase
  p0_ff = footprints_poses.inertial(:,ii-1);
  p2_ff = footprints_poses.inertial(:,ii+1);
  p1_ff = (p0_ff + p2_ff)/2 + [0;0;data.step_parameters.step_height;0;0;0]; %Elevate foot
  
  initial_time = current_time;
  climbing_time = initial_time + ss_time/2;
  landing_time = climbing_time + ss_time/2;

  P0 = set_trajectory_condition(initial_time, p0_ff, zeros(6,1), zeros(6,1));
  P1 = set_trajectory_condition(climbing_time, p1_ff, zeros(6,1), zeros(6,1));
  P2 = set_trajectory_condition(landing_time, p2_ff, zeros(6,1), zeros(6,1)); 
  
  P = [P0 P1 P2];
  [pp_FF, dpp_FF, ddpp_FF] = poly5_viapoints_trajectory (P, data.trajectory_settings.parameters.Ts);
  
  if strcmp(floating_foot,'Left')
    trajectories.operational.inertial.trajectory   = insert_trajectory(trajectories.operational.inertial.trajectory,   data.trajectory_settings.humanoid_fields, pp_FF,  'LF');
    trajectories.operational.inertial.d_trajectory = insert_trajectory(trajectories.operational.inertial.d_trajectory, data.trajectory_settings.humanoid_fields, dpp_FF, 'LF');
    trajectories.operational.inertial.dd_trajectory = insert_trajectory(trajectories.operational.inertial.dd_trajectory, data.trajectory_settings.humanoid_fields, ddpp_FF, 'LF');
    SF = -1;
    floating_foot = 'Right';
  else
    trajectories.operational.inertial.trajectory   = insert_trajectory(trajectories.operational.inertial.trajectory,   data.trajectory_settings.humanoid_fields, pp_FF,  'RF');
    trajectories.operational.inertial.d_trajectory = insert_trajectory(trajectories.operational.inertial.d_trajectory, data.trajectory_settings.humanoid_fields, dpp_FF, 'RF');
    trajectories.operational.inertial.dd_trajectory = insert_trajectory(trajectories.operational.inertial.dd_trajectory, data.trajectory_settings.humanoid_fields, ddpp_FF, 'RF');
    SF = 1;
    floating_foot = 'Left';
  end

  pp_SF  = create_trajectory_structure(SF*ones(size(pp_FF.time)), data.trajectory_settings.parameters.Ts, pp_FF.time);    
  trajectories.operational.inertial.trajectory   = insert_trajectory(trajectories.operational.inertial.trajectory,   data.trajectory_settings.humanoid_fields, pp_SF,  'SF');
  trajectories.operational.inertial.d_trajectory = insert_trajectory(trajectories.operational.inertial.d_trajectory, data.trajectory_settings.humanoid_fields, pp_SF, 'SF');
  trajectories.operational.inertial.dd_trajectory = insert_trajectory(trajectories.operational.inertial.dd_trajectory, data.trajectory_settings.humanoid_fields, pp_SF, 'SF');
    
%     % Double Support Phase 2 (half of Double Support phase)
  SF = 0;
  double_supp_time = landing_time + ds_time/2;
  time_DS = landing_time:data.trajectory_settings.parameters.Ts:double_supp_time;

  pp_SF  = create_trajectory_structure(SF*ones(size(time_DS)), data.trajectory_settings.parameters.Ts, time_DS);
  trajectories.operational.inertial.trajectory   = insert_trajectory(trajectories.operational.inertial.trajectory,   data.trajectory_settings.humanoid_fields, pp_SF,  'SF');
  trajectories.operational.inertial.d_trajectory = insert_trajectory(trajectories.operational.inertial.d_trajectory, data.trajectory_settings.humanoid_fields, pp_SF, 'SF');
  trajectories.operational.inertial.dd_trajectory = insert_trajectory(trajectories.operational.inertial.dd_trajectory, data.trajectory_settings.humanoid_fields, pp_SF, 'SF');
 
  current_time = double_supp_time;
end

%%%%%%%%%%
% LOCAL %%
%%%%%%%%%%
% 
% % Check input value before assign it to variable
% if ~check_input_value(str2double(get(handles.edit_double_support_percentage,'String')), 'Double Step (DS) percentage')
% %   data.step_parameters.ds_percentage = 0.20; % Default
%   return;
% end
% data.step_parameters.ds_percentage = str2double(get(handles.edit_double_support_percentage,'String'))/100;
% 
% 
% % Calculate double and single step time
% ds_time = round_to_Ts(data.step_parameters.ds_percentage * data.step_parameters.t_step, data.trajectory_settings.parameters.Ts);
% ss_time = round_to_Ts(data.step_parameters.t_step - ds_time, data.trajectory_settings.parameters.Ts);
% current_time = 0;
% 
% 
% % RF_poses
% % LF_poses
% [trajectories.operational.local.trajectory trajectories.operational.local.d_trajectory trajectories.operational.local.dd_trajectory] = create_TEO_structures(data.trajectory_settings.parameters.Ts);
% 
% % Get which is the floating_foot
% floating_foot = data.step_parameters.init_floating_foot;
% 
% for ii = 1:size(footprints_poses.local,2)
%   % Double Support Phase 1 (half of Double Support phase)
%   initial_time = current_time;
%   final_time = initial_time + ds_time/2;
% 
%   % DS1
%   % Left Foot
%   P1_LF = set_trajectory_condition(initial_time, zeros(6,1), zeros(6,1), zeros(6,1));
%   P2_LF = set_trajectory_condition(final_time, zeros(6,1), zeros(6,1), zeros(6,1));
%   P_LF = [P1_LF P2_LF];
%   [pp_LF, dpp_LF, ddpp_LF] = poly5_viapoints_trajectory (P_LF, data.trajectory_settings.parameters.Ts);
%   trajectories.operational.local.trajectory   = insert_trajectory(trajectories.operational.local.trajectory,   data.trajectory_settings.humanoid_fields, pp_LF,  'LF');
%   trajectories.operational.local.d_trajectory = insert_trajectory(trajectories.operational.local.d_trajectory, data.trajectory_settings.humanoid_fields, dpp_LF, 'LF');
%   trajectories.operational.local.dd_trajectory = insert_trajectory(trajectories.operational.local.dd_trajectory, data.trajectory_settings.humanoid_fields, ddpp_LF, 'LF');
% 
%   % Right Foot
%   P1_RF = set_trajectory_condition(initial_time, zeros(6,1), zeros(6,1), zeros(6,1));
%   P2_RF = set_trajectory_condition(final_time, zeros(6,1), zeros(6,1), zeros(6,1));
%   P_RF = [P1_RF P2_RF];
%   [pp_RF, dpp_RF, ddpp_RF] = poly5_viapoints_trajectory (P_RF, data.trajectory_settings.parameters.Ts);
%   trajectories.operational.local.trajectory   = insert_trajectory(trajectories.operational.local.trajectory,   data.trajectory_settings.humanoid_fields, pp_RF,  'RF');
%   trajectories.operational.local.d_trajectory = insert_trajectory(trajectories.operational.local.d_trajectory, data.trajectory_settings.humanoid_fields, dpp_RF, 'RF');
%   trajectories.operational.local.dd_trajectory = insert_trajectory(trajectories.operational.local.dd_trajectory, data.trajectory_settings.humanoid_fields, ddpp_RF, 'RF');
% 
%   %DEBUG
%   datads1 = size(pp_RF.data);
%   Tds1 = pp_RF.T;
%     
%   
%   current_time = final_time;
%   % Single Support Phase
%   p0_ff = zeros(6,1);
%   p1_ff = [footprints_poses.local(1,ii)/2; footprints_poses.local(2,ii)/2; data.step_parameters.step_height; 0; 0; 0]; %Elevate foot
%   p2_ff = footprints_poses.local(:,ii);
% 
%   initial_time = current_time;
%   climbing_time = initial_time + ss_time/2;
%   landing_time = climbing_time + ss_time/2;
% 
%   P0 = set_trajectory_condition(initial_time, p0_ff, zeros(6,1), zeros(6,1));
%   P1 = set_trajectory_condition(climbing_time, p1_ff, zeros(6,1), zeros(6,1));
%   P2 = set_trajectory_condition(landing_time, p2_ff, zeros(6,1), zeros(6,1));
% 
%   P = [P0 P1 P2];
%   [pp_FF, dpp_FF, ddpp_FF] = poly5_viapoints_trajectory (P, data.trajectory_settings.parameters.Ts);
%   if strcmp(floating_foot,'Left')
%     trajectories.operational.local.trajectory   = insert_trajectory(trajectories.operational.local.trajectory,   data.trajectory_settings.humanoid_fields, pp_FF,  'LF');
%     trajectories.operational.local.d_trajectory = insert_trajectory(trajectories.operational.local.d_trajectory, data.trajectory_settings.humanoid_fields, dpp_FF, 'LF');
%     trajectories.operational.local.dd_trajectory = insert_trajectory(trajectories.operational.local.dd_trajectory, data.trajectory_settings.humanoid_fields, ddpp_FF, 'LF');
%     SF = -1;
%     floating_foot = 'Right';
%   else
%     trajectories.operational.local.trajectory   = insert_trajectory(trajectories.operational.local.trajectory,   data.trajectory_settings.humanoid_fields, pp_FF,  'RF');
%     trajectories.operational.local.d_trajectory = insert_trajectory(trajectories.operational.local.d_trajectory, data.trajectory_settings.humanoid_fields, dpp_FF, 'RF');
%     trajectories.operational.local.dd_trajectory = insert_trajectory(trajectories.operational.local.dd_trajectory, data.trajectory_settings.humanoid_fields, ddpp_FF, 'RF');
%     SF = 1;
%     floating_foot = 'Left';
%   end
% 
%   pp_SF  = create_trajectory_structure(SF*ones(size(pp_FF.time)), data.trajectory_settings.parameters.Ts, pp_FF.time);
%   trajectories.operational.local.trajectory   = insert_trajectory(trajectories.operational.local.trajectory,   data.trajectory_settings.humanoid_fields, pp_SF,  'SF');
%   trajectories.operational.local.d_trajectory = insert_trajectory(trajectories.operational.local.d_trajectory, data.trajectory_settings.humanoid_fields, pp_SF, 'SF');
%   trajectories.operational.local.dd_trajectory = insert_trajectory(trajectories.operational.local.dd_trajectory, data.trajectory_settings.humanoid_fields, pp_SF, 'SF');
% 
%   
% % Double Support Phase 2 (half of Double Support phase)
%   SF = 0;
%   double_supp_time = landing_time + ds_time/2;
%   time_DS = landing_time:data.trajectory_settings.parameters.Ts:double_supp_time;
% 
%   pp_SF  = create_trajectory_structure(SF*ones(size(time_DS)), data.trajectory_settings.parameters.Ts, time_DS);
%   trajectories.operational.local.trajectory   = insert_trajectory(trajectories.operational.local.trajectory,   data.trajectory_settings.humanoid_fields, pp_SF,  'SF');
%   trajectories.operational.local.d_trajectory = insert_trajectory(trajectories.operational.local.d_trajectory, data.trajectory_settings.humanoid_fields, pp_SF, 'SF');
%   trajectories.operational.local.dd_trajectory = insert_trajectory(trajectories.operational.local.dd_trajectory, data.trajectory_settings.humanoid_fields, pp_SF, 'SF');
% 
%   current_time = double_supp_time;
% end


% Plot Feet trajectories
plot_feet_trajectories();

% Enable Plots
set(handles.pushbutton_whole_trajectory,'Enable','on')
set(handles.pushbutton_plot_feet_trajectory,'Enable','on')


function pushbutton_local_motion_cog_Callback(hObject, eventdata, handles)
global trajectories
global footprints_poses median_zmp
global data

global zmp_trajectory

% Get and check needed parameters
if ~check_input_value(str2double(get(handles.edit_cog_zc,'String')), 'Height of Center of Gravity (zc)')
  return;
end
data.cog_parameters.zc = str2double(get(handles.edit_cog_zc,'String'));

if ~check_input_value(str2double(get(handles.edit_cog_lambda,'String')), 'Lambda','nan')
  return;
end
data.cog_parameters.lambda = str2double(get(handles.edit_cog_lambda,'String'));

if ~check_input_value(str2double(get(handles.edit_cog_beta,'String')), 'Beta','nan')
  return;
end
data.cog_parameters.beta = str2double(get(handles.edit_cog_beta,'String'));

if ~check_input_value(str2double(get(handles.edit_cog_alpha,'String')), 'Alpha','nan')
  return;
end
data.cog_parameters.alpha = str2double(get(handles.edit_cog_alpha,'String'));

% Calculate ds_time and ss_time
t0 = 0;
ds_time = round_to_Ts(data.step_parameters.ds_percentage*data.step_parameters.t_step, data.trajectory_settings.parameters.Ts);
ss_time = round_to_Ts(data.step_parameters.t_step - ds_time, data.trajectory_settings.parameters.Ts);

% Group times
cog_times.t0 = t0;
cog_times.ds_time = ds_time;
cog_times.ss_time = ss_time;

% Select the model to generate the COG trajectory
cog_list_options = cellstr(get(handles.popupmenu_local_motion_cog,'String'));
cog_option = cog_list_options{get(handles.popupmenu_local_motion_cog,'Value')};

switch cog_option
    case 'Polynomial 5'
        [cog_traj, dcog_traj, ddcog_traj] = cog_polynomial5_trajectory(footprints_poses, median_zmp, data.trajectory_settings.parameters.Ts,  ss_time, ds_time, t0, data.cog_parameters.zc);
%         tamano_cog_poly5 = size(cog_traj.data)
%         tamano_dcog_poly5 = size(dcog_traj.data)
%         tamano_ddcog_poly5 = size(ddcog_traj.data)
    case '3D LIPM'
        [cog_traj, dcog_traj, ddcog_traj] = cog_3d_LIPM_trajectory(footprints_poses, median_zmp, data.trajectory_settings.parameters.Ts, ss_time, ds_time, t0, data.cog_parameters.zc, data.cog_parameters.beta, data.cog_parameters.lambda, data.trajectory_settings.parameters.g);
%         tamano_cog_3dlipm = size(cog_traj.data)
%         tamano_cog_3dlipm = size(dcog_traj.data)
%         tamano_cog_3dlipm = size(ddcog_traj.data)
    case 'Cart Table - Kajita'
        [cog_traj, dcog_traj, ddcog_traj] = cog_cart_table_trajectory('Kajita', zmp_trajectory.data, data.trajectory_settings.parameters.Ts, cog_times, data.cog_parameters.zc, data.cog_parameters.beta, data.cog_parameters.lambda, data.trajectory_settings.parameters.g, zmp_trajectory.time);
        
    case 'Cart Table - Wieber'
        [cog_traj, dcog_traj, ddcog_traj] = cog_cart_table_trajectory('Wieber', zmp_trajectory.data, data.trajectory_settings.parameters.Ts, cog_times, data.cog_parameters.zc, data.cog_parameters.beta, data.cog_parameters.lambda, data.trajectory_settings.parameters.g, zmp_trajectory.time);
        
  otherwise
        % - Polynomial 3
        % - Polynomial 7
        disp('ERROR: Opcion no implementada');
        return;
end

trajectories.operational.inertial.trajectory   = insert_trajectory(trajectories.operational.inertial.trajectory,   data.trajectory_settings.humanoid_fields, cog_traj,  'CoM');
trajectories.operational.inertial.d_trajectory = insert_trajectory(trajectories.operational.inertial.d_trajectory, data.trajectory_settings.humanoid_fields, dcog_traj, 'CoM');
trajectories.operational.inertial.dd_trajectory = insert_trajectory(trajectories.operational.inertial.dd_trajectory, data.trajectory_settings.humanoid_fields, ddcog_traj, 'CoM');

% Plot CoM trajectories
plot_CoM_trajectory();

% Update handles structure   
guidata(hObject, handles);


function [cog_traj, dcog_traj, ddcog_traj] = cog_polynomial5_trajectory(footprints_poses, median_zmp, Ts,  TimeSS, TimeDS, t0, zc, alpha, beta, lambda, floating_foot)

if nargin < 12
  poses_type = 'inertial';
end


  current_time = t0;

  for ii=2:size(footprints_poses.inertial,2)-1,
      TDS1 = current_time;
  %     TSSprev = round_to_Ts(t0 + TimeDS/2,Ts);
  %     TSS = round_to_Ts(TSSprev + TimeSS/2,Ts);
      TSS = round_to_Ts(current_time + TimeDS/2 + TimeSS/2,Ts);
  %     TSSnext = round_to_Ts(TSS + TimeSS/2,Ts);
  %     TDS2 = round_to_Ts(TSSnext + TimeDS/2,Ts);
      TDS2 = round_to_Ts(TSS + TimeSS/2 + TimeDS/2,Ts);

  %     DSglobal_pose1 = [median_zmp.inertial(:,ii-1);zc;0;0;mean([footprints_poses.inertial(6,ii),footprints_poses.inertial(6,ii-1)])];
  %     DSglobal_pose2 = [median_zmp.inertial(:,ii);zc;0;0;mean([footprints_poses.inertial(6,ii),footprints_poses.inertial(6,ii+1)])];

      p1_com = [median_zmp.inertial(:,ii-1);zc;0;0;mean([footprints_poses.inertial(6,ii),footprints_poses.inertial(6,ii-1)])];
      p5_com = [median_zmp.inertial(:,ii);zc;0;0;mean([footprints_poses.inertial(6,ii),footprints_poses.inertial(6,ii+1)])];

      p3_com = footprints_poses.inertial(:,ii); p3_com(3) = zc;

  %     % Change global reference to foot reference    
  %     DSpose1 = pose_tr2rpy(pose_rpy2tr(p3_com)\pose_rpy2tr(DSglobal_pose1));
  %     DSpose2 = pose_tr2rpy(pose_rpy2tr(p3_com)\pose_rpy2tr(DSglobal_pose2));
  %     
  %     lambdapose1 = DSpose1(1)*data.cog_parameters.lambda;
  %     lambdapose2 = DSpose2(1)*data.cog_parameters.lambda;
  % 
  %     betapose1 = DSpose1(2)*data.cog_parameters.beta;
  %     betapose2 = DSpose1(2)*data.cog_parameters.beta;
  %     
  %     
  %     p2_com = footprints_poses.inertial(:,ii-1); p2_com(3) = zc;
  %     p4_com = footprints_poses.inertial(:,ii-1); p4_com(3) = zc;
  % 
  %     % Convert foot reference to global reference
  %     p2_com = transform_frame(pose_rpy2tr(footprints_poses.inertial(:,ii)), p2_com);
  %     p4_com = transform_frame(pose_rpy2tr(footprints_poses.inertial(:,ii)), p4_com);

      P1 = set_trajectory_condition(TDS1, p1_com, zeros(6,1), zeros(6,1));
  %     P2 = set_trajectory_condition(TSSprev, p2_com, zeros(6,1), zeros(6,1));
      P3 = set_trajectory_condition(TSS, p3_com, zeros(6,1), zeros(6,1));
  %     P4 = set_trajectory_condition(TSSnext, p4_com, zeros(6,1), zeros(6,1));
      P5 = set_trajectory_condition(TDS2, p5_com, zeros(6,1), zeros(6,1));
  %     P = [P P1 P2 P3 P4 P5];

      if ii==2,
          P = [P1 P3 P5];
      else
          P = [P P3 P5];
      end
      current_time = TDS2;

  end
[cog_traj, dcog_traj, ddcog_traj] = poly5_viapoints_trajectory (P, Ts);



function calculate_zmp_trajectory_from_cog(trajectory, d_trajectory, dd_trajectory)
global g
global zmp_trajectory
% zmp_trajectory = zeros(2,size(trajectory.CoM,2));

zmp_trajectory = trajectory.CoM(1:2,:) - (data.cog_parameters.zc/g)*dd_trajectory.CoM(1:2,:);

hold on
  plot3(zmp_trajectory(1,:), zmp_trajectory(2,:), zeros(1,size(zmp_trajectory,2)), 'color', 'm');
hold off





function pushbutton_plot_feet_trajectory_Callback(hObject, eventdata, handles)
global trajectories

 figure(500),
 subplot(2,1,1), plot(trajectories.operational.inertial.trajectory.time, trajectories.operational.inertial.trajectory.RF(1,:),'*'),
 hold on, subplot(2,1,1), plot(trajectories.operational.inertial.trajectory.time,trajectories.operational.inertial.trajectory.LF(1,:),'*r'),
 subplot(2,1,1), plot(trajectories.operational.inertial.trajectory.time,0.3*trajectories.operational.inertial.trajectory.SF,'-g'), hold off
 
 subplot(2,1,2), plot(trajectories.operational.inertial.trajectory.time, trajectories.operational.inertial.trajectory.RF(2,:),'*'),
 hold on, subplot(2,1,2), plot(trajectories.operational.inertial.trajectory.time,trajectories.operational.inertial.trajectory.LF(2,:),'*r'),
 subplot(2,1,2), plot(trajectories.operational.inertial.trajectory.time,0.3*trajectories.operational.inertial.trajectory.SF,'-g'), hold off
 


function pushbutton_PLOT_FOOT_Y_Callback(hObject, eventdata, handles)
global trajectories
 figure(501), plot(trajectories.operational.inertial.trajectory.time, trajectories.operational.inertial.trajectory.RF(2,:),'*'),
 hold on, plot(trajectories.operational.inertial.trajectory.time,trajectories.operational.inertial.trajectory.LF(2,:),'*r'),
 plot(trajectories.operational.inertial.trajectory.time,0.3*trajectories.operational.inertial.trajectory.SF,'-g'), hold off


function pushbutton_cart_table_cog_Callback(hObject, eventdata, handles)
global zmp_trajectory


% function pushbutton_3dlipm_cog_Callback(hObject, eventdata, handles)
% global trajectories
% global footprints_poses median_zmp 
% global g
% global data
% 
% t0 = 0
% 
% ds_time = round_to_Ts(data.step_parameters.ds_percentage*data.step_parameters.t_step, data.trajectory_settings.parameters.Ts);
% ss_time = round_to_Ts(data.step_parameters.t_step - ds_time, data.trajectory_settings.parameters.Ts);
% 
% data.cog_parameters.zc = str2double(get(handles.edit_cog_zc,'String'));
% 
% data.cog_parameters.lambda = str2double(get(handles.edit_cog_lambda,'String'));
% data.cog_parameters.alpha = str2double(get(handles.edit_cog_alpha,'String'));
% data.cog_parameters.beta = str2double(get(handles.edit_cog_beta,'String'));
% 
% [cog_traj, dcog_traj, ddcog_traj] = cog_3d_LIPM_trajectory(footprints_poses, median_zmp, data.trajectory_settings.parameters.Ts, ss_time, ds_time, t0, data.cog_parameters.zc, data.cog_parameters.beta, data.cog_parameters.lambda, g);
% 
% trajectories.operational.inertial.trajectory   = insert_trajectory(trajectories.operational.inertial.trajectory,   data.trajectory_settings.humanoid_fields, cog_traj,  'CoM');
% trajectories.operational.inertial.d_trajectory = insert_trajectory(trajectories.operational.inertial.d_trajectory, data.trajectory_settings.humanoid_fields, dcog_traj, 'CoM');
% trajectories.operational.inertial.dd_trajectory = insert_trajectory(trajectories.operational.inertial.dd_trajectory, data.trajectory_settings.humanoid_fields, ddcog_traj, 'CoM');
% 
% 
% % Plot CoM trajectories
% plot_CoM_trajectory();


function change_parameters_Callback(hObject, eventdata, handles)


function change_initial_configuration_Callback(hObject, eventdata, handles)
global data
data.trajectory_settings.q0  = change_configuration(data.trajectory_settings.q0, data.trajectory_settings.TEO);
guidata(hObject,handles);


%% Joints Space functions
function pushbutton_step_by_step_ik_Callback(hObject, eventdata, handles)

global data footprints_poses trajectories median_zmp

% check and update all variables before continue
if ~check_all_input_value(handles)
  return;
end

finish_both_feet = data.step_parameters.finish_both_feet;

if finish_both_feet;
  nfootprints = 2*data.step_parameters.nsteps + 2;
else
  nfootprints = 2*data.step_parameters.nsteps;
end
footprints_poses.local = zeros(6,nfootprints);
median_zmp.local = zeros(6,nfootprints);



zc = data.cog_parameters.zc;
alpha = data.cog_parameters.alpha;
beta = data.cog_parameters.beta;
lambda = data.cog_parameters.lambda;
cog_option = data.cog_parameters.cog_option;
g_acc = data.trajectory_settings.parameters.g;

step_height = data.step_parameters.step_height;
step_d = data.step_parameters.step_d;
theta = data.step_parameters.theta_variation;
foot_movement_option = data.step_parameters.foot_option;
arms_movement_option = data.step_parameters.arms_option;
floating_foot = data.step_parameters.init_floating_foot;

t0 = 0;
% Calculate double and single step time
Ts = data.trajectory_settings.parameters.Ts;
TimeDS = round_to_Ts(data.step_parameters.ds_percentage*data.step_parameters.t_step, Ts);
TimeSS = round_to_Ts(data.step_parameters.t_step - TimeDS, Ts);

ntrajdata = 1;
trajectories.joints.q = zeros(26,data.step_parameters.nsteps*round(data.step_parameters.t_step/Ts)+1); % x1 +1?? Revisar
trajectories.joints.dq = zeros(26,data.step_parameters.nsteps*round(data.step_parameters.t_step/Ts)+1);
trajectories.joints.ddq = zeros(26,data.step_parameters.nsteps*round(data.step_parameters.t_step/Ts)+1);
trajectories.joints.q(:,1) = data.trajectory_settings.q0;

[trajectories.operational.local.trajectory trajectories.operational.local.d_trajectory trajectories.operational.local.dd_trajectory] = create_TEO_structures(Ts);


if strcmp(floating_foot, 'Right'),
  frame_sign = -1;
  ff_field = 'RF';
elseif strcmp(floating_foot, 'Left'),
  frame_sign = 1;
  ff_field = 'LF';
end


for ii = 1:nfootprints/2
%   % Get the variation
%   if (data.step_parameters.finish_both_feet) && (ii == (nfootprints - 1))
%     xvar = data.step_parameters.step_length/2;
%     thetaincr = theta/2;
%   elseif ii==1
%     xvar = data.step_parameters.step_length/2;
%     thetaincr = theta/2;
%   else
%     xvar = data.step_parameters.step_length;
%     thetaincr = theta;
%   end
%    
%   yvar = 0;
%   zvar = 0; % Because it is just footprints 
%   
%   % Calculate footprints_poses
%   
%   if ii == 1
%     footprints_poses.local(:,ii) = zeros(6,1); footprints_poses.local(2,1) = frame_sign*step_d*2;
%   else
%     footprints_poses.local(:,ii) = -footprints_poses.local(:,ii-1);
%   end
%   
%   footprints_poses.local(1:3,ii+1) = footprints_poses.local(1:3,ii) + rotz(thetaincr)*[xvar; yvar; zvar];
%   footprints_poses.local(6,ii+1) = footprints_poses.local(6,ii) + thetaincr;
%   
%   plot_footprint_local(footprints_poses.local(:,ii+1))
%   frame_sign = -frame_sign;

if isempty(footprints_poses.inertial)
  errordlg('ERROR: No Footprints data','Footprints Error');
  warning('WarnTEOTraGen:noInputData','No Footprints data');
  return;
end

prev_footprint = pose_rpy2tr(footprints_poses.inertial(:,ii));
support_footprint = pose_rpy2tr(footprints_poses.inertial(:,ii+1));
next_footprint = pose_rpy2tr(footprints_poses.inertial(:,ii+2));

footprints_poses.local(:,2*ii-1) = pose_tr2rpy(multiply_homogeneous_matrix({invert_homogeneous_matrix(support_footprint) prev_footprint}));
footprints_poses.local(:,2*ii) = pose_tr2rpy(multiply_homogeneous_matrix({invert_homogeneous_matrix(support_footprint) next_footprint}));

median_zmp.local(:,2*ii-1) = footprints_poses.local(:,2*ii-1)/2;
median_zmp.local(:,2*ii) = footprints_poses.local(:,2*ii)/2;

end


for ii = 1:2:nfootprints
  step_times.Tinit = t0;
  step_times.TDS1 = round_to_Ts(step_times.Tinit + TimeDS/2,Ts);
  step_times.TSS = round_to_Ts(step_times.TDS1 + TimeSS/2,Ts);
  step_times.TDS2 = round_to_Ts(step_times.TSS + TimeSS/2,Ts);
  step_times.Tend = round_to_Ts(step_times.TDS2 + TimeDS/2,Ts);

  % COG
  [cog_traj, dcog_traj, ddcog_traj] = cog_trajectory_generation(footprints_poses.local(:,ii+1), footprints_poses.local(:,ii), step_times, Ts, alpha, beta, lambda, cog_option, zc, g_acc);
  trajectories.operational.local.trajectory   = insert_trajectory(trajectories.operational.local.trajectory,   data.trajectory_settings.humanoid_fields, cog_traj,  'CoM');
  trajectories.operational.local.d_trajectory = insert_trajectory(trajectories.operational.local.d_trajectory, data.trajectory_settings.humanoid_fields, dcog_traj, 'CoM');
  trajectories.operational.local.dd_trajectory = insert_trajectory(trajectories.operational.local.dd_trajectory, data.trajectory_settings.humanoid_fields, ddcog_traj, 'CoM');
  
  % Foot
  [floating_foot_traj, floating_foot_dtraj, floating_foot_ddtraj] = foot_trajectory_generation(footprints_poses.local(:,ii+1), footprints_poses.local(:,ii), Ts,  step_times, step_height, foot_movement_option);
  trajectories.operational.local.trajectory   = insert_trajectory(trajectories.operational.local.trajectory,   data.trajectory_settings.humanoid_fields, floating_foot_traj,  ff_field);
  trajectories.operational.local.d_trajectory = insert_trajectory(trajectories.operational.local.d_trajectory, data.trajectory_settings.humanoid_fields, floating_foot_dtraj, ff_field);
  trajectories.operational.local.dd_trajectory = insert_trajectory(trajectories.operational.local.dd_trajectory, data.trajectory_settings.humanoid_fields, floating_foot_ddtraj, ff_field);  
  
  
  % Right Arm
  [right_arm_traj, right_arm_dtraj, right_arm_ddtraj] = arm_trajectory_generation(footprints_poses.local(:,ii+1), footprints_poses.local(:,ii), Ts,  step_times, floating_foot, 'Right', arms_movement_option);
  trajectories.operational.local.trajectory   = insert_trajectory(trajectories.operational.local.trajectory,   data.trajectory_settings.humanoid_fields, right_arm_traj,  'RH');
  trajectories.operational.local.d_trajectory = insert_trajectory(trajectories.operational.local.d_trajectory, data.trajectory_settings.humanoid_fields, right_arm_dtraj, 'RH');
  trajectories.operational.local.dd_trajectory = insert_trajectory(trajectories.operational.local.dd_trajectory, data.trajectory_settings.humanoid_fields, right_arm_ddtraj, 'RH');
  
  % Left Arm
  [left_arm_traj, left_arm_dtraj, left_arm_ddtraj] = arm_trajectory_generation(footprints_poses.local(:,ii+1), footprints_poses.local(:,ii), Ts,  step_times, floating_foot, 'Left', arms_movement_option);
  trajectories.operational.local.trajectory   = insert_trajectory(trajectories.operational.local.trajectory,   data.trajectory_settings.humanoid_fields, left_arm_traj,  'LH');
  trajectories.operational.local.d_trajectory = insert_trajectory(trajectories.operational.local.d_trajectory, data.trajectory_settings.humanoid_fields, left_arm_dtraj, 'LH');
  trajectories.operational.local.dd_trajectory = insert_trajectory(trajectories.operational.local.dd_trajectory, data.trajectory_settings.humanoid_fields, left_arm_ddtraj, 'LH');
  
  
  % Support_Foot field
  [ sf_traj, dsf_traj, ddsf_traj ] = support_foot_trajectory_generation(step_times, Ts, floating_foot);
  trajectories.operational.local.trajectory   = insert_trajectory(trajectories.operational.local.trajectory,   data.trajectory_settings.humanoid_fields, sf_traj,  'SF');
  trajectories.operational.local.d_trajectory = insert_trajectory(trajectories.operational.local.d_trajectory, data.trajectory_settings.humanoid_fields, dsf_traj, 'SF');
  trajectories.operational.local.dd_trajectory = insert_trajectory(trajectories.operational.local.dd_trajectory, data.trajectory_settings.humanoid_fields, ddsf_traj, 'SF');
  
% Extract the trajectory corresponding to this step
  ntrajdata_end = size(floating_foot_traj.data,2) + ntrajdata - 1;
  step_trajectory = extract_trajectory(trajectories.operational.local.trajectory,data.trajectory_settings.humanoid_fields, [step_times.Tinit step_times.Tend], 'time');
  step_dtrajectory = extract_trajectory(trajectories.operational.local.d_trajectory,data.trajectory_settings.humanoid_fields, [step_times.Tinit step_times.Tend], 'time');
  step_ddtrajectory = extract_trajectory(trajectories.operational.local.dd_trajectory,data.trajectory_settings.humanoid_fields, [step_times.Tinit step_times.Tend], 'time');

% Differential Inverse Kinematics Algorithm for the step
  [trajectories.joints.q(:,ntrajdata:ntrajdata_end), trajectories.joints.dq(:,ntrajdata:ntrajdata_end), trajectories.joints.ddq(:,ntrajdata:ntrajdata_end)] = inverse_ds_ss_jacobian_quat(trajectories.joints.q(:,ntrajdata), step_trajectory, step_dtrajectory, data.trajectory_settings.h);
  %[trajectories.joints.q(:,ntrajdata:ntrajdata_end), trajectories.joints.dq(:,ntrajdata:ntrajdata_end), trajectories.joints.ddq(:,ntrajdata:ntrajdata_end)] = inverse_ds_ss_jacobian_quat_second_order(trajectories.joints.q(:,ntrajdata), step_trajectory, step_dtrajectory, step_ddtrajectory, data.trajectory_settings.h);
  

  plot_footprint_local(footprints_poses.local(:,ii+1), floating_foot);
  
  t0 = step_times.Tend;
  ntrajdata =  ntrajdata_end;
  if strcmp(floating_foot, 'Right'),
    floating_foot = 'Left';
    ff_field = 'LF';
  elseif strcmp(floating_foot, 'Left'),
    floating_foot ='Right';
    ff_field = 'RF';
  end
    
end

% set(handles.pushbutton_whole_trajectory,'Enable','on')

%   % Get the variation
%   if ii == 1,
% %     xvar = data.step_parameters.step_length/4; % El primer paso solo sera la mitad de un paso completo
%     xvar = data.step_parameters.step_length/2;
%     thetaincr = theta/2;
%   else
%     xvar = data.step_parameters.step_length/2;
%     thetaincr = theta;
%   end
%   yvar = data.step_parameters.step_d*2;
%   zvar = 0; % Because it is just footprints
% 
%   % Get the inertial theta
%   theta_inertial = footprints_poses.inertial(6,ii+1) + thetaincr;
%   
%   footprints_poses.inertial(1:3,ii+2) = poscurrentfootprint.inertial + rotz(theta_inertial)*[xvar; -(-1)^(ii+startLF)*yvar; zvar];
%   footprints_poses.inertial(6,ii+2) = theta_inertial;


%   [ footprints_poses.local(:,ii), footprints_poses.local(:,ii+1) ] = footprint_pose_generation(data.trajectory_settings.h, q(:,ntrajdata) , xvar, step_d, theta_incr, floating_foot);
  
%   
%   % 1 - CoM
%   % Calculate ds_time and ss_time
%   step_times.Tinit = t0;
%   step_times.TDS1 = round_to_Ts(step_times.Tinit + TimeDS/2,Ts);
%   step_times.TSS = round_to_Ts(step_times.TDS1 + TimeSS/2,Ts);
%   step_times.TDS2 = round_to_Ts(step_times.TSS + TimeSS/2,Ts);
%   step_times.Tend = round_to_Ts(step_times.TDS2 + TimeDS/2,Ts);
%   
%   [ cog_traj, dcog_traj, ddcog_traj ] = cog_trajectory_generation( footprints_poses.local(:,ii+1), footprints_poses.local(:,ii), step_times, Ts, floating_foot, alpha, beta, lambda, cog_option);
%   
%   trajectories.operational.local.trajectory   = insert_trajectory(trajectories.operational.local.trajectory,   data.trajectory_settings.humanoid_fields, cog_traj,  'CoM');
%   trajectories.operational.local.d_trajectory = insert_trajectory(trajectories.operational.local.d_trajectory, data.trajectory_settings.humanoid_fields, dcog_traj, 'CoM');
%   trajectories.operational.local.dd_trajectory = insert_trajectory(trajectories.operational.local.dd_trajectory, data.trajectory_settings.humanoid_fields, ddcog_traj, 'CoM');
%   
%   % 2 - Foot Trajectory
%   [ foot_traj, dfoot_traj, ddfoot_traj ] = foot_trajectory_generation( data.trajectory_settings.h, q(:,ntrajdata), step_times, Ts, footprints_poses.local(:,ii+1), footprints_poses.local(:,ii), floating_foot, step_height, foot_option);
%   if strcmp(floating_foot, 'Right'),
%     humanoid_part = 'RF';
%   elseif strcmp(floating_foot, 'Left'),
%     humanoid_part ='LF';
%   else
%     error('ErrorTEOTraGen:wrongOption', 'Wrong Floating Foot option'); 
%   end
%   
%   trajectories.operational.local.trajectory   = insert_trajectory(trajectories.operational.local.trajectory,   data.trajectory_settings.humanoid_fields, foot_traj,  humanoid_part);
%   trajectories.operational.local.d_trajectory = insert_trajectory(trajectories.operational.local.d_trajectory, data.trajectory_settings.humanoid_fields, dfoot_traj, humanoid_part);
%   trajectories.operational.local.dd_trajectory = insert_trajectory(trajectories.operational.local.dd_trajectory, data.trajectory_settings.humanoid_fields, ddfoot_traj, humanoid_part);
% 
%   
%   %%DOMINGO::: VER SI POR SI SOLO LO RELLENA A CEROS, o si copia el ultimo
%   %%v
% %   [ support_foot_traj, dsupport_foot_traj, ddsupport_foot_traj ] = other_foot_trajectory_generation( data.trajectory_settings.h, q(:,ntrajdata), step_times, Ts, floating_foot);
% %   if strcmp(floating_foot, 'Right'),
% %     humanoid_part = 'LF';
% %   elseif strcmp(floating_foot, 'Left'),
% %     humanoid_part ='RF';
% %   else
% %     error('ErrorTEOTraGen:wrongOption', 'Wrong Floating Foot option'); 
% %   end
% % 
% %   trajectories.operational.local.trajectory   = insert_trajectory(trajectories.operational.local.trajectory,   data.trajectory_settings.humanoid_fields, support_foot_traj,  humanoid_part);
% %   trajectories.operational.local.d_trajectory = insert_trajectory(trajectories.operational.local.d_trajectory, data.trajectory_settings.humanoid_fields, dsupport_foot_traj, humanoid_part);
% %   trajectories.operational.local.dd_trajectory = insert_trajectory(trajectories.operational.local.dd_trajectory, data.trajectory_settings.humanoid_fields, ddsupport_foot_traj, humanoid_part);
% 
%   
% %   
%   [ sf_traj, dsf_traj, ddsf_traj ] = support_foot_trajectory_generation( step_times, Ts, floating_foot);
%   trajectories.operational.local.trajectory   = insert_trajectory(trajectories.operational.local.trajectory,   data.trajectory_settings.humanoid_fields, sf_traj,  'SF');
%   trajectories.operational.local.d_trajectory = insert_trajectory(trajectories.operational.local.d_trajectory, data.trajectory_settings.humanoid_fields, dsf_traj, 'SF');
%   trajectories.operational.local.dd_trajectory = insert_trajectory(trajectories.operational.local.dd_trajectory, data.trajectory_settings.humanoid_fields, ddsf_traj, 'SF');
%   
%   % 3 - Full_Body
%   % Sumar a q(end) las trayectories de Foot y CoM
%   % 4 - pushbutton_ik2
%   ntrajdata_end = size(foot_traj.data,2) + ntrajdata - 1;
%   ntrajdata_foot = size(foot_traj.data,2) + ntrajdata - 1;
%   step_trajectory = extract_trajectory(trajectories.operational.local.trajectory,data.trajectory_settings.humanoid_fields, [step_times.Tinit step_times.Tend], 'time');
%   step_dtrajectory = extract_trajectory(trajectories.operational.local.d_trajectory,data.trajectory_settings.humanoid_fields, [step_times.Tinit step_times.Tend], 'time');
%     
% %   [q(:,ntrajdata:ntrajdata_end), dq(:,ntrajdata:ntrajdata_end), ddq(:,ntrajdata:ntrajdata_end)] = ik_full_body(q(:,ntrajdata), step_trajectory, step_dtrajectory, data.trajectory_settings.h, data.trajectory_settings.parameters);
% % [q(:,ntrajdata:ntrajdata_end), dq(:,ntrajdata:ntrajdata_end), ddq(:,ntrajdata:ntrajdata_end)] = ik_full_body_v2(q(:,ntrajdata), step_dtrajectory, data.trajectory_settings.h, data.trajectory_settings.parameters);
% 
%   if strcmp(floating_foot, 'Right'),
%     [q(:,ntrajdata:ntrajdata_end), dq(:,ntrajdata:ntrajdata_end), ddq(:,ntrajdata:ntrajdata_end)] = inverse_ds_ss_left_jacobian_quat(q(:,ntrajdata), step_trajectory, step_dtrajectory, data.trajectory_settings.h);
%   elseif strcmp(floating_foot, 'Left'),
%     [q(:,ntrajdata:ntrajdata_end), dq(:,ntrajdata:ntrajdata_end), ddq(:,ntrajdata:ntrajdata_end)] = inverse_ds_ss_right_jacobian_quat(q(:,ntrajdata), step_trajectory, step_dtrajectory, data.trajectory_settings.h);
%   else
%     error('ErrorTEOTraGen:wrongOption', 'Wrong Floating Foot option'); 
%   end
%   
%   
%   plot_footprint_local(footprints_poses.local(:,ii+1), floating_foot)
%   if strcmp(floating_foot, 'Right'),
%     floating_foot = 'Left';
%   elseif strcmp(floating_foot, 'Left'),
%     floating_foot ='Right';
%   else
%     error('ErrorTEOTraGen:wrongOption', 'Wrong Floating Foot option'); 
%   end
%   t0 = round_to_Ts(t0 + data.step_parameters.t_step, data.trajectory_settings.parameters.Ts);
%   ntrajdata =  ntrajdata_end;
% 
% trajectories.joints.q = q;
% trajectories.joints.dq = dq;
% trajectories.joints.ddq = ddq;






%%%%%%%%%%%%%%%%%%%%
%% Plot Functions %%
%%%%%%%%%%%%%%%%%%%%

function plot_footprint_local(current_footprint_pose, floating_foot)
global gui_plots data

if ~isfield(gui_plots,'last_footprint');
  if strcmp(data.step_parameters.init_floating_foot, 'Right')
%     footprint1 = zeros(6,1); footprint1(2,1) = -data.step_parameters.step_d;
    footprint2 = zeros(6,1); footprint2(2,1) = data.step_parameters.step_d;
  else
%     footprint1 = zeros(6,1); footprint1(2,1) = data.step_parameters.step_d;
    footprint2 = zeros(6,1); footprint2(2,1) = -data.step_parameters.step_d;
  end
  
  gui_plots.last_footprint = footprint2;
%   plot_footprint(footprint1(1:3), footprint1(6), 'green');
%   plot_footprint(footprint2(1:3), footprint2(6), 'green');
end

footprint_to_plot = pose_tr2rpy(multiply_homogeneous_matrix({pose_rpy2tr(gui_plots.last_footprint) pose_rpy2tr(current_footprint_pose)}));
plot_footprint(footprint_to_plot(1:3), footprint_to_plot(6), floating_foot, 'green');
gui_plots.last_footprint = footprint_to_plot;


function plot_CoM_trajectory()
% Plot CoM trajectory (Using inertial data)
global plotCoM
global trajectories

if (exist('trajectories','var') && isstruct(trajectories.operational.inertial.trajectory))
   if ~isempty(trajectories.operational.inertial.trajectory.CoM)
    if ishandle(plotCoM),
        delete(plotCoM);
    end

    hold on
    plotCoM = plot3(trajectories.operational.inertial.trajectory.CoM(1,:),trajectories.operational.inertial.trajectory.CoM(2,:),trajectories.operational.inertial.trajectory.CoM(3,:),'color','b');
    drawnow
    hold off
   else
    disp('PLOT_COM ERROR: There is not a CoM trajectory to plot');   
   end
else
    disp('PLOT_COM ERROR: There is not a trajectory structure to plot');
end


function plot_feet_trajectories()
% Plot Feet Trajectories (Using inertial data)

global plotRF_traj plotLF_traj
global trajectories

if (exist('trajectories','var') && isstruct(trajectories.operational.inertial.trajectory))
   if ~isempty(trajectories.operational.inertial.trajectory.RF)
    if ishandle(plotRF_traj),
        delete(plotRF_traj);
    end
    hold on
    plotRF_traj = plot3(trajectories.operational.inertial.trajectory.RF(1,:),trajectories.operational.inertial.trajectory.RF(2,:),trajectories.operational.inertial.trajectory.RF(3,:),'color','red');
    hold off
   else
    disp('PLOT_FEET ERROR: There is not a Right Foot trajectory to plot');   
   end
   if ~isempty(trajectories.operational.inertial.trajectory.LF)
    if ishandle(plotLF_traj),
        delete(plotLF_traj);
    end   
    hold on
    plotLF_traj = plot3(trajectories.operational.inertial.trajectory.LF(1,:),trajectories.operational.inertial.trajectory.LF(2,:),trajectories.operational.inertial.trajectory.LF(3,:),'color','red');
    drawnow
    hold off
   else
    disp('PLOT_FEET ERROR: There is not a Left Foot trajectory to plot');   
   end
else
    disp('PLOT_FEET ERROR: There is not a trajectory structure to plot');
end


function plot_median_zmp (median_zmp, color_plot)

if nargin < 2
  color_plot = 'm';
end

% Plot Median_ZMP trajectory
global plotMedianZMP

if ~isempty(median_zmp.inertial)
    if ishandle(plotMedianZMP),
        delete(plotMedianZMP);
    end

    for ii=1:size(median_zmp.inertial,2);
        hold on
        plotMedianZMP = plot(median_zmp.inertial(1,ii),median_zmp.inertial(2,ii),'X','linewidth',5,'color',color_plot);
        drawnow
        hold off
    end
else
    disp('PLOT_MEDIAN_ZMP ERROR: There is not a CoM trajectory to plot');
end


function h = plot_footprint(posfootprint, theta, floating_leg, color)
global data
% Plot a footprint specifying global X, Y and Theta

if nargin < 4
  color='k';
end

if strcmp(floating_leg,'Right')
  leg = 'right';
elseif strcmp(floating_leg,'Left')
  leg = 'left';
else
  disp('Wrong foot option');
end

% footwidth = 0.15;
% footlength = 0.25;
xmin = data.trajectory_settings.TEO.legs.(leg).foot.limits.x(1);
xmax = data.trajectory_settings.TEO.legs.(leg).foot.limits.x(2);
ymin = data.trajectory_settings.TEO.legs.(leg).foot.limits.y(1);
ymax = data.trajectory_settings.TEO.legs.(leg).foot.limits.y(2);

% footprintRF = rectangle('Position', [posfootprint(1)-footlength/2, posfootprint(2)-footwidth/2, footlength, footwidth]);
% X = [-footlength/2 footlength/2 footlength/2 -footlength/2 -footlength/2];
% Y = [footwidth/2 footwidth/2 -footwidth/2 -footwidth/2 footwidth/2];
X = [xmin xmax xmax xmin xmin];
Y = [ymax ymax ymin ymin ymax];

P = [X;Y];
ct = cos(theta);
st = sin(theta);
R = [ct -st;st ct];
P = R * P;

hold on
  h = plot(P(1,:) + posfootprint(1), P(2,:) + posfootprint(2),color);
  drawnow
hold off


function plot_world_frame(handles)
%En caso hubiera mas plots usar: axes(handles.axes1) para cambiar el gca

% WORLD COORDINATES
world_coord_length = 0.2;
worldX = line('color','red', 'LineWidth', 2);
set(worldX,'xdata', [0 world_coord_length], 'ydata', [0 0], 'zdata', [0 0]);
worldY = line('color','green', 'LineWidth', 2);
set(worldY,'xdata', [0 0], 'ydata', [0 world_coord_length], 'zdata', [0 0]);
worldZ = line('color','blue', 'LineWidth', 2);
set(worldZ,'xdata', [0 0], 'ydata', [0 0], 'zdata', [0 world_coord_length]);
    
% cones of the axes
[xc, yc, zc] = cylinder([0 world_coord_length/15]);
zc(zc==0) = world_coord_length + world_coord_length/15;
zc(zc==1) = world_coord_length - world_coord_length/15;
worldX_cone = surface(zc,yc,xc,'FaceColor', [1 0 0],'FaceAlpha', 1,'EdgeColor', 'none');
worldY_cone = surface(xc,zc,yc,'FaceColor', [0 1 0],'FaceAlpha', 1,'EdgeColor', 'none');
worldZ_cone = surface(xc,yc,zc,'FaceColor', [0 0 1],'FaceAlpha', 1,'EdgeColor', 'none');


function plot2dcircle(x,y,r)
hold on
  th = 0:pi/50:2*pi;

  xunit = r * cos(th) + x;
  yunit = r * sin(th) + y;

  h = plot(xunit, yunit,'color','g');
  drawnow
hold off


%%%%%%%%%%%%%%%%%%%%%%%
%% Buttons Callbacks %%
%%%%%%%%%%%%%%%%%%%%%%%

% --- Executes on button press in pushbutton_whole_trajectory.
function pushbutton_whole_trajectory_Callback(hObject, eventdata, handles)
global data trajectories

 trajectory_plots(trajectories.operational.inertial, trajectories.joints, data.trajectory_settings.TEO);
 %trajectory_plots(trajectories.operational.local, trajectories.joints, data.trajectory_settings.TEO);


function pushbutton_ROS_visualization_Callback(hObject, eventdata, handles)
global data trajectories
ros_visualization(trajectories.joints.q, trajectories.operational.local.trajectory.SF, data.trajectory_settings.parameters.Ts);


function pushbutton_local_motion_arms_Callback(hObject, eventdata, handles)



%%%%%%%%%%%%%%%%%%%
%% GUI functions %%
%%%%%%%%%%%%%%%%%%%

function update_floor_options(handles)
% Change the Floor (Panel) Limits and scale
xmin = str2double(get(handles.edit_xmin,'String'));
xmax = str2double(get(handles.edit_xmax,'String'));
ymin = str2double(get(handles.edit_ymin,'String'));
ymax = str2double(get(handles.edit_ymax,'String'));
axis(handles.axes_flat_floor, [xmin xmax ymin ymax]);
xscale = str2double(get(handles.edit_xscale,'String'));
yscale = str2double(get(handles.edit_yscale,'String'));
set(handles.axes_flat_floor,'XTick',xmin:xscale:xmax);
set(handles.axes_flat_floor,'YTick',ymin:yscale:ymax);


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
end


function pushbutton_reset_nsteps_Callback(hObject, eventdata, handles)
global footprints_poses median_zmp 
global trajectories
global gui_plots
global data

global Points thetas p 
cla (handles.axes_flat_floor)
plot_world_frame(handles)
% Plot footprints
posfootprintRF.inertial = [0, -data.step_parameters.step_d, 0];
posfootprintLF.inertial = [0, data.step_parameters.step_d, 0];

plot_footprint(posfootprintRF.inertial,0,'Right');
plot_footprint(posfootprintLF.inertial,0,'Left');
footprints_poses.local = [];
footprints_poses.inertial = [];
median_zmp.inertial = [];
median_zmp.local = [];
trajectories.operational.inertial.trajectory = [];
trajectories.operational.inertial.d_trajectory = [];
trajectories.operational.inertial.dd_trajectory = [];
trajectories.operational.local.trajectory = [];
trajectories.operational.local.d_trajectory = [];
trajectories.operational.local.dd_trajectory = [];
trajectories.joints = [];

Points = [0 0 0];
thetas = 0;
p = [];

gui_plots = [];

% Disable Plots buttons
set(handles.pushbutton_whole_trajectory,'Enable','off')
set(handles.pushbutton_plot_feet_trajectory,'Enable','off')


function ok = check_all_input_value(handles)
global data
% Check all introduced values
if ~check_input_value(str2double(get(handles.edit_nsteps,'String')), 'Number of Steps')
  ok = 0; 
  return;
else
  data.step_parameters.nsteps = str2double(get(handles.edit_nsteps,'String'));
end

if ~check_input_value(str2double(get(handles.edit_step_length,'String')), 'Step Length', 'nan')
  ok = 0;
  return;
else
  data.step_parameters.step_length = str2double(get(handles.edit_step_length,'String'));
end

if ~check_input_value(str2double(get(handles.edit_step_height,'String')), 'Step Height')
  ok = 0;
  return;
else
  data.step_parameters.step_height = str2double(get(handles.edit_step_height,'String'));
end

if ~check_input_value(str2double(get(handles.edit_theta_variation,'String')), 'Theta Variation','nan')
  ok = 0;
  return;
else
  data.step_parameters.theta_variation = str2double(get(handles.edit_theta_variation,'String'));  
end

if ~check_input_value(str2double(get(handles.edit_step_d,'String')), '"d" Variation','nan')
  ok = 0;
  return;
else
  data.step_parameters.step_d = str2double(get(handles.edit_step_d,'String'));  
end

if ~check_input_value(str2double(get(handles.edit_double_support_percentage,'String'))/100, 'Double Step (DS) percentage')
  ok = 0;
  return;
else
  data.step_parameters.ds_percentage = str2double(get(handles.edit_double_support_percentage,'String'))/100;  
end

if ~check_input_value(str2double(get(handles.edit_ts,'String')), 'Sample Time (Ts)')
  ok = 0;
  return;
else
  data.trajectory_settings.parameters.Ts = str2double(get(handles.edit_ts,'String')); 
end

if ~check_input_value(str2double(get(handles.edit_cog_zc,'String')), 'Height of Center of Gravity (zc)')
  ok = 0;
  return;
else
  data.cog_parameters.zc = str2double(get(handles.edit_cog_zc,'String'));
end

if ~check_input_value(str2double(get(handles.edit_cog_lambda,'String')), 'Lambda','nan')
  ok = 0;
  return;
else
  data.cog_parameters.lambda = str2double(get(handles.edit_cog_lambda,'String'));
end

if ~check_input_value(str2double(get(handles.edit_cog_beta,'String')), 'Beta','nan')
  ok = 0;
  return;
else
  data.cog_parameters.beta = str2double(get(handles.edit_cog_beta,'String'));
end

if ~check_input_value(str2double(get(handles.edit_cog_alpha,'String')), 'Alpha','nan')
  ok = 0;
  return;
else
  data.cog_parameters.alpha = str2double(get(handles.edit_cog_alpha,'String'));  
end

% Select the model to generate the COG trajectory
cog_list_options = cellstr(get(handles.popupmenu_local_motion_cog,'String'));
data.cog_parameters.cog_option = cog_list_options{get(handles.popupmenu_local_motion_cog,'Value')};

% Select the model to generate the Foot trajectory
foot_list_options = cellstr(get(handles.popupmenu_local_motion_feet,'String'));
data.step_parameters.foot_option = foot_list_options{get(handles.popupmenu_local_motion_feet,'Value')};

% Select the model to generate the Arms trajectory
arms_list_options = cellstr(get(handles.popupmenu_local_motion_arms,'String'));
data.step_parameters.arms_option = arms_list_options{get(handles.popupmenu_local_motion_arms,'Value')};

ok = 1;



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Plot global trajectory functions%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function plot_local_frame(handles, initial, theta, axes_size)
%En caso hubiera mas plots usar: axes(handles.axes1) para cambiar el gca

% Param
if nargin <4
  axes_size = 0.1;
end

x_var = [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0; 0 0 1] * [axes_size; 0; 0];
y_var = [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0; 0 0 1] * [0; axes_size; 0];
z_var = [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0; 0 0 1] * [0; 0; axes_size];


% Local Coordinates
worldX = line('color','red', 'LineWidth', 2);
set(worldX,'xdata', [initial(1) initial(1)+x_var(1)], 'ydata', [initial(2) initial(2)+x_var(2)], 'zdata', [initial(3) initial(3)+x_var(3)]);
worldY = line('color','green', 'LineWidth', 2);
set(worldY,'xdata', [initial(1) initial(1)+y_var(1)], 'ydata', [initial(2) initial(2)+y_var(2)], 'zdata', [initial(3) initial(3)+y_var(3)]);
worldZ = line('color','blue', 'LineWidth', 2);
set(worldZ,'xdata', [initial(1) initial(1)+z_var(1)], 'ydata', [initial(2) initial(2)+z_var(2)], 'zdata', [initial(3) initial(3)+z_var(3)]);
%     
% % cones of the axes
% [xc, yc, zc] = cylinder([0 world_coord_length/15]);
% zc(zc==0) = world_coord_length + world_coord_length/15;
% zc(zc==1) = world_coord_length - world_coord_length/15;
% worldX_cone = surface(zc,yc,xc,'FaceColor', [1 0 0],'FaceAlpha', 1,'EdgeColor', 'none');
% worldY_cone = surface(xc,zc,yc,'FaceColor', [0 1 0],'FaceAlpha', 1,'EdgeColor', 'none');
% worldZ_cone = surface(xc,yc,zc,'FaceColor', [0 0 1],'FaceAlpha', 1,'EdgeColor', 'none');


function finish_plot()
set(gcf,'windowbuttonmotionfcn','');
set(gcf,'windowbuttonupfcn','');
set(gcf,'windowbuttondownfcn','');
set(gcf,'keypressfcn','');


function done_pencil(src,evendata)
%all this funciton does is turn the motion function off 
set(gcf,'windowbuttonmotionfcn','')
set(gcf,'windowbuttonupfcn','')


%%%%%%%%%%%%%%%%%%
%% Menu Buttons %%
%%%%%%%%%%%%%%%%%%

function back_Callback(hObject, eventdata, handles)
% hObject    handle to back (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


function back_main_menu_Callback(hObject, eventdata, handles)
TEOTraGen
close footprint_generation


%%%%%%%%%%%%%%%%%%%%%
%% others hObjects %%
%%%%%%%%%%%%%%%%%%%%%

function edit_step_d_Callback(hObject, eventdata, handles)

function edit_step_d_CreateFcn(hObject, eventdata, handles)
  if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
      set(hObject,'BackgroundColor','white');
  end
  
  
function popupmenu_local_motion_arms_Callback(hObject, eventdata, handles)

function popupmenu_local_motion_arms_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end





function edit_nsteps_Callback(hObject, eventdata, handles)

function edit_nsteps_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit_theta_variation_Callback(hObject, eventdata, handles)

function edit_theta_variation_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function popupmenu_init_sf_Callback(hObject, eventdata, handles)
global data
contents = cellstr(get(hObject,'String'));
data.step_parameters.init_floating_foot = contents{get(hObject,'Value')};

function popupmenu_init_sf_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function popupmenu_local_motion_feet_Callback(hObject, eventdata, handles)

function popupmenu_local_motion_feet_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function checkbox_finish_both_feet_Callback(hObject, eventdata, handles)
global data
data.step_parameters.finish_both_feet = get(hObject,'Value');


function edit_step_length_Callback(hObject, eventdata, handles)

function edit_step_length_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit_step_height_Callback(hObject, eventdata, handles)

function edit_step_height_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit_time_step_Callback(hObject, eventdata, handles)

function edit_time_step_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit_ts_Callback(hObject, eventdata, handles)

function edit_ts_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit_double_support_percentage_Callback(hObject, eventdata, handles)

function edit_double_support_percentage_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit_cog_lambda_Callback(hObject, eventdata, handles)

function edit_cog_lambda_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit_cog_alpha_Callback(hObject, eventdata, handles)

function edit_cog_alpha_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit_cog_beta_Callback(hObject, eventdata, handles)

function edit_cog_beta_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit_cog_zc_Callback(hObject, eventdata, handles)

function edit_cog_zc_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit_cog_g_Callback(hObject, eventdata, handles)

function edit_cog_g_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit26_Callback(hObject, eventdata, handles)

function edit26_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function popupmenu_local_motion_cog_Callback(hObject, eventdata, handles)

function popupmenu_local_motion_cog_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes during object creation, after setting all properties.
function edit_global_trajectory_time_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_global_trajectory_time (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



% --- Executes on key press with focus on pushbutton22 and none of its controls.
function pushbutton22_KeyPressFcn(hObject, eventdata, handles)
% hObject    handle to pushbutton22 (see GCBO)
% eventdata  structure with the following fields (see UICONTROL)
%	Key: name of the key that was pressed, in lower case
%	Character: character interpretation of the key(s) that was pressed
%	Modifier: name(s) of the modifier key(s) (i.e., control, shift) pressed
% handles    structure with handles and user data (see GUIDATA)


function view_Callback(hObject, eventdata, handles)


function flat_floor_options_Callback(hObject, eventdata, handles)
handles.floor_options  = change_floor_options(handles.floor_options, handles.axes_flat_floor);
guidata(hObject,handles);


function pushbutton_matlab_visualization_Callback(hObject, eventdata, handles)
global trajectories data
  matlab_visualization (trajectories.operational.trajectory, trajectories.joints.q, data.trajectory_settings.TEO, data.trajectory_settings.h)  

  
function ik_gains_Callback(hObject, eventdata, handles)
global data
  [data.trajectory_settings.parameters.kp data.trajectory_settings.parameters.ko] = change_ik_gains(data.trajectory_settings.parameters.kp, data.trajectory_settings.parameters.ko);


% --------------------------------------------------------------------
function cart_table_model_Callback(hObject, eventdata, handles)
% hObject    handle to cart_table_model (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton_local_motion_zmp.
function pushbutton_local_motion_zmp_Callback(hObject, eventdata, handles)
% global trajectories
% global footprints_SF footprints_poses
% global median_zmp
% current_time = 0;
% 
% global zmp_trajectory_CartTable
% 
% zmp_points = zeros(2,size(footprints_poses.inertial,2)-1);
% zmp_trajectory_CartTable = zeros(2,size(trajectory.CoM,2));
% 
% zmp_points(:,1) = median_zmp.inertial(:,1);
% 
% for ii=2:size(footprints_poses.inertial,2)
%     p1_zmp = median_zmp.inertial(1:2,ii-2);
%     p2_zmp = footprints_poses.inertial(1:2,ii-1);
%     p3_zmp = median_zmp.inertial(1:2,ii-1);
%    
%     init_time = current_time;
%     end_time = init_time + data.step_parameters.t_step/2;
%     
%     DM=[p1_zmp;p2_zmp];
%     
%     current_time = ff_time + data.step_parameters.t_step/2;
%     
% %     zmp_trajectory(1,:) = p1_zmp(1):1/data.trajectory_settings.parameters.Ts:p2_zmp(1);
% %     zmp_trajectory(2,:) = p1_zmp(2):1/data.trajectory_settings.parameters.Ts:p2_zmp(2);
% end
% 
% hold on
% plot3(zmp_trajectory(1,:), zmp_trajectory(2,:), zeros(1,size(zmp_trajectory,2)), 'color', 'm');
% hold off

global footprints_poses median_zmp
global zmp_trajectory zmp_trajectoryX zmp_trajectoryXX
global data

current_time = 0;
data.step_parameters.ds_percentage = str2double(get(handles.edit_double_support_percentage,'String'))/100;
if isnan(data.step_parameters.ds_percentage)
  data.step_parameters.ds_percentage = 0.20;
end

% SI NO TERMINA EN DS DEBERA TENER UNA COLUMNA MENOS
zmp_points = zeros(2,2*size(footprints_poses.inertial,2));


ds_time = round_to_Ts(data.step_parameters.ds_percentage*data.step_parameters.t_step, data.trajectory_settings.parameters.Ts);
ss_time = round_to_Ts(data.step_parameters.t_step - ds_time, data.trajectory_settings.parameters.Ts);


global P
for ii=2:size(footprints_poses.inertial,2)-1
  if ii == 2
    P = set_trajectory_condition(current_time, [median_zmp.inertial(1:2,ii-1);0], zeros(3,1), zeros(3,1));
  end
current_time = current_time + ds_time/2;
P1 = set_trajectory_condition(current_time, [footprints_poses.inertial(1:2,ii);0], zeros(3,1), zeros(3,1));
current_time = current_time + ss_time;
P2 = set_trajectory_condition(current_time, [footprints_poses.inertial(1:2,ii);0], zeros(3,1), zeros(3,1));
current_time = current_time + ds_time/2;
P3 = set_trajectory_condition(current_time, [median_zmp.inertial(1:2,ii);0], zeros(3,1), zeros(3,1));
P  = [P P1 P2 P3];
end
[zmp_trajectory, dx, ddx] = poly5_viapoints_trajectory (P, data.trajectory_settings.parameters.Ts);
[zmp_trajectoryX, dxX, ddxX] = poly3_viapoints_trajectory (P, data.trajectory_settings.parameters.Ts);
[zmp_trajectoryXX, dxXX, ddxXX] = poly7_viapoints_trajectory (P, data.trajectory_settings.parameters.Ts);
hold on
  plot3(zmp_trajectory.data(1,:), zmp_trajectory.data(2,:), zmp_trajectory.data(3,:), 'color', 'm');
  drawnow
hold off
