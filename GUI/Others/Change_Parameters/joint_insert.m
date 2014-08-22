function varargout = joint_insert(varargin)
% JOINT_INSERT MATLAB code for joint_insert.fig
%      JOINT_INSERT, by itself, creates a new JOINT_INSERT or raises the existing
%      singleton*.
%
%      H = JOINT_INSERT returns the handle to a new JOINT_INSERT or the handle to
%      the existing singleton*.
%
%      JOINT_INSERT('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in JOINT_INSERT.M with the given input arguments.
%
%      JOINT_INSERT('Property','Value',...) creates a new JOINT_INSERT or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before joint_insert_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to joint_insert_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help joint_insert

% Last Modified by GUIDE v2.5 19-Apr-2014 00:03:29

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @joint_insert_OpeningFcn, ...
                   'gui_OutputFcn',  @joint_insert_OutputFcn, ...
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


% --- Executes just before joint_insert is made visible.
function joint_insert_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to joint_insert (see VARARGIN)

% Choose default command line output for joint_insert
handles.output = hObject;
humanoid_part = varargin{1};
q0 = varargin{2};
humanoid = varargin{3};

switch humanoid_part
  case 'RH'
   set(handles.text_title,'String','Right Arm');
   joint_humanoid_part = humanoid.arms.right.joint;
   humanoid_part_position = 15:20;
  case 'LH'
   set(handles.text_title,'String','Left Arm');
   joint_humanoid_part = humanoid.arms.left.joint;
   humanoid_part_position = 21:26;
  case 'RF'
   set(handles.text_title,'String','Right Leg');
   joint_humanoid_part = humanoid.legs.right.joint;
   humanoid_part_position = 1:6;
  case 'LF'
   set(handles.text_title,'String','Left Leg');
   joint_humanoid_part = humanoid.legs.left.joint;
   humanoid_part_position = 7:12;
end

q = q0(humanoid_part_position);

for jj = 1:6
 set(handles.(strcat('text_',num2str(1+jj))),'String', joint_humanoid_part(jj).name)
 set(handles.(strcat('joint_slider',num2str(jj))),...
     'Min',joint_humanoid_part(jj).angle_limits(1),...
     'Max',joint_humanoid_part(jj).angle_limits(2),...
     'SliderStep',[0.001 0.1]/(diff(joint_humanoid_part(jj).angle_limits)));
 set(handles.(strcat('joint_slider',num2str(jj))), 'Value',q(jj));
 set(handles.(strcat('joint_val',num2str(jj))), 'String',q(jj));
 set(handles.(strcat('joint_min',num2str(jj))),'String',strcat('Min: ', num2str(joint_humanoid_part(jj).angle_limits(1)),' rad'));
 set(handles.(strcat('joint_max',num2str(jj))),'String',strcat('Max: ', num2str(joint_humanoid_part(jj).angle_limits(2)),' rad'));
 
end

handles.humanoid_part = humanoid_part;
handles.humanoid = humanoid;
handles.humanoid_part_position = humanoid_part_position;
handles.q = q;
handles.q0 = q0;
handles.joint_humanoid_part = joint_humanoid_part;
guidata(hObject, handles);

plot_humanoid_part(hObject,handles)

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes joint_insert wait for user response (see UIRESUME)
uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = joint_insert_OutputFcn(hObject, eventdata, handles) 
varargout{1} = handles.q0;
close(gcf);


function joint_slider1_Callback(hObject, eventdata, handles)
update_slider_value(hObject,handles,'joint_val1')

handles.q(1) = (get(hObject,'Value'));
guidata(hObject,handles)

update_humanoid_plot(hObject,handles)
guidata(hObject,handles)


function joint_slider1_CreateFcn(hObject, eventdata, handles)
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
  set(hObject,'BackgroundColor',[.9 .9 .9]);
end

function joint_val1_Callback(hObject, eventdata, handles)
  val2update_slider_value(hObject,handles,'joint_slider1', 1)
guidata(hObject,handles)

function joint_val1_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
  set(hObject,'BackgroundColor','white');
end


function text_2_Callback(hObject, eventdata, handles)

function text_2_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function joint_slider2_Callback(hObject, eventdata, handles)
update_slider_value(hObject,handles,'joint_val2')

handles.q(2) = (get(hObject,'Value'));
guidata(hObject,handles)

update_humanoid_plot(hObject,handles)
guidata(hObject,handles)

function joint_slider2_CreateFcn(hObject, eventdata, handles)
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

function joint_val2_Callback(hObject, eventdata, handles)
val2update_slider_value(hObject,handles,'joint_slider2', 2)
guidata(hObject,handles)

function joint_val2_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function text_3_Callback(hObject, eventdata, handles)

function text_3_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function joint_slider3_Callback(hObject, eventdata, handles)
update_slider_value(hObject,handles,'joint_val3')

handles.q(3) = (get(hObject,'Value'));
guidata(hObject,handles)

update_humanoid_plot(hObject,handles)
guidata(hObject,handles)

function joint_slider3_CreateFcn(hObject, eventdata, handles)
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

function joint_val3_Callback(hObject, eventdata, handles)
val2update_slider_value(hObject,handles,'joint_slider3', 3)
guidata(hObject,handles)

function joint_val3_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function text_4_Callback(hObject, eventdata, handles)
 
function text_4_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function joint_slider4_Callback(hObject, eventdata, handles)
update_slider_value(hObject,handles,'joint_val4')

handles.q(4) = (get(hObject,'Value'));
guidata(hObject,handles)

update_humanoid_plot(hObject,handles)
guidata(hObject,handles)

function joint_slider4_CreateFcn(hObject, eventdata, handles)
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

function joint_val4_Callback(hObject, eventdata, handles)
val2update_slider_value(hObject,handles,'joint_slider4', 4)
guidata(hObject,handles)

function joint_val4_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function text_5_Callback(hObject, eventdata, handles)

function text_5_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function joint_slider5_Callback(hObject, eventdata, handles)
update_slider_value(hObject,handles,'joint_val5')

handles.q(5) = (get(hObject,'Value'));
guidata(hObject,handles)

update_humanoid_plot(hObject,handles)
guidata(hObject,handles)

function joint_slider5_CreateFcn(hObject, eventdata, handles)
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

function joint_val5_Callback(hObject, eventdata, handles)
val2update_slider_value(hObject,handles,'joint_slider5', 5)
guidata(hObject,handles)

function joint_val5_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function text_6_Callback(hObject, eventdata, handles)

function text_6_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function joint_slider6_Callback(hObject, eventdata, handles)
update_slider_value(hObject,handles,'joint_val6')

handles.q(6) = get(hObject,'Value');
guidata(hObject,handles)

update_humanoid_plot(hObject,handles)
guidata(hObject, handles)

function joint_slider6_CreateFcn(hObject, eventdata, handles)
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

function joint_val6_Callback(hObject, eventdata, handles)
val2update_slider_value(hObject,handles,'joint_slider6', 6)
guidata(hObject,handles)

function joint_val6_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function text_7_Callback(hObject, eventdata, handles)

function text_7_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function save_button_Callback(hObject, eventdata, handles)

val = [str2double(get(handles.joint_val1,'String'));str2double(get(handles.joint_val2,'String'));str2double(get(handles.joint_val3,'String'));str2double(get(handles.joint_val4,'String'));str2double(get(handles.joint_val5,'String'));str2double(get(handles.joint_val6,'String'))];

handles.q0(handles.humanoid_part_position,1) = val;
guidata(hObject, handles);

if isequal(get(gcf, 'waitstatus'), 'waiting')
    % The GUI is still in UIWAIT, us UIRESUME
    uiresume(gcf);
else
    % The GUI is no longer waiting, just close it
    close(gcf);
end




function update_slider_value(hObject,handles,text_val)
 set(handles.(text_val),'String',num2str(get(hObject,'Value')));
guidata(hObject,handles);

function val2update_slider_value(hObject,handles,text_slider, njoint)
%
val = str2double(get(hObject,'String'));
min_val = get(handles.(text_slider),'Min');
max_val = get(handles.(text_slider),'Max');
if and(val>=min_val,val<=max_val)
    set(handles.(text_slider),'Value',val);
    handles.q(njoint) = val;
else
    errordlg('Out of range value','Error')
    return
end
update_humanoid_plot(hObject, handles)
guidata(hObject,handles)


function plot_humanoid_part(hObject, handles)
global TEO_humanoid_part
q = handles.q;

q_vector = generate_symbolic_vector('theta', 6);
humanoid = handles.humanoid;  
switch handles.humanoid_part
  case 'RH'
    Rotation_Matrix_Plot_Arms =  r2t([1 0 0;...
                        0 -1 0;...
                        0 0 -1]);
    [humanoid_arm.joint, humanoid_arm.n_joints] = humanoid_arm_DH_parameters(humanoid.arms.link_lengths(2:3), q_vector);
    for i=1:6
      TEO_right_arm(i) = Link(double([humanoid_arm.joint(i).theta-q_vector(i), humanoid_arm.joint(i).d, humanoid_arm.joint(i).a, humanoid_arm.joint(i).alpha, 0]));
    end
    TEO_humanoid_part = SerialLink(TEO_right_arm, 'name', 'TEO_Right_Arm', 'base', transl(humanoid.arms.right.joint(1).origin)*Rotation_Matrix_Plot_Arms);

  case 'LH'
    Rotation_Matrix_Plot_Arms =  r2t([1 0 0;...
                        0 -1 0;...
                        0 0 -1]);
    [humanoid_arm.joint, humanoid_arm.n_joints] = humanoid_arm_DH_parameters(humanoid.arms.link_lengths(2:3), q_vector);
    for i = 1:6
      TEO_right_arm(i) = Link(double([humanoid_arm.joint(i).theta-q_vector(i), humanoid_arm.joint(i).d, humanoid_arm.joint(i).a, humanoid_arm.joint(i).alpha, 0]));
    end
    TEO_humanoid_part = SerialLink(TEO_right_arm, 'name', 'TEO_Left_Arm', 'base', transl(humanoid.arms.right.joint(1).origin)*Rotation_Matrix_Plot_Arms);

  case 'RF'
    Rotation_Matrix_Plot_Legs =  r2t([1 0 0;...
                        0 1 0;...
                        0 0 -1]);
    [humanoid_leg.joint, humanoid_leg.n_joints] = humanoid_arm_DH_parameters(humanoid.legs.link_lengths(2:3), q_vector);
    for i = 1:6
      TEO_right_leg(i) = Link(double([humanoid_leg.joint(i).theta-q_vector(i), humanoid_leg.joint(i).d, humanoid_leg.joint(i).a, humanoid_leg.joint(i).alpha, 0]));
    end
    TEO_humanoid_part = SerialLink(TEO_right_leg, 'name', 'TEO_Right_Leg', 'base', transl(humanoid.legs.right.joint(1).origin)*Rotation_Matrix_Plot_Legs);

  case 'LF'
    Rotation_Matrix_Plot_Legs =  r2t([1 0 0;...
                        0 1 0;...
                        0 0 -1]);
    [humanoid_leg.joint, humanoid_leg.n_joints] = humanoid_arm_DH_parameters(humanoid.legs.link_lengths(2:3), q_vector);
    for i = 1:6
      TEO_left_leg(i) = Link(double([humanoid_leg.joint(i).theta-q_vector(i), humanoid_leg.joint(i).d, humanoid_leg.joint(i).a, humanoid_leg.joint(i).alpha, 0]));
    end
    TEO_humanoid_part = SerialLink(TEO_left_leg, 'name', 'TEO_Right_Leg', 'base', transl(humanoid.legs.left.joint(1).origin)*Rotation_Matrix_Plot_Legs);
    
    
  otherwise
    disp('Error humanoid_part input');

end

axes(handles.axes1)
TEO_humanoid_part.plot(q','nobase','noshadow','nojaxes','noname','nowrist')

set(gca,'XDir','reverse');
set(gca,'YDir','reverse');
axis([-1.25 1.25 -1.25 1.25 -0.5 1.8])

hold on
plot_world_frame(handles)
hold off

guidata(hObject,handles)


function update_humanoid_plot(hObject, handles)
global TEO_humanoid_part
% TEO_humanoid_part = handles.TEO_humanoid_part;
TEO_humanoid_part.plot(handles.q');
drawnow
guidata(hObject,handles)

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


function pushbutton_cancel_Callback(hObject, eventdata, handles)
if isequal(get(gcf, 'waitstatus'), 'waiting')
    % The GUI is still in UIWAIT, us UIRESUME
    uiresume(gcf);
else
    % The GUI is no longer waiting, just close it
    close(gcf);
end
