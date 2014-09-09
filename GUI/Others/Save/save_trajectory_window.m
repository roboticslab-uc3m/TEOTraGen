function varargout = save_trajectory_window(varargin)
% SAVE_TRAJECTORY_WINDOW MATLAB code
% Author: Domingo Esteban
%
% RobitcsLab, Universidad Carlos III de Madrid
% 2013

% Edit the above text to modify the response to help save_trajectory_window

% Last Modified by GUIDE v2.5 18-Sep-2013 19:24:01

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @save_trajectory_window_OpeningFcn, ...
                   'gui_OutputFcn',  @save_trajectory_window_OutputFcn, ...
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


% --- Executes just before save_trajectory_window is made visible.
function save_trajectory_window_OpeningFcn(hObject, eventdata, handles, varargin)

% Choose default command line output for save_trajectory_window
handles.output = hObject;

movegui(gcf,'center')


% Evaluate Input Data
if isempty(varargin)
  warning('No inputs for save_trajectory_window.');
  close(gcf)
%   settings_trajectory_generation;
  return;
else
  if strcmp(varargin{5},'operational')
    handles.traj_name_string = varargin{1};
    handles.save_traj = varargin{2};
    handles.save_dtraj = varargin{3};
    handles.save_ddtraj = varargin{4};
    handles.trajectory_space = varargin{5};
    handles.trajectory_to_save{1} = 'traj';
    handles.trajectory_to_save{2} = 'dtraj';
    handles.trajectory_to_save{3} = 'ddtraj';
        
  elseif strcmp(varargin{5},'joints')
    handles.traj_name_string = varargin{1};
    handles.save_q = varargin{2};
    handles.save_dq = varargin{3};
    handles.save_ddq = varargin{4};
    handles.save_traj = [];
    handles.save_dtraj = [];
    handles.save_ddtraj = [];
    handles.trajectory_space = varargin{5};
    handles.trajectory_to_save{1} = 'q';
    handles.trajectory_to_save{2} = 'dq';
    handles.trajectory_to_save{3} = 'ddq';
  else
    errordlg('Wrong option to save','Save Error')
    return
  end
end

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes save_trajectory_window wait for user response (see UIRESUME)
% uiwait(handles.save_format);


% --- Outputs from this function are returned to the command line.
function varargout = save_trajectory_window_OutputFcn(hObject, eventdata, handles) 

varargout{1} = handles.output;


% --- Executes on selection change in format_popup.
function format_popup_Callback(hObject, eventdata, handles)



% --- Executes during object creation, after setting all properties.
function format_popup_CreateFcn(hObject, eventdata, handles)
% hObject    handle to format_popup (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in save_pushbutton.
function save_pushbutton_Callback(hObject, eventdata, handles)
option = get(handles.format_popup,'Value');

if (isempty(handles.save_traj) && isempty(handles.save_dtraj) && isempty(handles.save_ddtraj))
  q = handles.save_q;
  dq = handles.save_dq;
  ddq = handles.save_ddq;

  try
      if option == 1 % .csv format - Comma
          [m,n] = size(q);
          [file1,path] = uiputfile([handles.traj_name_string '_q.csv'], 'Save Joint Angles as');
          csvid = fopen(file1, 'w');
          fprintf(csvid, '%1.4f,%1.4f,%1.4f,%1.4f,%1.4f,%1.4f,%1.4f,%1.4f,%1.4f,%1.4f,%1.4f,%1.4f,%1.4f,%1.4f,%1.4f,%1.4f,%1.4f,%1.4f,%1.4f,%1.4f,%1.4f,%1.4f,%1.4f,%1.4f,%1.4f,%1.4f\n', q);
          fclose(csvid);

          [file2,path] = uiputfile([handles.traj_name_string '_dq.csv'], 'Save Joint Velocities as');
          csvid = fopen(file2, 'w');
          fprintf(csvid, '%1.4f,%1.4f,%1.4f,%1.4f,%1.4f,%1.4f,%1.4f,%1.4f,%1.4f,%1.4f,%1.4f,%1.4f,%1.4f,%1.4f,%1.4f,%1.4f,%1.4f,%1.4f,%1.4f,%1.4f,%1.4f,%1.4f,%1.4f,%1.4f,%1.4f,%1.4f\n', dq);
          fclose(csvid);

          [file3,path] = uiputfile([handles.traj_name_string '_ddq.csv'], 'Save Joint Accelerations as');
          csvid = fopen(file3, 'w');
          fprintf(csvid, '%1.4f,%1.4f,%1.4f,%1.4f,%1.4f,%1.4f,%1.4f,%1.4f,%1.4f,%1.4f,%1.4f,%1.4f,%1.4f,%1.4f,%1.4f,%1.4f,%1.4f,%1.4f,%1.4f,%1.4f,%1.4f,%1.4f,%1.4f,%1.4f,%1.4f,%1.4f\n', ddq);
          fclose(csvid);

      elseif option == 2 % .txt format
          for i = 1:3
            data_to_save = handles.(strcat('save_', handles.trajectory_to_save{i}));
  %           [m,n] = size(Q);
            Q = data_to_save';
            [file, path] = uiputfile([handles.traj_name_string '_' handles.trajectory_to_save{i} '.txt'],['Save ' handles.trajectory_to_save{i} ' as:']);
            dlmwrite(file, Q, 'delimiter','\t','precision','%.4f')
          end
      elseif option == 3 % Custom format
          [m,n] = size(q);
          [file1,path] = uiputfile([handles.traj_name_string '_q.csv'], 'Save Joint Angles as');
          csvid = fopen(file1, 'w');
          fprintf(csvid, '%1.2f %1.2f %1.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %1.2f\n',...
                                                                                                                                     [q(6,:); q(5,:); q(4,:); q(3,:); q(2,:); q(1,:); q(7:end,:)]);
          fclose(csvid);

          [file2,path] = uiputfile([handles.traj_name_string '_dq.csv'], 'Save Joint Velocities as');
          csvid = fopen(file2, 'w');
          fprintf(csvid, '%.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %1.2f\n',...
                                                                                                                                     [dq(6,:); dq(5,:); dq(4,:); dq(3,:); dq(2,:); dq(1,:); dq(7:end,:)]);
          fclose(csvid);

          [file3,path] = uiputfile([handles.traj_name_string '_ddq.csv'], 'Save Joint Accelerations as');
          csvid = fopen(file3, 'w');
          fprintf(csvid, '%.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %1.2f\n',...
                                                                                                                                     [ddq(6,:); ddq(5,:); ddq(4,:); ddq(3,:); ddq(2,:); ddq(1,:); ddq(7:end,:)]);
          fclose(csvid);      
      end
  catch
      disp('Save trajectories/joints canceled');
  end
else
  disp('Save operational trajectory no implemented');
end
close(gcf)
