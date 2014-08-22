function varargout = save_trajectory(varargin)
% SAVE_TRAJECTORY MATLAB code for save_trajectory.fig
%      SAVE_TRAJECTORY, by itself, creates a new SAVE_TRAJECTORY or raises the existing
%      singleton*.
%
%      H = SAVE_TRAJECTORY returns the handle to a new SAVE_TRAJECTORY or the handle to
%      the existing singleton*.
%
%      SAVE_TRAJECTORY('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in SAVE_TRAJECTORY.M with the given input arguments.
%
%      SAVE_TRAJECTORY('Property','Value',...) creates a new SAVE_TRAJECTORY or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before save_trajectory_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to save_trajectory_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help save_trajectory

% Last Modified by GUIDE v2.5 18-Sep-2013 18:35:04

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @save_trajectory_OpeningFcn, ...
                   'gui_OutputFcn',  @save_trajectory_OutputFcn, ...
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


% --- Executes just before save_trajectory is made visible.
function save_trajectory_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to save_trajectory (see VARARGIN)

% Choose default command line output for save_trajectory
handles.output = hObject;

movegui('center')

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes save_trajectory wait for user response (see UIRESUME)
% uiwait(handles.save_format);


% --- Outputs from this function are returned to the command line.
function varargout = save_trajectory_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on selection change in format_popup.
function format_popup_Callback(hObject, eventdata, handles)
% hObject    handle to format_popup (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns format_popup contents as cell array
%        contents{get(hObject,'Value')} returns selected item from format_popup


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
option=get(handles.format_popup,'Value');
save{1}='trajectory';
save{2}='d_trajectory';
save{3}='dd_trajectory';
if option==2
    for i=1:3
        Q = handles.(['save_' (save{i})]);
        [m,n] = size(Q);
        %Qtext = [Q(1:6,:);Q(14:17,:);Q(7:12,:);Q(19:22,:);Q(13,:);zeros(1,n);Q(18,:)];
        [file,path] = uiputfile([handles.traj_name_string '_' save{i} '.txt'],['Save ' save{i} ' as:']);
        dlmwrite(file,Q,'delimiter','\t','precision','%.6f')
    end
end
