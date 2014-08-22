function varargout = change_ik_gains(varargin)
% CHANGE_IK_GAINS MATLAB code for change_ik_gains.fig
%      CHANGE_IK_GAINS, by itself, creates a new CHANGE_IK_GAINS or raises the existing
%      singleton*.
%
%      H = CHANGE_IK_GAINS returns the handle to a new CHANGE_IK_GAINS or the handle to
%      the existing singleton*.
%
%      CHANGE_IK_GAINS('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in CHANGE_IK_GAINS.M with the given input arguments.
%
%      CHANGE_IK_GAINS('Property','Value',...) creates a new CHANGE_IK_GAINS or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before change_ik_gains_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to change_ik_gains_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help change_ik_gains

% Last Modified by GUIDE v2.5 30-Mar-2014 17:42:27

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @change_ik_gains_OpeningFcn, ...
                   'gui_OutputFcn',  @change_ik_gains_OutputFcn, ...
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


function change_ik_gains_OpeningFcn(hObject, eventdata, handles, varargin)
% Choose default command line output for change_ik_gains
handles.output = hObject;

if isempty(varargin)
  warning('WarnTEOTraGen:noInputGUI', 'No inputs for change_ik_gains');
  handles.kp = 0;
  handles.ko = 0;
else
  % Get previous panels
  handles.kp = varargin{1};
  handles.ko = varargin{2};
end

set(handles.edit_kp,'String', handles.kp);
set(handles.edit_ko,'String', handles.ko);

% Update handles structure
guidata(hObject, handles);

% UIWAIT
uiwait(handles.figure1);


function varargout = change_ik_gains_OutputFcn(hObject, eventdata, handles)
  varargout{1} = handles.kp;
  varargout{2} = handles.ko;
  close(gcf);



function pushbutton_ok_Callback(hObject, eventdata, handles)
handles.kp = str2double(get(handles.edit_kp,'String'));
handles.ko = str2double(get(handles.edit_ko,'String'));
guidata(hObject, handles);

if isequal(get(gcf, 'waitstatus'), 'waiting')
    % The GUI is still in UIWAIT, us UIRESUME
    uiresume(gcf);
else
    % The GUI is no longer waiting, just close it
    close(gcf);
end



function edit_kp_Callback(hObject, eventdata, handles)


function edit_kp_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_ko_Callback(hObject, eventdata, handles)


function edit_ko_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function pushbutton_cancel_Callback(hObject, eventdata, handles)
if isequal(get(gcf, 'waitstatus'), 'waiting')
    % The GUI is still in UIWAIT, us UIRESUME
    uiresume(gcf);
else
    % The GUI is no longer waiting, just close it
    close(gcf);
end
