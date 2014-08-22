function varargout = change_floor_options(varargin)
% CHANGE_FLOOR_OPTIONS MATLAB code for change_floor_options.fig
%      CHANGE_FLOOR_OPTIONS, by itself, creates a new CHANGE_FLOOR_OPTIONS or raises the existing
%      singleton*.
%
%      H = CHANGE_FLOOR_OPTIONS returns the handle to a new CHANGE_FLOOR_OPTIONS or the handle to
%      the existing singleton*.
%
%      CHANGE_FLOOR_OPTIONS('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in CHANGE_FLOOR_OPTIONS.M with the given input arguments.
%
%      CHANGE_FLOOR_OPTIONS('Property','Value',...) creates a new CHANGE_FLOOR_OPTIONS or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before change_floor_options_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to change_floor_options_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help change_floor_options

% Last Modified by GUIDE v2.5 08-Jun-2014 10:17:46

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @change_floor_options_OpeningFcn, ...
                   'gui_OutputFcn',  @change_floor_options_OutputFcn, ...
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


function change_floor_options_OpeningFcn(hObject, eventdata, handles, varargin)
% Choose default command line output for change_floor_options
handles.output = hObject;

if isempty(varargin)
  warning('WarnTEOTraGen:noInputGUI', 'No inputs for change_floor_options');
  handles.floor_options = 0;
%   handles.axes_flat_floor = 0;
else
  % Get previous values
  handles.floor_options = varargin{1};
  handles.axes_flat_floor = varargin{2};
end


if isstruct(handles.floor_options) 
  set(handles.edit_xmin,'String', num2str(handles.floor_options.xmin));
  set(handles.edit_xmax,'String', num2str(handles.floor_options.xmax));
  set(handles.edit_ymin,'String', num2str(handles.floor_options.ymin));
  set(handles.edit_ymax,'String', num2str(handles.floor_options.ymax));
  set(handles.edit_xscale,'String', num2str(handles.floor_options.xscale));
  set(handles.edit_yscale,'String', num2str(handles.floor_options.yscale));
else
  set(handles.edit_xmin,'String', num2str(0));
  set(handles.edit_xmax,'String', num2str(0));
  set(handles.edit_ymin,'String', num2str(0));
  set(handles.edit_ymax,'String', num2str(0));
  set(handles.edit_xscale,'String', num2str(0));
  set(handles.edit_yscale,'String', num2str(0));
end

% Update handles structure
guidata(hObject, handles);

% UIWAIT
uiwait(handles.figure1);



function varargout = change_floor_options_OutputFcn(hObject, eventdata, handles) 
varargout{1} = handles.floor_options;

close(gcf)



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



function pushbutton_cancel_Callback(hObject, eventdata, handles)

set(handles.edit_xmin,'String', num2str(handles.floor_options.xmin));
set(handles.edit_xmax,'String', num2str(handles.floor_options.xmax));
set(handles.edit_ymin,'String', num2str(handles.floor_options.ymin));
set(handles.edit_ymax,'String', num2str(handles.floor_options.ymax));
set(handles.edit_xscale,'String', num2str(handles.floor_options.xscale));
set(handles.edit_yscale,'String', num2str(handles.floor_options.yscale));

% Update handles structure
guidata(hObject, handles);

update_floor_options(handles)

if isequal(get(gcf, 'waitstatus'), 'waiting')
    % The GUI is still in UIWAIT, us UIRESUME
    uiresume(gcf);
else
    % The GUI is no longer waiting, just close it
    close(gcf);
end


function pushbutton_apply_Callback(hObject, eventdata, handles)

handles.floor_options.xmin = str2num(get(handles.edit_xmin,'String'));
handles.floor_options.xmax = str2num(get(handles.edit_xmax,'String'));
handles.floor_options.ymin = str2num(get(handles.edit_ymin,'String'));
handles.floor_options.ymax = str2num(get(handles.edit_ymax,'String'));
handles.floor_options.xscale = str2num(get(handles.edit_xscale,'String'));
handles.floor_options.yscale = str2num(get(handles.edit_yscale,'String'));

% Update handles structure
guidata(hObject, handles);

if isequal(get(gcf, 'waitstatus'), 'waiting')
    % The GUI is still in UIWAIT, us UIRESUME
    uiresume(gcf);
else
    % The GUI is no longer waiting, just close it
    close(gcf);
end


function edit_xmin_Callback(hObject, eventdata, handles)
update_floor_options(handles)


function edit_xmin_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit_xmax_Callback(hObject, eventdata, handles)
update_floor_options(handles)


function edit_xmax_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit_ymin_Callback(hObject, eventdata, handles)
update_floor_options(handles)


function edit_ymin_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit_ymax_Callback(hObject, eventdata, handles)
update_floor_options(handles)


function edit_ymax_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit_xscale_Callback(hObject, eventdata, handles)
update_floor_options(handles)


function edit_xscale_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit_yscale_Callback(hObject, eventdata, handles)
update_floor_options(handles)


function edit_yscale_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
