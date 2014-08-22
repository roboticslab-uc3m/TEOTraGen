%% TEOWholeGen
% *TEOWholeGen* (TEO Step Generator)

function varargout = TEOWholeGen(varargin)
% TEOWholeGen MATLAB code for TEOWholeGen.fig
%      TEOWholeGen, by itself, creates a new TEOWholeGen or raises the existing
%      singleton*.
%
%      H = TEOWholeGen returns the handle to a new TEOWholeGen or the handle to
%      the existing singleton*.
%
%      TEOWholeGen('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in TEOWholeGen.M with the given input arguments.
%
%      TEOWholeGen('Property','Value',...) creates a new TEOWholeGen or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before TEOWholeGen_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to TEOWholeGen_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help TEOWholeGen

% Last Modified by GUIDE v2.5 31-Jul-2014 16:05:01

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @TEOWholeGen_OpeningFcn, ...
                   'gui_OutputFcn',  @TEOWholeGen_OutputFcn, ...
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


% --- Executes just before TEOWholeGen is made visible.
function TEOWholeGen_OpeningFcn(hObject, eventdata, handles, varargin)

handles.output = hObject;
%--------------------------------------------------------------------------
% To put screen correctly

scrsz = get(0,'ScreenSize');
pos_act = get(gcf,'Position');
xr = scrsz(3) - pos_act(3);
xp = round(xr/2);
yr = scrsz(4) - pos_act(4);
yp = round(yr/2);
set(gcf,'Position',[xp yp pos_act(3) pos_act(4)]);

% to chargue main image 'portada.jpg'
axes(handles.axes1)
[r,map] = imread('TEO-whole-background.png','png');
image(r);colormap(map);axis off

% Update handles structure
guidata(hObject, handles);

% --- Outputs from this function are returned to the command line.
function varargout = TEOWholeGen_OutputFcn(hObject, eventdata, handles) 

varargout{1} = handles.output;

% --- Executes on button press in button_joints_space.
function button_joints_space_Callback(hObject, eventdata, handles)
joints_space_interpolation
close TEOWholeGen


function button_operational_space_Callback(hObject, eventdata, handles)
settings_trajectory_generation
close TEOWholeGen
