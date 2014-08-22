function varargout = TEOTraGen(varargin)
% TEOTraGen MATLAB code for TEOTraGen.fig
%      TEOTraGen, by itself, creates a new TEOTraGen or raises the existing
%      singleton*.
%
%      H = TEOTraGen returns the handle to a new TEOTraGen or the handle to
%      the existing singleton*.
%
%      TEOTraGen('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in TEOTraGen.M with the given input arguments.
%
%      TEOTraGen('Property','Value',...) creates a new TEOTraGen or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before TEOTraGen_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to TEOTraGen_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help TEOTraGen

% Last Modified by GUIDE v2.5 31-Jul-2014 18:50:25

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @TEOTraGen_OpeningFcn, ...
                   'gui_OutputFcn',  @TEOTraGen_OutputFcn, ...
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


% --- Executes just before TEOTraGen is made visible.
function TEOTraGen_OpeningFcn(hObject, eventdata, handles, varargin)

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
[r,map]=imread('Teo-photo1.jpg','jpg');
image(r);colormap(map);axis off
axes(handles.axes2)
[r,map]=imread('uc3m.png','png');
image(r);colormap(map);axis off
% Update handles structure
guidata(hObject, handles);

% --- Outputs from this function are returned to the command line.
function varargout = TEOTraGen_OutputFcn(hObject, eventdata, handles) 

varargout{1} = handles.output;


% --- Executes on button press in button_teo_step_gen.
function button_teo_step_gen_Callback(hObject, eventdata, handles)

TEOStepGen

close TEOTraGen


function button_teo_whole_gen_Callback(hObject, eventdata, handles)

TEOWholeGen

close TEOTraGen
