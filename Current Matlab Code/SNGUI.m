function varargout = SNGUI(varargin)
% SNGUI MATLAB code for SNGUI.fig
%      SNGUI, by itself, creates a new SNGUI or raises the existing
%      singleton*.
%
%      H = SNGUI returns the handle to a new SNGUI or the handle to
%      the existing singleton*.
%
%      SNGUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in SNGUI.M with the given input arguments.
%
%      SNGUI('Property','Value',...) creates a new SNGUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before SNGUI_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to SNGUI_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help SNGUI

% Last Modified by GUIDE v2.5 09-Mar-2017 11:56:37

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @SNGUI_OpeningFcn, ...
                   'gui_OutputFcn',  @SNGUI_OutputFcn, ...
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


% --- Executes just before SNGUI is made visible.
function SNGUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to SNGUI (see VARARGIN)

% Choose default command line output for SNGUI
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes SNGUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);



% --- Outputs from this function are returned to the command line.
function varargout = SNGUI_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

% --- Executes on button press in togglebutton2.
function togglebutton2_Callback(hObject, eventdata, handles)
% hObject    handle to togglebutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of togglebutton2
if get(hObject,'Value')    
    cla reset;
    % use v1 for smaller robots
    %SensorNodev1;
    % use v2 for custom robots
    SensorNodev2;
end
