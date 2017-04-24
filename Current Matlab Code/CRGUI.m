function varargout = CRGUI(varargin)
% CRGUI MATLAB code for CRGUI.fig
%      CRGUI, by itself, creates a new CRGUI or raises the existing
%      singleton*.
%
%      H = CRGUI returns the handle to a new CRGUI or the handle to
%      the existing singleton*.
%
%      CRGUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in CRGUI.M with the given input arguments.
%
%      CRGUI('Property','Value',...) creates a new CRGUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before CRGUI_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to CRGUI_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help CRGUI

% Last Modified by GUIDE v2.5 21-Apr-2017 14:48:26

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @CRGUI_OpeningFcn, ...
                   'gui_OutputFcn',  @CRGUI_OutputFcn, ...
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


% --- Executes just before CRGUI is made visible.
function CRGUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to CRGUI (see VARARGIN)

% Choose default command line output for CRGUI
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes CRGUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);



% --- Outputs from this function are returned to the command line.
function varargout = CRGUI_OutputFcn(hObject, eventdata, handles) 
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
    CRMain;
end
