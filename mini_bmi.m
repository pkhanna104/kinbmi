function varargout = mini_bmi(varargin)
%MINI_BMI M-file for mini_bmi.fig
%      MINI_BMI, by itself, creates a new MINI_BMI or raises the existing
%      singleton*.
%
%      H = MINI_BMI returns the handle to a new MINI_BMI or the handle to
%      the existing singleton*.
%
%      MINI_BMI('Property','Value',...) creates a new MINI_BMI using the
%      given property value pairs. Unrecognized properties are passed via
%      varargin to mini_bmi_OpeningFcn.  This calling syntax produces a
%      warning when there is an existing singleton*.
%
%      MINI_BMI('CALLBACK') and MINI_BMI('CALLBACK',hObject,...) call the
%      local function named CALLBACK in MINI_BMI.M with the given input
%      arguments.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help mini_bmi

% Last Modified by GUIDE v2.5 25-Mar-2015 20:20:24

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @mini_bmi_OpeningFcn, ...
                   'gui_OutputFcn',  @mini_bmi_OutputFcn, ...
                   'gui_LayoutFcn',  [], ...
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


% --- Executes just before mini_bmi is made visible.
function mini_bmi_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   unrecognized PropertyName/PropertyValue pairs from the
%            command line (see VARARGIN)

% Choose default command line output for mini_bmi
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes mini_bmi wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = mini_bmi_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in go_button.
function go_button_Callback(hObject, eventdata, handles)
% hObject    handle to go_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



function targetSizeBox_Callback(hObject, eventdata, handles)
% hObject    handle to targetSizeBox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of targetSizeBox as text
%        str2double(get(hObject,'String')) returns contents of targetSizeBox as a double


% --- Executes during object creation, after setting all properties.
function targetSizeBox_CreateFcn(hObject, eventdata, handles)
% hObject    handle to targetSizeBox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function cursorSizeBox_Callback(hObject, eventdata, handles)
% hObject    handle to cursorSizeBox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of cursorSizeBox as text
%        str2double(get(hObject,'String')) returns contents of cursorSizeBox as a double


% --- Executes during object creation, after setting all properties.
function cursorSizeBox_CreateFcn(hObject, eventdata, handles)
% hObject    handle to cursorSizeBox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
