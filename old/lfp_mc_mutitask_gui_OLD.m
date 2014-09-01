function varargout = lfp_mc_mutitask_gui(varargin)
% LFP_MC_MUTITASK_GUI M-file for lfp_mc_mutitask_gui.fig
%      LFP_MC_MUTITASK_GUI, by itself, creates a new LFP_MC_MUTITASK_GUI or raises the existing
%      singleton*.
%
%      H = LFP_MC_MUTITASK_GUI returns the handle to a new LFP_MC_MUTITASK_GUI or the handle to
%      the existing singleton*.
%
%      LFP_MC_MUTITASK_GUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in LFP_MC_MUTITASK_GUI.M with the given input arguments.
%
%      LFP_MC_MUTITASK_GUI('Property','Value',...) creates a new LFP_MC_MUTITASK_GUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before lfp_mc_mutitask_gui_OpeningFunction gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to lfp_mc_mutitask_gui_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help lfp_mc_mutitask_gui

% Last Modified by GUIDE v2.5 16-Apr-2014 09:26:51

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @lfp_mc_mutitask_gui_OpeningFcn, ...
                   'gui_OutputFcn',  @lfp_mc_mutitask_gui_OutputFcn, ...
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


% --- Executes just before lfp_mc_mutitask_gui is made visible.
function lfp_mc_mutitask_gui_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to lfp_mc_mutitask_gui (see VARARGIN)

% Choose default command line output for lfp_mc_mutitask_gui
handles.output = hObject;

addpath('C:\Documents and Settings\Jose M Carmena\My Documents\Amy\Matlab\chronux')
addpath([pwd '\classes'])
addpath([pwd '\utils'])
addpath([pwd '\utils\logdet'])

subject = 'seba';

handles.go_flag = 0;

% 8 cm = 1 normalized unit
handles.target_radius = 1.2/8;
handles.cursor_radius = 0.4/8;

% initialize necessary connections
handles.task_connect = dexterit_interface('192.168.0.2');
handles.neural_connect = plexon_interface();

%Updated:
handles.disp_xlim = [-1.5,1.5];
handles.disp_ylim = [-1.5,1.5];

hold(handles.main_disp,'on');
handles.h_mc_target = scatter(handles.main_disp,0,0,500,'g','filled');
handles.h_mc_cursor = scatter(handles.main_disp,0,0,200,'r','filled');
handles.h_lfp_target = scatter(handles.main_disp,0,0,500,'c','square','filled');
handles.h_lfp_cursor = scatter(handles.main_disp,0,0,200,'m','square','filled');

axis(handles.main_disp,1.2*[-1 1 -1 1]);
set(handles.main_disp,'color','k');

% Obtain the axes size (in axpos) in Points
currentunits = get(handles.main_disp,'Units');
set(handles.main_disp, 'Units', 'Points');
axpos = get(handles.main_disp,'Position');
set(handles.main_disp, 'Units', currentunits);
scale_factor = axpos(3)/diff(xlim(handles.main_disp));

set(handles.h_mc_target,'SizeData',(2*handles.target_radius*scale_factor)^2);
set(handles.h_mc_cursor,'SizeData',(2*handles.cursor_radius*scale_factor)^2);
set(handles.h_lfp_target,'SizeData',(2*handles.target_radius*scale_factor)^2);
set(handles.h_lfp_cursor,'SizeData',(2*handles.cursor_radius*scale_factor)^2);


hold(handles.task_perf_disp,'on');
handles.h_reward        = plot(handles.task_perf_disp,0,0,'k');
handles.h_init          = plot(handles.task_perf_disp,0,0,'b');
% handles.h_ctr_holderr   = plot(handles.task_perf_disp,0,0,'r');
% handles.h_tar_holderr   = plot(handles.task_perf_disp,0,0,'g');
% handles.h_timeout       = plot(handles.task_perf_disp,0,0,'m');
legend_strs = {'reward','trials initiated','center hold error',...
               'target hold error','reach timeout'};
legend(handles.task_perf_disp,legend_strs,'location','eastoutside');
ylim(handles.task_perf_disp,[0 20]);
set(handles.task_perf_disp,'YTick',0:4:20);
set(handles.task_perf_disp,'XGrid','on','YGrid','on');

% load default values into text boxes and dropdown menus
set(handles.total_time_box,'String',600);
set(handles.dt_box, 'String',100);
handles.t = 0;

% auto name files
[data_filename decoder_filename neural_filename] = GetAutoName(1,subject);
handles.data_fullname    = [pwd '\' subject '\data\' data_filename];
handles.decoder_fullname = [pwd '\' subject '\decoders\' decoder_filename];
handles.neural_fullname = [pwd '\' subject '\neural\' neural_filename];

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes lfp_mc_mutitask_gui wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = lfp_mc_mutitask_gui_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

function total_time_box_Callback(hObject, eventdata, handles)
% hObject    handle to total_time_box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of total_time_box as text
%        str2double(get(hObject,'String')) returns contents of total_time_box as a double


% --- Executes during object creation, after setting all properties.
function total_time_box_CreateFcn(hObject, eventdata, handles)
% hObject    handle to total_time_box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in go.
function go_Callback(hObject, eventdata, handles)
% hObject    handle to go (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


dt = str2double(get(handles.dt_box,'String'))/1000;
fs = handles.fs;

% load('co_targetpos.dat');  % loads a variable called target positions
% target_positions = reshape(co_targetpos,2,8)';
% target_positions = [0 0; target_positions];  % loaded variable doesn't include center

%variable named 'target_positions' that is a 1x2 cell array with
%target_positions{1}= 9x2 matrix of targets for MC and target_positions{2}
%= 9x2 matrix of targets for LFP control
load('lfp_mc_targets.mat') 

pow_extractor = handles.pow_extractor; %from loaded file
n_mc_kin_vars = 4; %[xpos ypos xvel yvel];
n_lfp_kin_vars = 2; %[ypos yvel];
n_features = pow_extractor.n_features;

total_time = str2double(get(handles.total_time_box,'String'));
total_itrs = round(total_time/dt) + 1;

data = struct;
data.mc_target_pos          = zeros(2,total_itrs); 
data.lfp_target_pos         = zeros(n_lfp_kin_vars,total_itrs);
data.features               = zeros(n_features,total_itrs);
data.mc_cursor_kin          = zeros(n_mc_kin_vars,total_itrs); %
data.lfp_cursor_kin         = zeros(n_lfp_kin_vars,total_itrs); %

data.itr_times              = zeros(total_itrs,1);
data.dropped_packets        = zeros(total_itrs,1);
data.sampled_times          = zeros(total_itrs,1);
data.n_samp_recv            = zeros(total_itrs,1);

data.events                 = cell(total_itrs,1);    
data.reward_trials          = zeros(total_itrs,1);
data.init_trials            = zeros(total_itrs,1);
data.center_hold_errors     = zeros(total_itrs,1);
data.target_hold_errors     = zeros(total_itrs,1);
data.reach_timeouts         = zeros(total_itrs,1);

% measured in trials/min
data.trial_reward_rate      = zeros(total_itrs,1);
data.trial_init_rate        = zeros(total_itrs,1);
data.center_hold_error_rate = zeros(total_itrs,1);
data.target_hold_error_rate = zeros(total_itrs,1);
data.reach_timeout_rate     = zeros(total_itrs,1);

perf_hist_min = 2;

CODES = struct;
CODES.TRIAL_INIT        = 5;
CODES.REWARD            = 9;
CODES.CENTER_HOLD_ERROR = 15;
CODES.TARGET_HOLD_ERROR = 19;
CODES.REACH_TIMEOUT     = 18;

neural_buffer_t = 1;  % in secs
neural_buffer_size = round(neural_buffer_t*fs);
neural_buffer = ringbuffer(length(handles.all_chan),neural_buffer_size);

% features_buffer_t = 1;  % in secs
% features_buffer_size = round(features_buffer_t/dt);
% features_buffer = ringbuffer(f_extractor.n_features,features_buffer_size);

% assume initial target is the center
current_target{1} = 1;
current_target{2} = 1;
data.mc_target_pos(1:2,1) = target_positions{1}(current_target{1},:)';

hold_flag={};
hold_flag{1} = 0;
hold_flag{2} = 0;

% set lfp initial cursor position to center
data.lfp_cursor_kin(1:2,1) = target_positions{2}(1,:)';

% set mc initial position to current position
handles.neural_connect.getAD();
tmp=handles.neural_connect.getKinematics();
data.mc_cursor_kin(1:2,1) = tmp(1:2);
clear tmp

% flip go switch
handles.go_flag = 1;
guidata(hObject,handles);

msg = sprintf('Time start: %s',datestr(now,'HH:MM:SS'));
update_status(handles,msg,0);

handles.neural_connect.clearBuffer();
handles.task_connect.sendGo();

for t = 2:total_itrs
    t0 = GetSecs();
    set(handles.dispbox,'String',sprintf('Time Elapsed: %3.3f s',t*handles.dt));
    handles.t = t;
    guidata(hObject,handles);
    
    [data.n_samp_recv(t) data.sampled_times(t)] = handles.neural_connect.getAD();
    new_neural = handles.neural_connect.getNeural();
    new_kin = handles.neural_connect.getKinematics();
    
    handles.neural_connect.getTS();
    data.events{t} = handles.neural_connect.getEvents();

    data.reward_trials(t)      = ismember(CODES.REWARD,data.events{t});
    data.init_trials(t)        = ismember(CODES.TRIAL_INIT,data.events{t});
    data.center_hold_errors(t) = ismember(CODES.CENTER_HOLD_ERROR,data.events{t});
    data.target_hold_errors(t) = ismember(CODES.TARGET_HOLD_ERROR,data.events{t});
    data.reach_timeouts(t)     = ismember(CODES.REACH_TIMEOUT,data.events{t});

    if data.reward_trials(t)
        msg = sprintf('Reward trial # %d',sum(data.reward_trials));
        update_status(handles,msg,t);
    end
    
    if data.center_hold_errors(t)
        msg = sprintf('Center Hold Error');
        update_status(handles,msg,t);
    end
    
    if data.target_hold_errors(t)
        msg = sprintf('Target Hold Error');
        update_status(handles,msg,t);
    end

    inds = max(1,t-round(perf_hist_min*60/dt)):t;
    data.trial_reward_rate(t)      = sum(data.reward_trials(inds))/perf_hist_min;
    data.trial_init_rate(t)        = sum(data.init_trials(inds))/perf_hist_min;
    data.center_hold_error_rate(t) = sum(data.center_hold_errors(inds))/perf_hist_min;
    data.target_hold_error_rate(t) = sum(data.target_hold_errors(inds))/perf_hist_min;
    data.reach_timeout_rate(t)     = sum(data.reach_timeouts(inds))/perf_hist_min;

    %MC target first: 
    try
        [new_target_pos,current_target,hold_flag] = ...
            compute_current_lfp_mc_targets(data.events{t},target_positions,...
            current_target,hold_flag);
    catch
        new_target_pos{1} = data.mc_target_pos(:,t-1);
        new_target_pos{2} = data.lfp_target_pos(:,t-1);
    end
    
    %MC update: 
    if ~isempty(new_target_pos{1})
        data.mc_target_pos(1:2,t) = new_target_pos{1}';
    else
        data.mc_target_pos(1:2,t) = data.mc_target_pos(:,t-1);
    end
    
    %LFP update:
    if ~isempty(new_target_pos{2})
        data.lfp_target_pos(:,t) = new_target_pos{2}';
    else
        data.lfp_target_pos(:,t) = data.lfp_target_pos(:,t-1);
    end

    %PUT THIS BACK:
    if data.n_samp_recv(t) > 0

        neural_buffer.insert(new_neural);
        data.features(:,t) = pow_extractor.extract_features(neural_buffer);                                    
%         features_buffer.insert(data.features(:,t));

        data.lfp_cursor_kin(:,t) = calc_lfp_cursor(pow_extractor,data.features(:,t),handles);
        data.mc_cursor_kin(:,t) = new_kin; %index into new_kin?

        % keep cursor in workspace
        [data.mc_cursor_kin(1,t),clipped_x] = clip(data.mc_cursor_kin(1,t),handles.disp_xlim);
        [data.mc_cursor_kin(2,t),clipped_y] = clip(data.mc_cursor_kin(2,t),handles.disp_ylim);

        [data.lfp_cursor_kin(1,t),clipped_y] = clip(data.lfp_cursor_kin(1,t),handles.disp_ylim);

        handles = guidata(hObject);
        handles.task_connect.sendPosVelXY([0 data.lfp_cursor_kin(1,t) 0 data.lfp_cursor_kin(2,t)]' .* [1 1 dt dt]');  
        
        handles = guidata(hObject);
 
    else  % not enough data collected
        data.mc_cursor_kin(:,t) = data.mc_cursor_kin(:,t-1);
        data.lfp_cursor_kin(:,t) = data.lfp_cursor_kin(:,t-1);
    end

    update_displays(handles,data,t);

    drawnow;
    
    handles = guidata(hObject);
    
    if handles.go_flag == 0
        break;
    end

    time_elapsed = GetSecs() - t0;
    if time_elapsed <= dt
%         fprintf('Iteration time = %0.3f\n',time_elapsed);
        WaitSecs(dt - time_elapsed);
    else
        fprintf('WARNING: iteration time = %0.3f\n',time_elapsed);
    end
    data.itr_times(t) = GetSecs() - t0;
end

handles.go_flag = 0;
guidata(hObject,handles);

msg = sprintf('Time end: %s',datestr(now,'HH:MM:SS'));
update_status(handles,msg,t);

% 
% if save_clda_vars
%     data.update_cnt       = update_cnt;
%     data.update_itrs      = update_itrs(1:update_cnt);
%     data.X_hist           = X_hist(1:update_cnt);
%     data.Y_hist           = Y_hist(1:update_cnt);
%     data.dec_params_hist  = dec_params_hist(1:update_cnt);
%     data.clda_params_hist = clda_params_hist(1:update_cnt);
% end

data.total_itrs = t;
data.messages = get(handles.status,'String');

handles.decoder = decoder;
handles.data = data;

save(handles.data_fullname,'data');
if save_clda_vars
    save(handles.decoder_fullname,'decoder','pow_extractor','clda_alg');
else
    save(handles.decoder_fullname,'decoder','pow_extractor');
end

fprintf('\nReward trials: %d\n',sum(data.reward_trials));

% fclose(handles.neural_file);
guidata(hObject,handles);


% --- Executes on button press in stop.
function stop_Callback(hObject, eventdata, handles)
% hObject    handle to stop (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

handles.go_flag = 0;
set(handles.dispbox,'String','Stopped.');
guidata(hObject,handles);

function status_Callback(hObject, eventdata, handles)
% hObject    handle to status (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of status as text
%        str2double(get(hObject,'String')) returns contents of status as a double


% --- Executes during object creation, after setting all properties.
function status_CreateFcn(hObject, eventdata, handles)
% hObject    handle to status (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in load_decoder.
function load_decoder_Callback(hObject, eventdata, handles)
% hObject    handle to load_decoder (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

[filename, pathname] = uigetfile({'*.mat'});
load([pathname filename]);

%Decoders are structs of the following form: 
%   - baseline_calc struct
%       - mn, sd, frange for common freq_ranges
%   - frange = [low_power high_power]
%   - baseline_used struct
%       - mn, sd for frange above
%   - task_name
load('lfp_mc_targets.mat') 

%Load baseline used info
handles.baseline_mean = decoder.baseline_used.mean;
handles.baseline_sd = decoder.baseline_used.sd;
handles.baseline_zsc2targ = min(abs(target_positions{2}(2:end,2)));
set(handles.baseline_used_box,'Value',1);
set(handles.low_power_box,'String',num2str(decoder.frange(1)))
set(handles.high_power_box,'String',num2str(decoder.frange(2)))
set(handles.task_name_box,'String',decoder.task_name);
handles.baseline_calc_from_day = decoder.baseline_calc;

handles.pow_extractor = pow_extractor;
update_status(handles,'baselines and feature extractor loaded',0);

handles.dt = str2double(get(handles.dt_box, 'String'))/1000; %Ms to sec
handles.fs = pow_extractor.fs;

% load channels used
handles.all_chan = 1:128;
handles.chan_offset = 64;
handles.used_chan = pow_extractor.used_chan;

handles.unused_chan = setxor(handles.all_chan,handles.used_chan);
handles.unused_chan = handles.unused_chan(:);

set(handles.used_chan_listbox,'Value',1);
set(handles.used_chan_listbox,'String',handles.used_chan + handles.chan_offset);
set(handles.unused_chan_listbox,'Value',1);
set(handles.unused_chan_listbox,'String',handles.unused_chan + handles.chan_offset);
guidata(hObject,handles);

function update_displays(handles,data,t)
    set(handles.h_mc_target,'xdata',data.mc_target_pos(1,t),...
                         'ydata',data.mc_target_pos(2,t));
    set(handles.h_mc_cursor,'xdata',data.mc_cursor_kin(1,t),...
                         'ydata',data.mc_cursor_kin(2,t));
    set(handles.h_lfp_target,'xdata',-.8,...
                         'ydata',data.lfp_target_pos(1,t));
    set(handles.h_lfp_cursor,'xdata',-.8,...
                         'ydata',data.lfp_cursor_kin(1,t));

    set(handles.h_reward,'xdata',(1:t)*handles.dt,...
                         'ydata',data.trial_reward_rate(1:t));
    set(handles.h_init,'xdata',(1:t)*handles.dt,...
                       'ydata',data.trial_init_rate(1:t));


function mean_baseline_used_box_box_Callback(hObject, eventdata, handles)
% hObject    handle to mean_baseline_used_box_box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of mean_baseline_used_box_box as text
%        str2double(get(hObject,'String')) returns contents of mean_baseline_used_box_box as a double


% --- Executes during object creation, after setting all properties.
function mean_baseline_used_box_box_CreateFcn(hObject, eventdata, handles)
% hObject    handle to mean_baseline_used_box_box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function sd_baseline_used_box_box_Callback(hObject, eventdata, handles)
% hObject    handle to sd_baseline_used_box_box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of sd_baseline_used_box_box as text
%        str2double(get(hObject,'String')) returns contents of sd_baseline_used_box_box as a double


% --- Executes during object creation, after setting all properties.
function sd_baseline_used_box_box_CreateFcn(hObject, eventdata, handles)
% hObject    handle to sd_baseline_used_box_box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function dt_box_Callback(hObject, eventdata, handles)
% hObject    handle to dt_box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of dt_box as text
%        str2double(get(hObject,'String')) returns contents of dt_box as a double


% --- Executes during object creation, after setting all properties.
function dt_box_CreateFcn(hObject, eventdata, handles)
% hObject    handle to dt_box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function update_status(handles,msg,t)

messages = get(handles.status,'String');
msg = sprintf('%d    %s',t,msg);
messages = [messages; {msg}];
set(handles.status,'String',messages);
set(handles.status,'Value',length(messages));

if handles.go_flag
    drawnow;
end



function low_power_box_Callback(hObject, eventdata, handles)
% hObject    handle to low_power_box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of low_power_box as text
%        str2double(get(hObject,'String')) returns contents of low_power_box as a double


% --- Executes during object creation, after setting all properties.
function low_power_box_CreateFcn(hObject, eventdata, handles)
% hObject    handle to low_power_box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit10_Callback(hObject, eventdata, handles)
% hObject    handle to edit10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit10 as text
%        str2double(get(hObject,'String')) returns contents of edit10 as a double


% --- Executes during object creation, after setting all properties.
function edit10_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function task_name_box_Callback(hObject, eventdata, handles)
% hObject    handle to task_name_box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of task_name_box as text
%        str2double(get(hObject,'String')) returns contents of task_name_box as a double


% --- Executes during object creation, after setting all properties.
function task_name_box_CreateFcn(hObject, eventdata, handles)
% hObject    handle to task_name_box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end




% --- Executes on button press in baseline_used_box.
function baseline_used_box_Callback(hObject, eventdata, handles)
% hObject    handle to baseline_used_box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of baseline_used_box


% --- Executes on button press in baseline_from_box.
function baseline_from_box_Callback(hObject, eventdata, handles)
% hObject    handle to baseline_from_box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of baseline_from_box
if get(hObject,'Value')
    set(handles.baseline_used_box,'Value',0);
    low = str2double(get(handles.low_power_box,'String'))
    high = str2double(get(handles.high_power_box,'String'))
    frange = handles.baseline_calc_from_day.frange
    match = 0;
    for i = 1:size(frange,1)
        if (frange(i,1) == low) && (frange(i,2) == high)
            handles.baseline_mean = handles.baseline_calc_from_day.means(i);
            handles.baseline_sd = handles.baseline_calc_from_day.sds(i);
            match=1;
        end
    end
    
    if match<1
        set(hObject,'Value',0)
        set(handles.baseline_used_box,'Value',1);
        
        disp('Low / High Frequencies dont match saved baselines: Set Low / High power to one of the following:')
        for i = 1:size(frange,1)
            msg = sprintf('Low: %d, High: %d',frange(i,1),frange(i,2));
            disp(msg)
        end
    end
end
guidata(hObject, handles);

% 
% % --- Executes during object creation, after setting all properties.
% function used_chan_listbox_CreateFcn(hObject, eventdata, handles)
% % hObject    handle to used_chan_listbox (see GCBO)
% % eventdata  reserved - to be defined in a future version of MATLAB
% % handles    empty - handles not created until after all CreateFcns called
% 
% % Hint: listbox controls usually have a white background on Windows.
% %       See ISPC and COMPUTER.
% if ispc && isequal(get(hObject,'BackgroundColor'), ...
%         get(0,'defaultUicontrolBackgroundColor'))
%     set(hObject,'BackgroundColor','white');
% end
% set(hObject,'Max',128);
% 
% 
% % --- Executes during object creation, after setting all properties.
% function unused_chan_listbox_CreateFcn(hObject, eventdata, handles)
% % hObject    handle to unused_chan_listbox (see GCBO)
% % eventdata  reserved - to be defined in a future version of MATLAB
% % handles    empty - handles not created until after all CreateFcns called
% 
% % Hint: listbox controls usually have a white background on Windows.
% %       See ISPC and COMPUTER.
% if ispc && isequal(get(hObject,'BackgroundColor'), ...
%         get(0,'defaultUicontrolBackgroundColor'))
%     set(hObject,'BackgroundColor','white');...
% 
% end
% set(hObject,'Max',128);
% 
% 
% % --- Executes on button press in remove_chan.
% function remove_chan_Callback(hObject, eventdata, handles)
% % hObject    handle to remove_chan (see GCBO)
% % eventdata  reserved - to be defined in a future version of MATLAB
% % handles    structure with handles and user data (see GUIDATA)
% 
% % indices of channels in "Used channels" list to move to "Unused channels"
% inds = get(handles.used_chan_listbox,'Value');
% if ~isempty(handles.used_chan)
%     handles.unused_chan = [handles.unused_chan; handles.used_chan(inds)];
%     handles.unused_chan = sort(handles.unused_chan);
% 
%     % remove corresponding channels from each feature extractor
%     bad_chan = handles.used_chan(inds);
% 
%     % remove corresponding columns from decoder
%     bad_inds = zeros(handles.f_extractor.n_features,1);
%     for k = bad_chan(:)'
%         bad_inds = bad_inds + (handles.f_extractor.chnfeat_index == k);
%     end
%     bad_rows = find(bad_inds == 1);
%     handles.decoder.remove_feature(bad_rows);
% 
%     % remove from extractor
%     handles.f_extractor.remove_channels(bad_chan);
% 
%     % update tuning plot
%     for k = bad_rows(:)'
%         delete(handles.tuning_plot.Clines(k))
%     end
%     handles.tuning_plot.Clines(bad_rows) = [];
%     update_tuning(handles.decoder, handles.tuning_plot);
% 
%     handles.used_chan(inds) = [];
%     set(handles.used_chan_listbox,'Value',1);
%     set(handles.used_chan_listbox,'String',handles.used_chan + handles.chan_offset);
%     set(handles.unused_chan_listbox,'String',handles.unused_chan + handles.chan_offset);
% end
% set(handles.used_total,'String',sprintf('Used: %d',length(handles.used_chan)));
% set(handles.unused_total,'String',sprintf('Unused: %d',length(handles.unused_chan)));
% guidata(hObject,handles);
% 
% % --- Executes on button press in add_chan.
% function add_chan_Callback(hObject, eventdata, handles)
% % hObject    handle to add_chan (see GCBO)
% % eventdata  reserved - to be defined in a future version of MATLAB
% % handles    structure with handles and user data (see GUIDATA)
% 
% % indices of channels in "Unused channels" list to move to "Used channels"
% inds = get(handles.unused_chan_listbox,'Value');
% if ~isempty(handles.unused_chan)
%     handles.used_chan = [handles.used_chan; handles.unused_chan(inds)];
%     handles.used_chan = sort(handles.used_chan);
%     handles.unused_chan(inds) = [];
%     set(handles.unused_chan_listbox,'Value',1);
%     set(handles.used_chan_listbox,'String',handles.used_chan + handles.chan_offset);
%     set(handles.unused_chan_listbox,'String',handles.unused_chan + handles.chan_offset);
% end
% guidata(hObject,handles);


