function varargout = simple_lfp_mc_mutitask_ubuntu(varargin)
% LFP_MC_MUTITASK_2 M-file for lfp_mc_mutitask_2.fig
%      LFP_MC_MUTITASK_2, by itself, creates a new LFP_MC_MUTITASK_2 or raises the existing
%      singleton*.
%
%      H = LFP_MC_MUTITASK_2 returns the handle to a new LFP_MC_MUTITASK_2 or the handle to
%      the existing singleton*.
%
%      LFP_MC_MUTITASK_2('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in LFP_MC_MUTITASK_2.M with the given input arguments.
%
%      LFP_MC_MUTITASK_2('Property','Value',...) creates a new LFP_MC_MUTITASK_2 or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before lfp_mc_mutitask_2_OpeningFunction gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to lfp_mc_mutitask_2_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help lfp_mc_mutitask_2

% Last Modified by GUIDE v2.5 16-Mar-2015 16:43:53

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @lfp_mc_mutitask_2_OpeningFcn, ...
                   'gui_OutputFcn',  @lfp_mc_mutitask_2_OutputFcn, ...
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


% --- Executes just before lfp_mc_mutitask_2 is made visible.
function lfp_mc_mutitask_2_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to lfp_mc_mutitask_2 (see VARARGIN)

% Choose default command line output for lfp_mc_mutitask_2
handles.output = hObject;

addpath(genpath('/home/lab/bmi_folders/Preeya/classes'))
addpath(genpath('/home/lab/bmi_folders/Preeya/utils'))

subject = 'seba';

handles.go_flag = 0;

% 8 cm = 1 normalized unit
handles.target_radius = 1.2/8;
handles.cursor_radius = 0.4/8;

% initialize necessary connections:
% connect to server: 

%handles.task_connect = dexterit_interface('192.168.0.2');
handles.neural_connect = plexon_client();

%Updated:
handles.disp_xlim = [-1.5,1.5];
handles.disp_ylim = [-1.5,1.5];

%Main Display Init:
hold(handles.main_disp,'on');
handles.h_mc_target = scatter(handles.main_disp,0,0,500,'g','filled');
handles.h_mc_cursor = scatter(handles.main_disp,0,0,200,'r','filled');
handles.h_lfp_target = scatter(handles.main_disp,-1,0,500,'c','filled');
handles.h_lfp_cursor = scatter(handles.main_disp,-1,0,200,'m','filled');

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

%Setup plot power: 
hold(handles.plot_power,'on');
handles.h_plot_total = plot(handles.plot_power,0,0,'k.-');
handles.h_plot_band = plot(handles.plot_power,0,0,'b.-');
ylim(handles.plot_power, [0 50]);

handles.h_plot_fraction = plot(handles.plot_fraction,0,0,'g.-');
ylim(handles.plot_fraction,[0,1]);

handles.h_plot_data = zeros(3,30);

% load default values into text boxes and dropdown menus
set(handles.total_time_box,'String',600);
set(handles.dt_box, 'String',100);
handles.t = 0;
handles.low_pass_filt = 1;

% auto name files
[data_filename decoder_filename neural_filename] = GetAutoName(1,subject);
handles.data_fullname    = [pwd '/../' subject '/data/' data_filename];
handles.decoder_fullname = [pwd '/../' subject '/decoders/' decoder_filename];
handles.neural_fullname = [pwd '/../' subject '/neural/' neural_filename];

   
% Update handles structure
guidata(hObject, handles);

% UIWAIT makes lfp_mc_mutitask_2 wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = lfp_mc_mutitask_2_OutputFcn(hObject, eventdata, handles) 
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


%variable named 'target_positions' that is a 1x2 cell array with
%target_positions{1}= 9x2 matrix of targets for MC and target_positions{2}
%= 9x2 matrix of targets for LFP control. First 'target' is origin. 
load('lfp_mc_targets.mat') 

p_extractor = handles.p_extractor; %from loaded file
n_mc_kin_vars = 4; %[xpos ypos xvel yvel];
n_lfp_kin_vars = 2; %[ypos yvel];
n_features = p_extractor.n_features;

total_time = str2double(get(handles.total_time_box,'String'));
total_itrs = round(total_time/dt) + 1;

data = struct;
data.mc_target_pos          = zeros(2,total_itrs); 
data.lfp_target_pos         = zeros(n_lfp_kin_vars,total_itrs);
data.features               = zeros(n_features,total_itrs);
data.mc_cursor_kin          = zeros(n_mc_kin_vars,total_itrs); %
data.lfp_cursor_kin         = zeros(n_lfp_kin_vars,total_itrs); %
data.baseline_params        = zeros(2,total_itrs); %


data.itr_times              = zeros(total_itrs,1);
data.dropped_packets        = zeros(total_itrs,1);
data.sampled_times          = zeros(total_itrs,1);
data.n_samp_recv            = zeros(total_itrs,1);

data.events                 = cell(total_itrs,1); 
data.init_trials            = zeros(total_itrs,1);
data.lfp_acquired_trials    = zeros(total_itrs,1);
data.reward                 = zeros(total_itrs,1);
data.powerOK                = zeros(total_itrs,1);
data.mc_center_hold_errors  = zeros(total_itrs,1);
data.lfp_target_hold_errors = zeros(total_itrs,1);
data.mc_target_hold_errors  = zeros(total_itrs,1);

data.lfp_reach_timeouts     = zeros(total_itrs,1);
data.mc_reach_timeouts      = zeros(total_itrs,1);

% measured in trials/min
data.trial_reward_rate      = zeros(total_itrs,1);
data.trial_init_rate        = zeros(total_itrs,1);
data.mc_center_hold_error_rate = zeros(total_itrs,1);
data.lfp_target_hold_error_rate = zeros(total_itrs,1);
data.mc_target_hold_error_rate = zeros(total_itrs,1);
data.lfp_reach_timeout_rate     = zeros(total_itrs,1);
data.mc_reach_timeout_rate     = zeros(total_itrs,1);

perf_hist_min = 2;

CODES = struct;

%ERRORS:

%EVENTS
CODES.TRIAL_INIT        = 15;
CODES.LFP_ACQUIRED      = 5;
CODES.REWARD            = 9;

CODES.MC_CENTER_HOLD_ERROR = 4;
CODES.LFP_TARGET_HOLD_ERROR = 42;
CODES.MC_TARGET_HOLD_ERROR = 8;
CODES.LFP_REACH_TIMEOUT     = 41;
CODES.MC_REACH_TIMEOUT     = 12;

neural_buffer_t = 1;  % in secs
neural_buffer_size = round(neural_buffer_t*fs);
%neural_buffer = ringbuffer(length(handles.all_chan),neural_buffer_size);
neural_buffer = ringbuffer(length(handles.used_chan),neural_buffer_size);


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

%Start server: 
handles.neural_connect.add_chan(handles.used_chan,'replace_all')
handles.neural_connect.send_marco()
handles.neural_connect.get_polo(1)

% set mc initial position to current position
handles.neural_connect.get_data();
tmp=handles.neural_connect.get_kin();
data.mc_cursor_kin(1:2,1) = tmp(1:2);
clear tmp

% flip go switch
handles.go_flag = 1;
guidata(hObject,handles);

msg = sprintf('Time start: %s',datestr(now,'HH:MM:SS'));
update_status(handles,msg,0);

handles.neural_connect.clear_buffer();
handles.task_connect.sendGo();

if handles.sim_lfp
    q=load('dat052014g_bmi.mat'); 
    faux_lfp = q.data.lfp_cursor_kin(1,:);
    clear q
end

if handles.sim_lfp_darpa
    q=load('dat061014i_bmi.mat'); 
    faux_cursor = q.data.lfp_cursor_kin(1,:);
    faux_target_pos = q.data.lfp_target_pos(2,:);
    faux_target = calc_targ_sim(q.data.lfp_target_pos(2,:));
    faux_rew = q.data.reward_trials;
    clear q 
    faux_cursor = faux_cursor(2070:end);
    faux_target_pos = faux_target_pos(2070:end);
    faux_target = faux_target(2070:end);
    faux_rew = faux_rew(2070:end);
end

for t = 2:total_itrs
 
    if mod(t, 10);
        handles.neural_connect.keep_conn_alive()
    end
    t0 = GetSecs();
    set(handles.dispbox,'String',sprintf('Time Elapsed: %3.3f s',t*handles.dt));
    handles.t = t;
    guidata(hObject,handles);
    
    data.n_samp_recv(t)  = handles.neural_connect.get_data();
    new_neural = handles.neural_connect.get_neural();
    new_kin = handles.neural_connect.get_kin();
    
    data.events{t} = handles.neural_connect.get_events();
    
    
    data.init_trials(t)        = ismember(CODES.TRIAL_INIT,data.events{t});   
    data.lfp_acquired_trials(t)= ismember(CODES.LFP_ACQUIRED,data.events{t});
    data.reward_trials(t)      = ismember(CODES.REWARD,data.events{t});
    data.mc_center_hold_errors(t) = ismember(CODES.MC_CENTER_HOLD_ERROR,data.events{t});
    data.lfp_target_hold_errors(t) = ismember(CODES.LFP_TARGET_HOLD_ERROR,data.events{t});
    data.mc_target_hold_errors(t) = ismember(CODES.MC_TARGET_HOLD_ERROR,data.events{t});
    data.lfp_reach_timeouts(t)     = ismember(CODES.LFP_REACH_TIMEOUT,data.events{t});
    data.mc_reach_timeouts(t)     = ismember(CODES.MC_REACH_TIMEOUT,data.events{t});
        
    
    if handles.fraction
        data.baseline_params(:,t) = [handles.frac_range_low; handles.frac_range_high];
    else
        data.baseline_params(:,t)  = [handles.baseline_mean; handles.baseline_targ_step];
    end
    
    if data.reward_trials(t)
        msg = sprintf('Reward trial # %d',sum(data.reward_trials));
        update_status(handles,msg,t);
    end
    
    if data.mc_center_hold_errors(t)
        msg = sprintf('MC Center Hold Error');
        update_status(handles,msg,t);
    end
    
    if data.lfp_target_hold_errors(t)
        msg = sprintf('LFP Target Hold Error');
        update_status(handles,msg,t);
    end

    inds = max(1,t-round(perf_hist_min*60/dt)):t;
    data.trial_reward_rate(t)      = sum(data.reward_trials(inds))/perf_hist_min;
    data.trial_init_rate(t)        = sum(data.init_trials(inds))/perf_hist_min;
    data.mc_center_hold_error_rate(t) = sum(data.mc_center_hold_errors(inds))/perf_hist_min;
    data.mc_target_hold_error_rate(t) = sum(data.mc_target_hold_errors(inds))/perf_hist_min;
    data.lfp_target_hold_error_rate(t) = sum(data.lfp_target_hold_errors(inds))/perf_hist_min;
    data.lfp_reach_timeout_rate(t)     = sum(data.lfp_reach_timeouts(inds))/perf_hist_min;
    data.mc_reach_timeout_rate(t)     = sum(data.mc_reach_timeouts(inds))/perf_hist_min;

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
    
    if handles.sim_lfp_darpa
        data.lfp_target_pos(2,t) = faux_target_pos(t);
    end

    if data.n_samp_recv(t) > 0 || handles.sim_lfp || handles.sim_lfp_darpa

        neural_buffer.insert(new_neural);
        data.features(:,t) = p_extractor.extract_features(neural_buffer);
        
        if handles.sim_lfp_darpa
            [cur_data,h_plot_data, powerOK] = calc_lfp_cursor(p_extractor,data.features(:,t),handles,faux_cursor,data.lfp_target_pos(:,t));
        elseif handles.sim_lfp
            [cur_data,h_plot_data, powerOK] = calc_lfp_cursor(p_extractor,data.features(:,t),handles,faux_lfp,data.lfp_target_pos(:,t));
        else
            [cur_data,h_plot_data, powerOK] = calc_lfp_cursor(p_extractor,data.features(:,t),handles,0,data.lfp_target_pos(:,t));
        end
        
        if handles.low_pass_filt > 1
            filt_ind = max(1,t+1-handles.low_pass_filt); %Previous activity: 'low_pass_filt'-1 x 100ms
            filt_dat = mean( [data.lfp_cursor_kin(:,filt_ind:t-1) cur_data'] ,2);
            data.lfp_cursor_kin(:,t) = filt_dat;
        else
            data.lfp_cursor_kin(:,t)=cur_data;   
        end

        data.mc_cursor_kin(:,t) = new_kin; %index into new_kin?

        % keep cursor in workspace
        [data.mc_cursor_kin(1,t),clipped_x] = clip(data.mc_cursor_kin(1,t),handles.disp_xlim);
        [data.mc_cursor_kin(2,t),clipped_y] = clip(data.mc_cursor_kin(2,t),handles.disp_ylim);

        [data.lfp_cursor_kin(1,t),clipped_y] = clip(data.lfp_cursor_kin(1,t),handles.disp_ylim);

        handles = guidata(hObject);
        %disp(num2str(powerError))
        %powerError = 0;
        if t>3
            ttot = GetSecs()-tsend;
        end
        
        %%%%%HACK%%%%%%%%%%
        powerOK = 1;
        if handles.sim_lfp_darpah_
            handles.task_connect.sendPosVelXY([-1 data.lfp_cursor_kin(1,t) faux_target(t)+7 powerOK]'); %[X pos, Y pos, fakeTarget, powerError flag]  
        elseif handles.beta_trig_stim > 0
            handles.task_connect.sendPosVelXY([handles.trigger_beta 0 0 0]'); %[beta_trigger]  
            if handles.trigger_beta
                disp('Trigger beta!')
            end
        else
            handles.task_connect.sendPosVelXY([-1 data.lfp_cursor_kin(1,t) 0 powerOK]'); %[X pos, Y pos, empty, powerError flag]  
        end
        
        tsend=GetSecs();
        data.powerOK(t) = powerOK;

        
        handles = guidata(hObject);
        
        handles.h_plot_data = h_plot_data;
        guidata(hObject,handles);
 
    else  % not enough data collected
        data.mc_cursor_kin(:,t) = data.mc_cursor_kin(:,t-1);
        data.lfp_cursor_kin(:,t) = data.lfp_cursor_kin(:,t-1);
    end
    
    %ind = 1:length(handles.p_extractor.used_chan):handles.p_extractor.n_features;
    %handles.chan_power_plot(:,1:end-1) = handles.chan_power_plot(:,2:end);
    %handles.chan_power_plot(:,end) = data.features(ind,t);

    %PK 11-17-14 update
    %update_displays(handles,data,t);
      
    drawnow;
    
    handles = guidata(hObject);
    
    if handles.go_flag == 0
        save_stuff(data,handles)
        break;
    end

    time_elapsed = GetSecs() - t0;
    if time_elapsed <= dt
        %fprintf('Iteration time = %0.3f\n',time_elapsed);
        WaitSecs(dt - time_elapsed);
    else
        fprintf('WARNING: iteration time = %0.3f\n',time_elapsed);
    end
    
    %disp(strcat(' iteration: ', num2str(t)));
    data.itr_times(t) = GetSecs() - t0;

    if t>2 && exist('ttot','var')
        data.itr_times(t)=ttot;
   
%         if ttot>dt
%             fprintf('WARNING: itr tm = %0.3f\n',ttot)
%         end
    end
end

handles.go_flag = 0;
guidata(hObject,handles);

msg = sprintf('Time end: %s',datestr(now,'HH:MM:SS'));
update_status(handles,msg,t);

fprintf('\nReward trials: %d\n',sum(data.reward_trials));

save_stuff(data,handles)

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

% --- Executes on selection change in status.
function status_Callback(hObject, eventdata, handles)
% hObject    handle to status (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = get(hObject,'String') returns status contents as cell array
%        contents{get(hObject,'Value')} returns selected item from status


% --- Executes during object creation, after setting all properties.
function status_CreateFcn(hObject, eventdata, handles)
% hObject    handle to status (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
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
%       - mn, sd for common freq_ranges
%       - frange = [low_power1 high_power1; low_power1 high_power1 etc...
%               corresponding to means / sds above]
%   - baseline_used struct
%       -either baseline_used.abs_power ro baseline_used.frac_power
%       - mn, sd, frange above
%   - task_name
load('lfp_mc_targets.mat') 

%Which decoder was used: 
if isfield(decoder.baseline_used,'frac_targ')
    handles.fraction = decoder.baseline_used.frac_targ;
else
    handles.fraction = [0 0 0 0];
end

%Parameters of decoder: 
%Fraction
set(handles.frac_range_low_box,'String',num2str(decoder.baseline_used.power.frac_range(1)))
set(handles.frac_range_high_box,'String',num2str(decoder.baseline_used.power.frac_range(2)))
handles.frac_range_high = decoder.baseline_used.power.frac_range(2); 
handles.frac_range_low =  decoder.baseline_used.power.frac_range(1); 

%Abs Power
handles.baseline_mean = decoder.baseline_used.power.mean;
handles.baseline_targ_step = decoder.baseline_used.power.targ_step;
set(handles.mean_baseline_used_box,'String',num2str(handles.baseline_mean))
set(handles.targ_step_baseline_used_box,'String',num2str(handles.baseline_targ_step))  

%Type of decoding
set(handles.t1_frac,'Value',decoder.baseline_used.frac_targ(1));
set(handles.t2_frac,'Value',decoder.baseline_used.frac_targ(2));
set(handles.t3_frac,'Value',decoder.baseline_used.frac_targ(3));
set(handles.t4_frac,'Value',decoder.baseline_used.frac_targ(4));

%Frequency range of decoder: 
set(handles.low_power_box,'String',num2str(decoder.baseline_used.power.frange(1)))
set(handles.high_power_box,'String',num2str(decoder.baseline_used.power.frange(2)))
handles.mod_freq_range = decoder.baseline_used.power.frange; 

%Extras: Log, filt
handles.log_flag = decoder.baseline_used.power.log_flag;
set(handles.log_box,'Value',handles.log_flag)
handles.low_pass_filt = decoder.baseline_used.power.low_pass_filt;
set(handles.low_pass_filt_box,'String',num2str(handles.low_pass_filt)); 


% if handles.fraction
% %     %Parameters of decoder: 
% %     set(handles.frac_range_low_box,'String',num2str(decoder.baseline_used.frac_power.frac_range(1)))
% %     set(handles.frac_range_high_box,'String',num2str(decoder.baseline_used.frac_power.frac_range(2)))
% %     handles.frac_range_high = decoder.baseline_used.frac_power.frac_range(2); 
% %     handles.frac_range_low = decoder.baseline_used.frac_power.frac_range(1);
% %     %handles.frac_targ_step = decoder.baseline_used.frac_power.frac_targ_step;
% %     
%     %Type of decoding
% %     set(handles.fraction_box,'Value',1);
% %     set(handles.baseline_used_box,'Value',0);
% %     set(handles.baseline_from_box,'Value',0);
% %     
%     %Frequency range of decoder: 
% %     set(handles.low_power_box,'String',num2str(decoder.baseline_used.frac_power.frange(1)))
% %     set(handles.high_power_box,'String',num2str(decoder.baseline_used.frac_power.frange(2)))
% %     handles.mod_freq_range = decoder.baseline_used.frac_power.frange; 
% % 
%     %Extras: Log, filt
%     handles.log_flag = decoder.baseline_used.frac_power.log_flag;
%     set(handles.log_box,'Value',handles.log_flag)
%     handles.low_pass_filt = decoder.baseline_used.frac_power.low_pass_filt;
%     set(handles.low_pass_filt_box,'String',num2str(handles.low_pass_filt));    
% 
% else
%     %Parameters of decoder: 
%     handles.baseline_mean = decoder.baseline_used.abs_power.mean;
%     handles.baseline_targ_step = decoder.baseline_used.abs_power.targ_step;
%     set(handles.mean_baseline_used_box,'String',num2str(handles.baseline_mean))
%     set(handles.targ_step_baseline_used_box,'String',num2str(handles.baseline_targ_step))  
%     
%     %Type of decoding:       
%     set(handles.baseline_used_box,'Value',1);
%     set(handles.fraction_box,'Value',0);
%     set(handles.baseline_calc_box,'Value',0);
%     
%     %Frequency range of decoder: 
%     set(handles.low_power_box,'String',num2str(decoder.baseline_used.abs_power.frange(1)))
%     set(handles.high_power_box,'String',num2str(decoder.baseline_used.abs_power.frange(2)))
%     handles.mod_freq_range = decoder.baseline_used.abs_power.frange; 
%     
%     %Extras: 
%     handles.log_flag = decoder.baseline_used.abs_power.log_flag;
%     set(handles.log_box,'Value',handles.log_flag)
%     handles.low_pass_filt = decoder.baseline_used.abs_power.low_pass_filt;
%     set(handles.low_pass_filt_box,'String',num2str(handles.low_pass_filt));    
% end

handles.baseline_targ_conv = max(abs(target_positions{2}(2:end,2)));

set(handles.task_name_box,'String',decoder.task_name);
handles.baseline_calc_from_day = decoder.baseline_calc;
handles.baseline_used_from_day = decoder.baseline_used;


try
    handles.powercap = decoder.powercap;
catch
    handles.powercap = 1000;
    disp('no powercap in decoder :( ')
end

set(handles.power_cap_box,'String',num2str(handles.powercap));

%Experiment Parameters: 
handles.sim_lfp = decoder.sim_lfp;
%handles.assym_beta = decoder.assym_beta;
%handles.assym_beta_totpwr = decoder.assym_beta_totpwr;
set(handles.sim_lfp_box,'Value',handles.sim_lfp)

%Setup Power extractor based on gui standards.
handles.p_extractor = pow_extractor_pk(decoder.p_extractor_params.window_size,...
                                      decoder.p_extractor_params.franges,...
                                      handles.mod_freq_range,...
                                      decoder.p_extractor_params.f_max,...
                                      decoder.p_extractor_params.fs,...
                                      decoder.p_extractor_params.used_chan);
                                  

%Load channel power plot:
%handles.chan_power_plot = zeros(handles.p_extractor.n_features/length(handles.p_extractor.used_chan), 30);                             
update_status(handles,'baselines and feature extractor loaded',0);
handles.dt = str2double(get(handles.dt_box, 'String'))/1000; %Ms to sec
handles.fs = handles.p_extractor.fs;

% load channels used
handles.all_chan = 1:128;
handles.chan_offset = 65 - 9;
handles.used_chan = handles.p_extractor.used_chan';

handles.unused_chan = setxor(handles.all_chan,handles.used_chan);
handles.unused_chan = handles.unused_chan(:);

% set simulation
handles.sim_lfp = 0;
handles.sim_lfp_darpa = 0;

set(handles.used_chan_listbox,'Value',1);
set(handles.used_chan_listbox,'String',handles.used_chan + handles.chan_offset);
set(handles.unused_chan_listbox,'Value',1);
set(handles.unused_chan_listbox,'String',handles.unused_chan + handles.chan_offset);
guidata(hObject,handles);
    
% --- Executes on button press in mean_baseline_used_box.
function mean_baseline_used_box_Callback(hObject, eventdata, handles)
% hObject    handle to mean_baseline_used_box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of mean_baseline_used_box
% if handles.baseline_used_box
%     set(handles.baseline_from_box,'Value',0);
%     handles.baseline_mean = handles.baseline_used_from_day.mean;
%     set(handles.mean_baseline_used_box,'String',num2str(handles.baseline_mean))
%     handles.baseline_targ_step = handles.baseline_used_from_day.targ_step;
%     set(handles.targ_step_baseline_used_box,'String',num2str(handles.baseline_targ_step))
% else
% end

% --- Executes during object creation, after setting all properties.
function mean_baseline_used_box_CreateFcn(hObject, eventdata, handles)
% hObject    handle to mean_baseline_used_box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function targ_step_baseline_used_box_Callback(hObject, eventdata, ~)
% hObject    handle to targ_step_baseline_used_box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of targ_step_baseline_used_box as text
%        str2double(get(hObject,'String')) returns contents of targ_step_baseline_used_box as a double


% --- Executes during object creation, after setting all properties.
function targ_step_baseline_used_box_CreateFcn(hObject, eventdata, handles)
% hObject    handle to targ_step_baseline_used_box (see GCBO)
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


function high_power_box_Callback(hObject, eventdata, handles)
% hObject    handle to high_power_box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of high_power_box as text
%        str2double(get(hObject,'String')) returns contents of high_power_box as a double


% --- Executes during object creation, after setting all properties.
function high_power_box_CreateFcn(hObject, eventdata, handles)
% hObject    handle to high_power_box (see GCBO)
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
if get(hObject,'Value')    
    if isfield(handles.baseline_used_from_day,'abs_power')
        set(handles.baseline_from_box,'Value',0);
        set(handles.fraction_box,'Value',0);
        
        handles.baseline_mean = handles.baseline_used_from_day.abs_power.mean;
        set(handles.mean_baseline_used_box,'String',num2str(handles.baseline_mean))
    
        handles.baseline_targ_step = handles.baseline_used_from_day.abs_power.targ_step;
        set(handles.targ_step_baseline_used_box,'String',num2str(handles.baseline_targ_step))
    
        frange = handles.baseline_used_from_day.abs_power.frange;
        set(handles.high_power_box,'String',num2str(frange(2)));
        set(handles.low_power_box,'String',num2str(frange(1)));
    else
        set(handles.baseline_used_box,'Value',0);
        set(handles.baseline_from_box,'Value',0);
        set(handles.fraction_box,'Value',1);
        
        %Callback for load fraction
        
    end
        
else
    handles.baseline_mean = str2double(get(handles.mean_baseline_used_box,'String'));
    handles.baseline_targ_step = str2double(get(handles.targ_step_baseline_used_box,'String'));
end

guidata(hObject, handles);

% --- Executes on button press in baseline_from_box.
function baseline_from_box_Callback(hObject, eventdata, handles)
% hObject    handle to baseline_from_box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of baseline_from_box
if get(hObject,'Value') %If checked and toggled: 
    set(handles.baseline_used_box,'Value',0);
    set(handles.fraction_box,'Value',0);
    
    low = str2double(get(handles.low_power_box,'String'));
    high = str2double(get(handles.high_power_box,'String'));
    frange = handles.baseline_calc_from_day.frange;
    match = 0;
    for i = 1:size(frange,1)
        if (frange(i,1) == low) && (frange(i,2) == high)
            
            if get(handles.log_box,'Value')
                handles.baseline_mean = handles.baseline_calc_from_day.log_mean(i);
                set(handles.mean_baseline_used_box,'String',num2str(handles.baseline_mean))
            
                handles.baseline_targ_step = handles.baseline_calc_from_day.log_targ_step(i);
                set(handles.targ_step_baseline_used_box,'String',num2str(handles.baseline_targ_step))                
            
            else
                handles.baseline_mean = handles.baseline_calc_from_day.mean(i);
                set(handles.mean_baseline_used_box,'String',num2str(handles.baseline_mean))
            
                handles.baseline_targ_step = handles.baseline_calc_from_day.targ_step(i);
                set(handles.targ_step_baseline_used_box,'String',num2str(handles.baseline_targ_step))
            end
                
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
else
    handles.baseline_mean = str2double(get(handles.mean_baseline_used_box,'String'));
    handles.baseline_targ_step = str2double(get(handles.targ_step_baseline_used_box,'String'));
end
guidata(hObject, handles);


% --- Executes on selection change in used_chan_listbox.
function used_chan_listbox_Callback(hObject, eventdata, handles)
% hObject    handle to used_chan_listbox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = get(hObject,'String') returns used_chan_listbox contents as cell array
%        contents{get(hObject,'Value')} returns selected item from used_chan_listbox


% --- Executes during object creation, after setting all properties.
function used_chan_listbox_CreateFcn(hObject, eventdata, handles)
% hObject    handle to used_chan_listbox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in unused_chan_listbox.
function unused_chan_listbox_Callback(hObject, eventdata, handles)
% hObject    handle to unused_chan_listbox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = get(hObject,'String') returns unused_chan_listbox contents as cell array
%        contents{get(hObject,'Value')} returns selected item from unused_chan_listbox


% --- Executes during object creation, after setting all properties.
function unused_chan_listbox_CreateFcn(hObject, eventdata, handles)
% hObject    handle to unused_chan_listbox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in remove_all_button.
function remove_all_button_Callback(hObject, eventdata, handles)
% hObject    handle to remove_all_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in remove_chan_button.
function remove_chan_button_Callback(hObject, eventdata, handles)
% hObject    handle to remove_chan_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

inds = get(handles.used_chan_listbox,'Value');
if ~isempty(handles.used_chan)
    handles.unused_chan = [handles.unused_chan; handles.used_chan(inds)];
    handles.unused_chan = sort(handles.unused_chan);

    % remove corresponding channels from each feature extractor
    bad_chan = handles.used_chan(inds);

    % remove corresponding columns from decoder
    bad_inds = zeros(handles.p_extractor.n_features,1);
    for k = bad_chan(:)'
        bad_inds = bad_inds + (handles.p_extractor.chnfeat_index == k);
    end
    %bad_rows = find(bad_inds == 1);

    % remove from extractor
    handles.p_extractor.remove_channels(bad_chan);

    handles.used_chan(inds) = [];
    set(handles.used_chan_listbox,'Value',1);
    set(handles.used_chan_listbox,'String',handles.used_chan + handles.chan_offset);
    set(handles.unused_chan_listbox,'String',handles.unused_chan + handles.chan_offset);
end
%Remove from neural data request: 
handles.neural_connect.add_chan(handles.used_chan,'replace_all');

guidata(hObject,handles);

% --- Executes on button press in add_chan_button.
function add_chan_button_Callback(hObject, eventdata, handles)
% hObject    handle to add_chan_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

inds = get(handles.unused_chan_listbox,'Value');
if ~isempty(handles.unused_chan)
    handles.used_chan = [handles.used_chan; handles.unused_chan(inds)];
    handles.used_chan = sort(handles.used_chan);
    handles.unused_chan(inds) = [];
    set(handles.unused_chan_listbox,'Value',1);
    set(handles.used_chan_listbox,'String',handles.used_chan + handles.chan_offset);
    set(handles.unused_chan_listbox,'String',handles.unused_chan + handles.chan_offset);
end

%Add to  neural data request: 
handles.neural_connect.add_chan(handles.used_chan,'replace_all')

guidata(hObject,handles);


% --- Executes on button press in add_all_button.
function add_all_button_Callback(hObject, eventdata, handles)
% hObject    handle to add_all_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% 
% % --- Executes during object deletion, before destroying properties.
% function figure1_DeleteFcn(hObject, eventdata, handles)
% % hObject    handle to figure1 (see GCBO)
% % eventdata  reserved - to be defined in a future version of MATLAB
% % handles    structure with handles and user data (see GUIDATA)
% disp('cleaning up...');
% handles.neural_connect.close();
% handles.task_connect.close();
% 
% % try  % if go_Callback finished, then handles.neural_file isn't a valid file ID
% %     fclose(handles.neural_file);
% % end
% 
% try
%     fclose(handles.clda_alg.udp_obj);
%     delete(handles.clda_alg.udp_obj);
% end
% 
% disp('finished');



% --- Executes on button press in update_baselines.
function update_baselines_Callback(hObject, eventdata, handles)
% hObject    handle to update_baselines (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.baseline_mean = str2double(get(handles.mean_baseline_used_box,'String'));
handles.baseline_targ_step = str2double(get(handles.targ_step_baseline_used_box,'String'));

msg = sprintf('Updated Baseline Params: Mean %d, Targ Step %d',...
    str2double(get(handles.mean_baseline_used_box,'String')),...
    str2double(get(handles.targ_step_baseline_used_box,'String')));
update_status(handles,msg,handles.t);

guidata(hObject,handles);

% --- Executes during object deletion, before destroying properties.
function figure1_DeleteFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
disp('cleaning up...');
handles.neural_connect.close_sock();
handles.task_connect.close();

disp('finished');


% --- Executes on button press in log_box.
function log_box_Callback(hObject, eventdata, handles)
% hObject    handle to log_box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hint: get(hObject,'Value') returns toggle state of log_box

if get(hObject,'Value') %Turned on: 
    handles.log_flag = 1;
    msg = sprintf('log_flag ON');
    update_status(handles,msg,0);
    
    if get(handles.baseline_from_box,'Value')
            low = str2double(get(handles.low_power_box,'String'));
            high = str2double(get(handles.high_power_box,'String'));
            frange = handles.baseline_calc_from_day.frange;
            match = 0;
            
            for i = 1:size(frange,1)
                if (frange(i,1) == low) && (frange(i,2) == high)
                    handles.baseline_mean = handles.baseline_calc_from_day.log_mean(i);
                    set(handles.mean_baseline_used_box,'String',num2str(handles.baseline_mean))
            
                    handles.baseline_targ_step = handles.baseline_calc_from_day.log_targ_step(i);
                    set(handles.targ_step_baseline_used_box,'String',num2str(handles.baseline_targ_step))
                
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
            
    elseif get(handles.baseline_used_box,'Value')
       if and(isfield(handles.baseline_used_from_day,'abs_power'),handles.baseline_used_from_day.abs_power.log_flag)
                handles.baseline_mean = handles.baseline_used_from_day.abs_power.mean;
                handles.baseline_targ_step = handles.baseline_used_from_day.abs_power.targ_step;
                set(handles.mean_baseline_used_box,'String',num2str(handles.baseline_mean))
                set(handles.targ_step_baseline_used_box,'String',num2str(handles.baseline_targ_step))
       
       elseif and(isfield(handles.baseline_used_from_day,'frac_power'),handles.baseline_used_from_day.frac_power.log_flag)
                handles.frac_range_high = handles.baseline_used_from_day.frac_power.frac_range(2);
                handles.frac_range_low = handles.baseline_used_from_day.frac_power.frac_range(1);    
                set(handles.frac_range_high_box,'String',num2str(decoder.baseline_used.frac_power.frac_range(2)))
                set(handles.fraction_box,'Value',1);
                set(hObject,'Value',0)
                
       else
            disp('huh?')
            disp('log_flag not used on this day :/')
            set(hObject,'Value',0)
       end
       
    end
    
else %Turned OFF        
    handles.log_flag = 0;
    msg = sprintf('log10 OFF');
    update_status(handles,msg,0);
    
    if get(handles.baseline_from_box,'Value')
            low = str2double(get(handles.low_power_box,'String'));
            high = str2double(get(handles.high_power_box,'String'));
            frange = handles.baseline_calc_from_day.frange;
            match = 0;
            
            for i = 1:size(frange,1)
                if (frange(i,1) == low) && (frange(i,2) == high)
                    handles.baseline_mean = handles.baseline_calc_from_day.mean(i);
                    set(handles.mean_baseline_used_box,'String',num2str(handles.baseline_mean))
            
                    handles.baseline_targ_step = handles.baseline_calc_from_day.targ_step(i);
                    set(handles.targ_step_baseline_used_box,'String',num2str(handles.baseline_targ_step))
                
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
            
    elseif get(handles.baseline_used_box,'Value')
        if handles.baseline_used_from_day.log_flag
            disp('Only log used during this day :/')
            set(hObject,'Value',1)
        else
            handles.baseline_mean = handles.baseline_used_from_day.mean;
            handles.baseline_targ_step = handles.baseline_used_from_day.targ_step;
            set(handles.mean_baseline_used_box,'String',num2str(handles.baseline_mean))
            set(handles.targ_step_baseline_used_box,'String',num2str(handles.baseline_targ_step))       
        end
    end
end

guidata(hObject,handles);


function update_displays(handles,data,t)
    %Update Task Display:
    set(handles.h_mc_target,'xdata',data.mc_target_pos(1,t),...
                         'ydata',data.mc_target_pos(2,t));
    set(handles.h_mc_cursor,'xdata',data.mc_cursor_kin(1,t),...
                         'ydata',data.mc_cursor_kin(2,t));
    set(handles.h_lfp_target,'xdata',-.8,...
                         'ydata',data.lfp_target_pos(2,t));
    set(handles.h_lfp_cursor,'xdata',-.8,...
                         'ydata',data.lfp_cursor_kin(1,t));

%     set(handles.h_reward,'xdata',(1:t)*handles.dt,...
%                          'ydata',data.trial_reward_rate(1:t));
%     set(handles.h_init,'xdata',(1:t)*handles.dt,...
%                        'ydata',data.trial_init_rate(1:t));

    %Update Channel display: 
    ind = 1:30; 
    set(handles.h_plot_total,'xdata',ind,'ydata',handles.h_plot_data(1,:));
    set(handles.h_plot_band,'xdata',ind,'ydata',handles.h_plot_data(2,:));   
    set(handles.h_plot_fraction,'xdata',ind,'ydata',handles.h_plot_data(3,:));
       
    %Update Histogram:
    %bins=[-1.5:.1:1.5];
    %hist(handles.power_hist_disp,data.lfp_cursor_kin(1,1:t),bins);
    
%Save Stuff: 
function save_stuff(data, handles)
    data.total_itrs = handles.t;
    data.messages = get(handles.status,'String');

    %handles.data = data;

    %Save data:
    save(handles.data_fullname,'data');

    %Save decoder:
    decoder = struct();
    baseline_calc=struct();
    baseline_calc.frange = handles.p_extractor.ranges;

    %Calculate data mean / targ_step:
    [mean, targ_step, log_mean, log_targ_step] = calc_baseline(data.features,handles); 
    baseline_calc.mean = mean; %feat x 1 array
    baseline_calc.targ_step = targ_step;%feat x 1 array
    baseline_calc.log_mean = log_mean;
    baseline_calc.log_targ_step = log_targ_step;
      
    %Baseline data actually used: 
    baseline_used=struct();
    baseline_used.frac_targ = [handles.t1_frac, handles.t2_frac, handles.t3_frac, handles.t4_frac];
    baseline_used.power = struct();
    baseline_used.power.frange(1) = str2double(get(handles.low_power_box,'String'));
    baseline_used.power.frange(2) = str2double(get(handles.high_power_box,'String'));
    baseline_used.power.log_flag = handles.log_flag;
    baseline_used.power.frac_range = [handles.frac_range_low handles.frac_range_high];
    baseline_used.power.low_pass_filt = handles.low_pass_filt;
    baseline_used.power.mean = str2double(get(handles.mean_baseline_used_box,'String'));
    baseline_used.power.targ_step = str2double(get(handles.targ_step_baseline_used_box,'String'));

    p_extractor_params=struct();
    p_extractor_params.franges = handles.p_extractor.ranges; %all freq ranges used in data.features
    p_extractor_params.window_size = handles.p_extractor.width_t;
    p_extractor_params.fs = handles.p_extractor.fs;
    p_extractor_params.used_chan = handles.p_extractor.used_chan;
    p_extractor_params.f_max = handles.p_extractor.params.fpass(2);

    task_name = get(handles.task_name_box,'String');

    decoder.baseline_used = baseline_used;
    decoder.baseline_calc = baseline_calc;
    decoder.p_extractor_params = p_extractor_params;
    decoder.task_name = task_name;
    decoder.sim_lfp = handles.sim_lfp;
    %decoder.assym_beta = handles.assym_beta;
    %decoder.assym_beta_totpwr = handles.assym_beta_totpwr;
    decoder.powercap = handles.powercap;
    
    save(handles.decoder_fullname,'decoder');


% --- Executes on button press in fraction_box.
function fraction_box_Callback(hObject, eventdata, handles)
% hObject    handle to fraction_box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of fraction_box

if get(hObject,'Value')
    handles.fraction = 1;
else
   handles.fraction = 0;
end

guidata(hObject, handles);



function frac_mean_box_Callback(hObject, eventdata, handles)
% hObject    handle to frac_mean_box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of frac_mean_box as text
%        str2double(get(hObject,'String')) returns contents of frac_mean_box as a double


% --- Executes during object creation, after setting all properties.
function frac_mean_box_CreateFcn(hObject, eventdata, handles)
% hObject    handle to frac_mean_box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function frac_range_low_box_Callback(hObject, eventdata, handles)
% hObject    handle to frac_range_low_box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of frac_range_low_box as text
%        str2double(get(hObject,'String')) returns contents of frac_range_low_box as a double


% --- Executes during object creation, after setting all properties.
function frac_range_low_box_CreateFcn(hObject, eventdata, handles)
% hObject    handle to frac_range_low_box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in update_frac.
function update_frac_Callback(hObject, eventdata, handles)
% hObject    handle to update_frac (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%handles.frac_mean = str2double(get(handles.frac_mean_box,'String'));
handles.frac_targ_step = str2double(get(handles.frac_range_high_box,'String')) - str2double(get(handles.frac_range_low_box,'String'));
handles.frac_range_low = str2double(get(handles.frac_range_low_box,'String'));
handles.frac_range_high = str2double(get(handles.frac_range_high_box,'String'));

msg = sprintf('Fraction baselines updated');
update_status(handles,msg,0);
guidata(hObject, handles);



function frac_range_high_box_Callback(hObject, eventdata, handles)
% hObject    handle to frac_range_high_box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of frac_range_high_box as text
%        str2double(get(hObject,'String')) returns contents of frac_range_high_box as a double


% --- Executes during object creation, after setting all properties.
function frac_range_high_box_CreateFcn(hObject, eventdata, handles)
% hObject    handle to frac_range_high_box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function low_pass_filt_box_Callback(hObject, eventdata, handles)
% hObject    handle to low_pass_filt_box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of low_pass_filt_box as text
%        str2double(get(hObject,'String')) returns contents of low_pass_filt_box as a double


% --- Executes during object creation, after setting all properties.
function low_pass_filt_box_CreateFcn(hObject, eventdata, handles)
% hObject    handle to low_pass_filt_box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in update_filt.
function update_filt_Callback(hObject, eventdata, handles)
% hObject    handle to update_filt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

handles.low_pass_filt = str2double(get(handles.low_pass_filt_box,'String'));
msg = sprintf('Updated Filter: %d',handles.low_pass_filt);
update_status(handles,msg,0);
guidata(hObject, handles);


% --- Executes on button press in sim_lfp_box.
function sim_lfp_box_Callback(hObject, eventdata, handles)
% hObject    handle to sim_lfp_box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hint: get(hObject,'Value') returns toggle state of sim_lfp_box
handles.sim_lfp = get(hObject,'Value');
guidata(hObject, handles);

% --- Executes on button press in assym_beta_box.
function assym_beta_box_Callback(hObject, eventdata, handles)
% hObject    handle to assym_beta_box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of assym_beta_box
handles.assym_beta = get(hObject,'Value');
handles.assym_beta_totpwr = str2double(get(handles.assym_beta_totpwr_box,'String'));
guidata(hObject, handles);


function assym_beta_box_totpwr_Callback(hObject, eventdata, handles)
% hObject    handle to assym_beta_box_totpwr (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of assym_beta_box_totpwr as text
%        str2double(get(hObject,'String')) returns contents of assym_beta_box_totpwr as a double


% --- Executes during object creation, after setting all properties.
function assym_beta_box_totpwr_CreateFcn(hObject, eventdata, handles)
% hObject    handle to assym_beta_box_totpwr (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end




% --- Executes on button press in update_totpwr.
function update_totpwr_Callback(hObject, eventdata, handles)
% hObject    handle to update_totpwr (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.assym_beta_totpwr = str2double(get(handles.assym_beta_totpwr_box,'String'));
guidata(hObject, handles);


function assym_beta_totpwr_box_Callback(hObject, eventdata, handles)
% hObject    handle to assym_beta_totpwr_box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of assym_beta_totpwr_box as text
%        str2double(get(hObject,'String')) returns contents of assym_beta_totpwr_box as a double


% --- Executes during object creation, after setting all properties.
function assym_beta_totpwr_box_CreateFcn(hObject, eventdata, handles)
% hObject    handle to assym_beta_totpwr_box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function power_cap_box_Callback(hObject, eventdata, handles)
% hObject    handle to power_cap_box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of power_cap_box as text
%        str2double(get(hObject,'String')) returns contents of power_cap_box as a double


% --- Executes during object creation, after setting all properties.
function power_cap_box_CreateFcn(hObject, eventdata, handles)
% hObject    handle to power_cap_box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in power_cap_update_box.
function power_cap_update_box_Callback(hObject, eventdata, handles)
% hObject    handle to power_cap_update_box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

handles.powercap = str2double(get(handles.power_cap_box,'String'));
msg = sprintf('Update PowerCap to %d', handles.powercap);
update_status(handles,msg,0);
guidata(hObject, handles);
disp(handles.powercap)



% --- Executes on button press in t1_frac.
function t1_frac_Callback(hObject, eventdata, handles)
% hObject    handle to t1_frac (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of t1_frac
if get(hObject,'Value')
    handles.fraction(1)=1;
else
    handles.fraction(1)=0;
end
guidata(hObject, handles);



% --- Executes on button press in t2_frac.
function t2_frac_Callback(hObject, eventdata, handles)
% hObject    handle to t2_frac (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of t2_frac
if get(hObject,'Value')
    handles.fraction(2)=1;
else
    handles.fraction(2)=0;
end
guidata(hObject, handles);


% --- Executes on button press in t3_frac.
function t3_frac_Callback(hObject, eventdata, handles)
% hObject    handle to t3_frac (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of t3_frac
if get(hObject,'Value')
    handles.fraction(3)=1;
else
    handles.fraction(3)=0;
end
guidata(hObject, handles);


% --- Executes on button press in t4_frac.
function t4_frac_Callback(hObject, eventdata, handles)
% hObject    handle to t4_frac (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of t4_frac
if get(hObject,'Value')
    handles.fraction(4)=1;
else
    handles.fraction(4)=0;
end
guidata(hObject, handles);


% --- Executes on button press in sim_lfp_darpa.
function sim_lfp_darpa_Callback(hObject, eventdata, handles)
% hObject    handle to sim_lfp_darpa (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of sim_lfp_darpa
if get(hObject,'Value')
    handles.sim_lfp_darpa = 1;
else
    handles.sim_lfp_darpa = 0;
end
guidata(hObject, handles);


% --- Executes on button press in beta_trig_stim_check.
function beta_trig_stim_check_Callback(hObject, eventdata, handles)
% hObject    handle to beta_trig_stim_check (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of beta_trig_stim_check
if get(hObject,'Value')
    handles.beta_trig_stim = 1;
else
    handles.beta_trig_stim = 0;
end
guidata(hObject, handles);


function beta_trig_stim_val_Callback(hObject, eventdata, handles)
% hObject    handle to beta_trig_stim_val (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of beta_trig_stim_val as text
%        str2double(get(hObject,'String')) returns contents of beta_trig_stim_val as a double
handles.beta_trig_stim_val = str2double(get(hObject,'String'));
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function beta_trig_stim_val_CreateFcn(hObject, eventdata, handles)
% hObject    handle to beta_trig_stim_val (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in beta_trig_stim_update.
function beta_trig_stim_update_Callback(hObject, eventdata, handles)
% hObject    handle to beta_trig_stim_update (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

handles.beta_trig_stim_val = str2double(get(handles.beta_trig_stim_val,'String'));
tmp = get(handles.beta_trig_stim_check, 'Value');
handles.beta_trig_stim = tmp;
printf('%f', tmp)
