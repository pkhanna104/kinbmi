function [lfp_kin_vars,h_plot_data, powerOK] = calc_lfp_cursor(p_extractor,features,handles,faux_lfp,lfp_targ_pos)


% features is chan x features
% input: calc_lfp_cursor(p_extractor,data.features(:,t),handles);

%Find task_relevant power range
task_ind = find(p_extractor.task_ranges>0);

%Find feature inds that correspond to these ranges:
feat_inds = find(p_extractor.ftfeat_index == task_ind);

%Mean across channels
task_feat = mean(features(feat_inds));

%Compare current power to baseline: 
%Zscore, and then set a Zscore of 1 = first target



%Which LFP target? 
if lfp_targ_pos(2)<-0.2
    lfptarg=4;
elseif lfp_targ_pos(2)<0.1
    lfptarg=3;
elseif lfp_targ_pos(2)<0.5
    lfptarg=2;
else
    lfptarg=1;
end

%Calculate total Power: 
total_power_ind = length(p_extractor.range_inds); 
total_inds = find(p_extractor.ftfeat_index == total_power_ind);


%Mean across channels
total_feat = mean(features(total_inds)); 


%If log? 
if handles.log_flag
    total_feat = log10(total_feat+10^-9);
    task_feat = log10(task_feat+10^-9);
end


if handles.fraction(lfptarg)

    %Fraction 
    lfppos = task_feat/total_feat;
    disp('fraction!')
else
    lfppos = task_feat/10;
    disp('power!')
end

%Center fraction:
dmn = lfppos - (mean([handles.frac_range_high, handles.frac_range_low]));

%Map centered fraction to target range
%baseline_convfrac2targ is 0.8125
%Here, map [-1, 1] to [0, .35]
zscore = dmn*(2*handles.baseline_targ_conv)/(handles.frac_range_high - handles.frac_range_low);

% % % %     OLD FRACTION WAY OF DOING IT: 
% % % %     %Here, map [-1, 1] to baseline_targ_step is [.25, .75] of CDF
% % % %     zsc = (lfppos - handles.baseline_mean)/handles.baseline_targ_step;
% % % % 
% % % %     %baseline_convfrac2targ is 0.8125
% % % %     zscore = zsc*handles.baseline_targ_conv;


if total_feat > handles.powercap
    powerOK = 0;
else
    powerOK = 1;
end

%Now map lfppos to target space:
handles.h_plot_data(:,1:end-1) = handles.h_plot_data(:,2:end);
h_plot_data = handles.h_plot_data;
h_plot_data(:,1:end-1) = handles.h_plot_data(:,2:end);
h_plot_data(:,end) = [total_feat;task_feat;lfppos];

lfp_kin_vars(1) = zscore; 
lfp_kin_vars(2) = 0;


if handles.sim_lfp
    lfp_kin_vars(1) = faux_lfp(handles.t);
    lfp_kin_vars(2) = 0;
end

if handles.sim_lfp_darpa
    lfp_kin_vars(1) = faux_lfp(handles.t);
    lfp_kin_vars(2) = 0;
end


end


