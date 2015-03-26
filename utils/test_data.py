#Extract data

tag = '090314a_bmi.mat'

dat = sio.loadmat('dat'+tag)
data = dat['data'][0][0]

ft = data['features']
curs = data['lfp_cursor_kin']

dec = sio.loadmat('dec'+tag)
decoder = dec['decoder'][0][0]


task_ind = find(p_extractor.task_ranges>0);
feat_inds = find(p_extractor.ftfeat_index == task_ind);
task_feat = mean(features(feat_inds));

total_power_ind = length(p_extractor.range_inds); 
total_inds = find(p_extractor.ftfeat_index == total_power_ind);

total_feat = mean(features(total_inds)); 


%Total power: 
lfppos_pwr = task_feat;
zsc = (lfppos - handles.baseline_mean)/handles.baseline_targ_step;
zscore = zsc*handles.baseline_targ_conv;

%Fraction: 
lfppos = task_feat/total_feat;
dmn = lfppos - (mean([handles.frac_range_high, handles.frac_range_low]));
zscore = dmn*(2*handles.baseline_targ_conv)/(handles.frac_range_high - handles.frac_range_low);

%Hacked Fraction
lfppos = task_feat/10;
dmn = lfppos - (mean([handles.frac_range_high, handles.frac_range_low]));
zscore = dmn*(2*handles.baseline_targ_conv)/(handles.frac_range_high - handles.frac_range_low);
