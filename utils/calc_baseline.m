function [mean, targ_step, log_mean, log_targ_step] = calc_baseline(features,handles)

%Assumes input to function is in a features x time array: 

f_ind = handles.p_extractor.ftfeat_index; 

mean = zeros(size(handles.p_extractor.ranges,1),1);
targ_step = zeros(size(handles.p_extractor.ranges,1),1);

log_mean = zeros(size(handles.p_extractor.ranges,1),1);
log_targ_step = zeros(size(handles.p_extractor.ranges,1),1);

for i = 1:size(handles.p_extractor.ranges,1)
    ind = find(f_ind == i);
    
    x_log = log10(features(ind,:)+10^-9);
    x_log = x_log(:);
    
    x = features(ind,:);
    x = x(:);
    
    xi = linspace(min(x),max(x),1000);
    xi_log = linspace(min(x_log),max(x_log),1000);

    f = ksdensity(x,xi,'function','cdf');
    f_log = ksdensity(x_log,xi_log,'function','cdf');
    try
        mean(i) = xi(min(find(f>0.5)));
        foo = xi(min(find(f>0.25)));
        targ_step(i) = mean(i) - foo; 
    
        log_mean(i)= xi_log(min(find(f_log>0.5)));
        foo = xi_log(min(find(f_log>0.25)));
        log_targ_step(i) = log_mean(i)-foo;
    catch
        disp('ERROR in CALC_BASELINE');
        mean(i) = 0;
        targ_step(i)=0;
        log_mean(i) =0;
        log_targ_step(i)=0;
    end
        
    
end