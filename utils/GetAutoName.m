% function to get filename
function [data_filename decoder_filename neural_filename] = GetAutoName(bmi_flag,subject)
    % get experiment number
    dlist = dir([subject '/data']);
    str = ['dat' datestr(date,'mmddyy')];
    ex = [];
    for k = 1:length(dlist)
        if strmatch(str,dlist(k).name)
            ex(end+1) = dlist(k).name(10);
        end
    end
    if ~isempty(ex)
        curex = max(ex)+1;
    else
        curex = 'a';
    end
    
    if bmi_flag
        suffix = '_bmi';
    else
        suffix = '_man';
    end
    
    data_filename    = ['dat' datestr(date,'mmddyy') curex suffix '.mat'];
    decoder_filename = ['dec' datestr(date,'mmddyy') curex suffix '.mat'];
    neural_filename  = ['neu' datestr(date,'mmddyy') curex suffix '.dat'];