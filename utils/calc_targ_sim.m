function [targ_ind] = calc_targ_sim(targ_loc)

targ_loc(isnan(targ_loc)) = 10;
t = unique(targ_loc);
targ_ind = zeros(1,length(targ_loc));

for i = 1:length(t)
    targ_ind(targ_loc==t(i))=i;
end

cnt = 0;
for tm = 1:length(targ_ind)
    
    if cnt > 20
        flag_reind = 1;
        disp('flag_reind on')
    else 
        flag_reind = 0;
    end
    
    if flag_reind > 0 && targ_ind(tm) < 6
        disp('reindexing')
        targ_ind(tm-5:tm) = targ_ind(tm);
        flag_reind = 0;
        cnt = 0;
    end
    
    if targ_ind(tm) > 5
        cnt = cnt + 1;
    else
        cnt = 0;
    end
    
end

    
end
        
