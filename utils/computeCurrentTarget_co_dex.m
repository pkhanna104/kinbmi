function [tarloc, lastTar, hold_flag] = computeCurrentTarget_co_dex(events,targets,lastTar,hold_flag)
% function to compute current target based on events      
    targOffset = 62;
    if ~isempty(events)
        curEv = events(end);    % take last event
        tar = find(events > targOffset,1,'last');
        if ~isempty(tar)
            lastTar = events(tar) - targOffset;
        end

        if (curEv == 5) || (curEv == 6)
            % head to lastTar
            tarloc = targets(lastTar,:);
            hold_flag = 0;

        elseif (curEv == 7)
            % should technically hold
            tarloc = targets(lastTar,:);
            hold_flag = 1;

        elseif (curEv == 15)
            % should technically hold at center
            tarloc = targets(1,:);
            hold_flag = 1;
            
        else
            tarloc = targets(1,:);
            hold_flag = 0;
        end
    else
        tarloc = [];
    end
        
