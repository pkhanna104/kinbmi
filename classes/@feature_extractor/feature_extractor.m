classdef feature_extractor < handle

    properties
        fs          % sampling frequency of neural data to be filtered  
        used_chan   % list of used channels
        n_features  % total # of features (= length(used_chan) x features/chan)
        use_differential_feat = 0;  % flag to use one channel's features as a
                                    % references for features extracted from 
                                    % other channels
        differential_chan  % channel to use as a reference

        perform_car = 0;
    end

%     methods(Abstract)
%         extract_features(obj,neural)
%     end
    
    methods
        function remove_channels(obj, bad_chan)
            % remove bad channels if exists
            [c ia ib] = setxor(obj.used_chan, bad_chan);
            obj.used_chan = obj.used_chan(ia);
            obj.n_features = obj.used_chan;
        end                          
    end
   
end
