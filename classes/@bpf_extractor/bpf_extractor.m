classdef bpf_extractor < feature_extractor
    
    properties
        width    % running average filter width (# of pts)
        width_t  % running average filter width (in seconds)
        h        % impuse response
        range    % freq ranges
    end

    methods
        function obj = bpf_extractor(width_t,order,f_range,fs,used_chan)
            width = round(width_t*fs);
            Wn = f_range * 2 / fs;
            [Bf Af] = butter(order,Wn);
            imp = [1 ; zeros(width - 1,1)];
                        
            obj.h = filter(Bf,Af,imp);                        
            obj.fs      = fs;
            obj.width   = width;
            obj.width_t = width_t;
            obj.used_chan = used_chan;
            obj.n_features = length(used_chan);
            obj.range = f_range;
        end
        
        function features = extract_features(obj,recent_neural)
            neural = recent_neural.read(obj.width);            
            features = neural(obj.used_chan,:)*flipud(obj.h(:));
            % may want to perform CAR first (on used channels only)
%             features = recent_neural(obj.used_chan,:)*flipud(obj.h);
        end
    end
   
end
        