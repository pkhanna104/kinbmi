classdef pow_extractor_pk < feature_extractor
% mtm method for power spectrum extraction
    
    properties
        width;    % running average filter width (# of pts)
        width_t;  % running average filter width (in seconds)
        ranges;    % freq ranges
        task_ranges; %boolean, same shape as freq ranges used for task
        range_inds;
        params;   % struct containing parameters for mtm
        used_chan_array;  % cell array of arrays of used channels for each range
        chnfeat_index; 
        ftfeat_index;
    end
    
    methods
        function obj = pow_extractor_pk(width_t, f_ranges, tk_ranges, f_max, fs, used_chan)
            width = round(width_t*fs);
            
            % determine frequencies of interest
            nfft=max(2^(nextpow2(width)),width);
            [f,findx]=getfgrid(fs,nfft,[0 f_max]);
            
            % frequency range indices
            bandc = {};
            for c = 1:size(f_ranges,1)
                bandc{c} = find((f >= f_ranges(c,1)) & (f <= f_ranges(c,2)));
            end
                        
            obj.range_inds = bandc;                                      
            obj.fs         = fs;
            obj.width      = width;
            obj.width_t    = width_t;
                        
            obj.ranges     = f_ranges;
            obj.params     = struct('fpass',[0 f_max],'Fs',fs,'tapers',[3 5]);
            obj.n_features = length(used_chan) * size(f_ranges,1);
            disp(strcat('n_features',num2str(obj.n_features)));
            obj.used_chan  = used_chan;
            obj.used_chan_array = {}; % have every frequency use same channels for now
            for c = 1:size(f_ranges,1)
                obj.used_chan_array{c,1} = used_chan;
            end
            
            obj.chnfeat_index = repmat(obj.used_chan(:),(size(f_ranges,1)),1);
            obj.ftfeat_index = reshape(repmat(1:size(obj.ranges,1),length(obj.used_chan),1),size(obj.ranges,1)*length(obj.used_chan),1);
          
            obj.task_ranges = zeros(length(f_ranges),1);
            for i =1:size(f_ranges,1)
                for j = 1:size(tk_ranges,1)
                    if f_ranges(i,:) == tk_ranges(j,:)
                        obj.task_ranges(i) = 1;
                    end
                end
            end
            
        end                        

        function features = extract_features(obj,recent_neural)
            % edited by PK, 4-15-14
            % compute power spectrum using mtm method
            
            x = recent_neural.read(obj.width)';
            if (obj.ranges(1,2)-obj.ranges(1,1) < 10)
                obj.params.pad = 2;
            end
            ind_chan = 1:length(obj.used_chan);
            
            [S,f] = mtspectrumc(x(:,ind_chan),obj.params);
            
            % compute average power of each band of interest
            % pow = zeros(size(obj.ranges,1),size(S,2));
            features = zeros(obj.n_features,1);
            cur = 0;
            for c = 1:size(obj.ranges,1)
                temp = S(obj.range_inds{c},:);    
                temp = sum(temp,1); %SUM across frequencies
                %temp = mean(temp,1) <--- this is what Sid/Kelvin used
%                 if obj.log_flag
%                     feat = log10(temp);
%                 else
%                     feat = temp;
%                 end : MOVE LOG TO calc_lfp_cursor
                feat = temp;
                features( (1 : length(feat)) + cur ) = feat;
                cur = cur + length(feat);                                
            end
            
            if cur ~= obj.n_features
                error('Incorrect number of features')
            end
        end        
        
        function remove_channels(obj,bad_chan)
            % removes bad_chan from every frequency range if they're using it
            n_features = 0;
            for k = 1 : size(obj.ranges,1)
                [c ia ib] = setxor(obj.used_chan_array{k},bad_chan);
                obj.used_chan_array{k} = obj.used_chan_array{k}(ia);
                n_features = n_features + length(obj.used_chan_array{k});
            end
            [c ia ib] = setxor(obj.used_chan,bad_chan);
            obj.used_chan = obj.used_chan(ia);   
            
            % update n_features
            obj.n_features = n_features;
            obj.chnfeat_index = repmat(obj.used_chan(:),(size(obj.ranges,1)),1);
        end
 
    end
end