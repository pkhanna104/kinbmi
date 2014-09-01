classdef combo_extractor < feature_extractor
    
    properties
        extractors;
        chnfeat_index;  % keeps track of which index is which channel
        n_decoders;     % number of decoders
        feat2Dec;
        description;    % description
        zscore;         % zscore flag
    end
    
    methods
        function obj = combo_extractor(filename,dec,fs,used_chan,zscore)
            
            % load spec file
            params = load(filename);
            
            % set zscore flag
            obj.zscore = zscore;
            
            % make combo_extractor for a specific decoder, 0 for all
            if dec > 0
                params = params( params(:,1) == dec, :);
                params(:,1) = 1;
            end            
            
            % get number of decoders
            n_decoders = length(unique(params(:,1)));
            
            % figure out the number of unique preproc methods
            params_pow = params( params(:,2) == 2, :);  % deal with the power spectrum separately
            params_nopow = params( params(:,2) ~= 2,:);
            [preprocs m decpreproc] = unique(params_nopow(:,2:end),'rows');

            % translate into actual params
            for k = 1:size(preprocs,1)
                switch preprocs(k,1)
                    case 1  % bpf                                                                                                
                        obj.extractors{k} = bpf_extractor(preprocs(k,2), preprocs(k,3), preprocs(k,4:5), fs, used_chan);
                    case 3  % SG filter
                        obj.extractors{k} = lfc_extractor(preprocs(k,2), preprocs(k,3), fs, used_chan);
                    case 4
                        obj.extractors{k} = lmp_extractor(preprocs(k,2), fs, used_chan);
                end
            end
            nonPowFeats = length(obj.extractors);
            
            % remember which decoder uses which features            
            for k = 1:n_decoders
                obj.feat2Dec{k} = decpreproc( params_nopow(:,1) == k );        
            end
            
            % check if we need power spectrum            
            if size(params_pow,1) >= 1
                freqRanges = unique(params_pow(:,5:6),'rows');
                obj.extractors{nonPowFeats+1} = pow_extractor(params_pow(1,3), freqRanges, 200, params_pow(1,4), fs, used_chan);
                totPowFeats = size(freqRanges,1);
            else
                totPowFeats = 0;
            end
            
            % determine which decoder wants what freq band
            for k = 1:size(params_pow,1)
                powfeats = find(freqRanges(:,1) == params_pow(k,5) & freqRanges(:,2) == params_pow(k,6)) + nonPowFeats;
                obj.feat2Dec{ params_pow(k,1) }(end+1,1) = powfeats;
            end
            
            % put together pretty display message
            j=1;
            for k = 1:n_decoders
                msg{j} = ['Decoder ' num2str(k) ':'];
                j=j+1;
                for m = 1:length(obj.feat2Dec{k})
                    mlist = obj.feat2Dec{k};
                    meth = mlist(m);
                    if  meth > nonPowFeats  % power
                        msg{j} = sprintf('Power: %s', num2str(obj.extractors{nonPowFeats+1}.ranges(meth - nonPowFeats,:)));
                    elseif strcmp(class(obj.extractors{meth}),'bpf_extractor')
                        msg{j} = sprintf('Bandpass: %s', num2str(obj.extractors{meth}.range));
                    else
                        msg{j} = sprintf('%s', class(obj.extractors{meth}));
                    end
                    j=j+1;
                end
                j=j+1;
            end
            obj.description = msg;
            
            
            obj.used_chan = used_chan;            
            obj.fs = fs;
            obj.n_features = length(used_chan) * (nonPowFeats + totPowFeats);
            obj.chnfeat_index = repmat(used_chan(:),(nonPowFeats + totPowFeats),1);
            obj.n_decoders = n_decoders;
        end
            
        function features = extract_features(obj,recent_neural)
            % loop through and extract features from each feature extractor
            
            if obj.use_differential_feat  % assumes the same features
                                          % are extracted from all chans
                features = zeros(obj.n_features - (obj.n_features/length(obj.used_chan)),1);
                diff_chan_ind = find(obj.used_chan == obj.differential_chan);
                                
                cur = 0;
                for k = 1:length(obj.extractors)
                    temp = obj.extractors{k}.extract_features(recent_neural);
                    
                    n_chan = length(obj.used_chan);
                    n_feat_per_chan = length(temp)/n_chan;
                    
                    % keep track of feature values extracted from the
                    % "reference/differential channel"
                    temp_fvals = temp((0:n_chan:(n_feat_per_chan-1)*n_chan) + diff_chan_ind);
                    
                    for i = 1:n_feat_per_chan
                        temp((1:n_chan) + (i-1)*n_chan) = temp((1:n_chan) + (i-1)*n_chan) - temp(diff_chan_ind + (i-1)*n_chan);
                    end
                    
%                     % after previous for loop, reassign feature values
%                     % coming from the differential channel
%                     temp((0:n_chan:(n_feat_per_chan-1)*n_chan) + diff_chan_ind) = temp_fvals;

                    % pass in random noise for the features coming from the
                    % differential channel
                    temp((0:n_chan:(n_feat_per_chan-1)*n_chan) + diff_chan_ind) = randn(size(temp_fvals));
                    
%                     temp((0:n_chan:(n_feat_per_chan-1)*n_chan) + diff_chan_ind) = [];
                    
                    features( (1 : length(temp)) + cur ) = temp;
                    cur = cur + length(temp);
                end                
            else
                features = zeros(obj.n_features,1);
                cur = 0;
                for k = 1:length(obj.extractors)
                    temp = obj.extractors{k}.extract_features(recent_neural);
                    features( (1 : length(temp)) + cur ) = temp;
                    cur = cur + length(temp);
                end
            end
%             if cur ~= obj.n_features
%                 error('Incorrect number of features')
%             end
            
        end
        
        function update_chnfeat_index(obj)
            % ensures that the channel feature index is consistent
            inds_all = [];
            for k = 1 : length(obj.extractors)                
                if strcmp(class(obj.extractors{k}),'pow_extractor')                    
                    % is power, so check used_chan_array instead
                    inds = cell2mat(obj.extractors{k}.used_chan_array);
                else
                    % everything else
                    inds = obj.used_chan(:);
                end
                inds_all = [inds_all ; inds];
            end
            obj.n_features = length(inds_all);
            obj.chnfeat_index = inds_all;
        end
                 
        function remove_channels(obj,bad_chan)
            % remove bad channels from each feature extractor if they're using it
            for k = 1:length(obj.extractors)
                obj.extractors{k}.remove_channels(bad_chan)
            end            
            [c ia ib] = setxor(obj.used_chan,bad_chan);
            obj.used_chan = obj.used_chan(ia);   
            
            % update chnfeat_index            
            obj.update_chnfeat_index();
            
        end                                    
    end
end
            