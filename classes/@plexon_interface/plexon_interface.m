classdef plexon_interface < handle
    properties
        % kinematic params
        adc_gain;
        kin_scale_mat;
        kin_offset_mat;
        L1;
        L2;
        L2ptr;
        sho_x;
        sho_y;
        
        % force params
        voltOffset;
        voltToNewton;
        gain;
        
        % neural data
        neural;
        channels;
        
        kin_channels;  % kinematic analog data
        force;  % force data
        connection;  % connection
        
        spike_ts_buffer;
        event_ts_buffer;
    end    
    
    methods
        function obj = plexon_interface()
%             addpath('C:\Users\Kelvin\Documents\Dropbox\SparseLFP\bmi\MatlabClientDevelopKit\ClientSDK')
            
            % initialize plexon connection
            obj.connection = PL_InitClient;
            
            % load kinematic parameters
            load('testcalib012813.mat'); %load('KinematicsParametersJeev2.mat');
            obj.adc_gain = 5/2048;
            angle_gain = 2.5;
            vel_gain = 0.5;
            acc_gain = 0.01;
            offset_angle = 4./angle_gain;
            obj.kin_scale_mat = [angle_gain angle_gain vel_gain vel_gain acc_gain acc_gain];
            obj.kin_offset_mat = [offset_angle offset_angle 0 0 0 0];    
            obj.L1 = L1;
            obj.L2 = L2;
            obj.L2ptr = L2ptr;
            obj.sho_x = sho_x;
            obj.sho_y = sho_y;
            
            % load force parameters
            obj.gain = 4;
            obj.voltOffset = 0.5;
            obj.voltToNewton = 10/4.5;
            
            % channels
            obj.channels = 9:136;

            obj.spike_ts_buffer = ringbuffer(4,2000);
            obj.event_ts_buffer = ringbuffer(4,100);
        end
        
        function [n_ts, ts] = getTS(obj)
            [n_ts, ts] = PL_GetTS(obj.connection);

            spike_ts = ts(ts(:,1)==1, :);
            event_ts = ts(ts(:,1)==4, :);

            if size(spike_ts,1) > (0.5 * obj.spike_ts_buffer.size())
                fprintf('Warning: %d spike ts inserted into buffer of size %d', ...
                        size(spike_ts,1), obj.spike_ts_buffer.size());
            end

            if size(event_ts,1) > (0.5 * obj.event_ts_buffer.size())
                fprintf('Warning: %d event ts inserted into buffer of size %d', ...
                        size(event_ts,1), obj.event_ts_buffer.size());
            end

            obj.spike_ts_buffer.insert(spike_ts');
            obj.event_ts_buffer.insert(event_ts');
        end

        function spike_counts = getSpikeCounts(obj, desired_unit_nums)
            ts = obj.spike_ts_buffer.read(obj.spike_ts_buffer.get_num_new_pts())';

            chan_nums = ts(ts(:,1)==1, 2);
            unit_nums = ts(ts(:,1)==1, 3);

            % get rid of unsorted units (unit_nums == 0)
            chan_nums = chan_nums(unit_nums~=0);
            unit_nums = unit_nums(unit_nums~=0);
            inds = (chan_nums-1)*4 + unit_nums;

            % count up how many events for each unit that spiked
            spike_count_bin_edges = sort([desired_unit_nums; desired_unit_nums + 0.5]);
            if length(inds) > 0
                Y = histc(inds, spike_count_bin_edges);
                spike_counts = Y(1:2:end);
            else
                spike_counts = zeros(length(desired_unit_nums),1);
            end
        end

        function events = getEvents(obj)
            ts = obj.event_ts_buffer.read(obj.event_ts_buffer.get_num_new_pts())';
            events = ts(ts(:,1)==4, 3);
        end

        function [n, t, ad] = getAD(obj)
            % gets analog data
            [n, t, ad] = PL_GetAD(obj.connection);
            obj.neural = ad(:,obj.channels)';
            obj.kin_channels = ad(:,1:6);
            
            % assumes force channel is channel 8
            obj.force = ad(:,8);
        end

        function neural = getNeural(obj)
            neural = obj.neural;
        end
        
        function kn = getKinematics(obj)
            % returns the state [px,py,vx,vy] from plexon ad channel
            kn_ad = obj.kin_channels;
            kn_ad(kn_ad(:,1)==0,:) = [];          % remove missing AD values
            kn_v = mean(kn_ad) * obj.adc_gain;  % in volts            
            kn_rad = kn_v ./ obj.kin_scale_mat + obj.kin_offset_mat;    % in radians
            kn_pos = joint2endpt(kn_rad,obj.L1,obj.L2,obj.L2ptr,obj.sho_x,obj.sho_y);   % in meters
            
            % rescale to normalized space
%             origin = [0.0375 0.13];
            origin = [0.0378  0.1333]; %in meters
            kn_pos(1:2) = (kn_pos(1:2) - origin) / 0.08; %kn_pos is in meters
            kn_pos(3:4) = kn_pos(3:4) / 0.08;
            kn = kn_pos(1:4);
        end
        
        function fc = getForce(obj)
            % returns the force value in Newtons * gain
            fc_ad = obj.force;
            fc_ad(fc_ad(:,1)<=0,:) = 0;        % remove missing AD values
            fc = ( mean(fc_ad)*obj.adc_gain - obj.voltOffset ) * obj.voltToNewton * obj.gain;                                       
            
            if fc < 0
                fc = 0;
            end
            
        end
        
        function clearBuffer(obj)
            PL_GetAD(obj.connection);
            PL_GetTS(obj.connection);
        end
        
        function close(obj)
            % close the connection
            PL_Close(obj.connection);
        end        
    end
end