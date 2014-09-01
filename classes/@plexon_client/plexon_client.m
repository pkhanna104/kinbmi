classdef plexon_client < handle
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
        
        % neural data
        neural;
        channels;
        n_chan;

        kin_channels;  % kinematic analog data
        n_kin_chan;
        n_tot_chan; 
        
        %Connection params: 
        selfip = '192.168.0.8';
        remoteip = '192.168.0.6';
        selfport = 6000;
        remoteport = 6000;
        
        %Timing params:
        initt0;
        dbefore;
        remotet0;
        localt0; 
        
        %Socket 
        sock;
        
        %Flags
        polo_recv;
                
        %Data buffers;
        neural_new;
        kin_new;
        events_new;

    end
    
    methods
        function obj = plexon_client()
            
            obj.selfip = '192.168.0.8';
            obj.remoteip = '192.168.0.6';
            obj.selfport = 6000;
            obj.remoteport = 6000;            
            
            % connect to plexon server using spikeclient formula
            pnet('closeall');
            obj.initt0 = tic;
            
            %To reconnect, start here:
            obj.sock=pnet('udpsocket',obj.selfport);
            if obj.sock == -1
                error('Could not open port %d',obj.selfport);
            end
            pnet(obj.sock,'setwritetimeout',1);
            pnet(obj.sock,'setreadtimeout',1);
                        
            obj.sock         
            %kin params: 
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
            
            %initialize channels:
            obj.channels = [];
                        
        end
        
        function add_chan(obj, ad_channels,replace_all)
            %obj.channels = ad_channels - 64 + 8 ; %AD 65 --> chan 9 %pow extractor already does this!
            
            if strcmp(replace_all,'replace_all')
                obj.channels = ad_channels;
            else
                obj.channels = [obj.channels ad_channels]; 
            end
            
            obj.n_chan = length(obj.channels);
            
            obj.kin_channels = 1:6;
            obj.n_kin_chan = 6;
            
            obj.n_tot_chan = obj.n_chan + obj.n_kin_chan;
            
        end
        
        function send_marco(obj)
            %Send request
            disp('Connecting to server');

            pnet(obj.sock,'printf',['MARCO' char(10) obj.selfip char(10)]);
            pnet(obj.sock,'write',uint16(obj.selfport));
            pnet(obj.sock,'write',length(obj.channels));
            pnet(obj.sock,'write',obj.channels);
            pnet(obj.sock,'write',[1]); %Include kinematics
            pnet(obj.sock,'writepacket',obj.remoteip,obj.remoteport);
            obj.dbefore = 0;
        end
        
        function get_polo(obj,wait_time)
            obj.polo_recv = 0;
            t1 = tic;
            
            while obj.polo_recv == 0
                disp(num2str(toc(t1)))
                if toc(t1)<wait_time
                    sz = pnet(obj.sock,'readpacket');
                    if sz > 0
                        msg = pnet(obj.sock,'readline');
                        if strcmp(msg,'POLO')
                             %Received acknowledgement, sync times
                            disp('Connected to server');
                            obj.remotet0 = pnet(obj.sock,'read',[1,1],'double');
                            obj.localt0 = toc(obj.initt0);
                            obj.polo_recv = 1;
                        end
                    end
                else
                    obj.polo_recv = -1;
                end
            end
        end
        
        function nad = get_data(obj)
            nad = 0;         
            if obj.polo_recv ~= 1
                disp('Can''t get messages yet -- not connected!')
            else
                sze = pnet(obj.sock,'readpacket');
                if sze > 0
                    %disp(num2str(sze))
                    msg = pnet(obj.sock,'readline');
                    if strcmp(msg,'SPIKES')
                        
                        %Read data!
                        %Packet num: 
                        pnet(obj.sock, 'read', [1,1],'double');
                        nad = pnet(obj.sock,'read',[1,1],'double');
                                                
                        %sze = sze - 7 - 8 - 8 - 8 - (nev*8);
                        sze = sze - 7 - 8 - 8;
                        npts = sze/(8*obj.n_tot_chan);
                                                
                        if npts ~= 0 %Data in this one
                            dat = zeros(npts,obj.n_tot_chan);
                            for c = 1:obj.n_tot_chan
                                try
                                    dat(:,c) = pnet(obj.sock,'read',[npts,1],'double');
                                catch
                                     disp('Corrupt message received');
                                     return
                                end
                            end
                            %obj.events_new = events;
                            obj.neural_new = dat(:,obj.n_kin_chan+1:end);
                            obj.kin_new = dat(:,1:obj.n_kin_chan);
                            
                        end
                                            
                    elseif strcmp(msg,'EVENTS')
                        nev = pnet(obj.sock,'read',[1,1],'double');
                        events = pnet(obj.sock,'read',[nev,2],'double');
                        obj.events_new = events;
                        
                    else
                        fprintf('Invalid message type received: %s\n',msg);
                        return;
                        
                    end

                    dnow = toc(obj.initt0);
                    if mod(obj.dbefore,2) > mod(dnow,2)
                        %Send keep alive signal every 10 seconds
                        pnet(obj.sock,'printf',['KEEPALIVE' char(10) obj.selfip char(10)]);
                        pnet(obj.sock,'write',int16(obj.selfport));
                        pnet(obj.sock,'writepacket',obj.remoteip,obj.remoteport);
                    end
                    
                    obj.dbefore = dnow;
                 else
                    %timed out... either no spikes for a second or some network issue
                    %Try reconnecting
                    disp('Timeout receiving spikes');
                    return;
                end
            end
        end
        
        function neural = get_neural(obj)
            neural = obj.neural_new'; %channels x time
        end
        
        function kn = get_kin(obj)
             % returns the state [px,py,vx,vy] from plexon ad channel
            kn_ad = obj.kin_new;
            if ~isempty(kn_ad)
                kn_ad(kn_ad(:,1)==0,:) = []; % remove missing AD values
                kn_v = mean(kn_ad) * obj.adc_gain;  % in volts   

                kn_rad = kn_v ./ obj.kin_scale_mat + obj.kin_offset_mat;    % in radians
                kn_pos = joint2endpt(kn_rad,obj.L1,obj.L2,obj.L2ptr,obj.sho_x,obj.sho_y);   % in meters

                % rescale to normalized space
                % origin = [0.0375 0.13];
                origin = [0.0378  0.1333]; %in meters
                kn_pos(1:2) = (kn_pos(1:2) - origin) / 0.08; %kn_pos is in meters
                kn_pos(3:4) = kn_pos(3:4) / 0.08;
                kn = kn_pos(1:4);
            else
                kn = [0,0,0,0];
            end
        end
        
        function event = get_events(obj)
            event = obj.events_new; %N_events x 2 ([ eventcode, ts])
        end
        
        function close_sock(obj)
            pnet(obj.sock,'close')
        end
        
        function keep_conn_alive(obj)
            pnet(obj.sock,'printf',['KEEPALIVE' char(10) obj.selfip char(10)]);
            pnet(obj.sock,'write',int16(obj.selfport));
            pnet(obj.sock,'writepacket',obj.remoteip,obj.remoteport);
        end
        
        function clear_buffer(obj)
            disp('clear?')
        end
    end
end
                        
                        