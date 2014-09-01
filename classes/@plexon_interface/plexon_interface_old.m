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
        
        % neural data
        neural;
        channels;
        
        % kinematic analog data
        kin_channels;
        
        % connection
        connection;
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
            
            % channels
%             obj.channels = 10:137;
            obj.channels = 9:136;
        end
        
        function [n, t, ad] = getAD(obj)
            % gets analog data
            [n, t, ad] = PL_GetAD(obj.connection);
            obj.neural = ad(:,obj.channels)';
            obj.kin_channels = ad(:,1:6);
        end
                
        function neural = getNeural(obj)
            % returns neural data
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
            origin = [0.0375 0.13];            
            kn_pos(1:2) = (kn_pos(1:2) - origin) / 0.08;
            kn_pos(3:4) = kn_pos(3:4) / 0.08;
            kn = kn_pos(1:4);
        end
            
        function events = getEvents(obj)
            % returns event code(s)
            [numtimestamps timestamps] = PL_GetTS(obj.connection);   % get events
            events = timestamps( timestamps(:,1)==4, 3);
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