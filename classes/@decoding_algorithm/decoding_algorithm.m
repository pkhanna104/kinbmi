classdef decoding_algorithm < handle

    properties
        dt            % iteration dt at which decoder is run
        n_kin_vars    % # of kinematic variables that decoder predicts
        n_features    % # of features passed into the decoder each iteration
        CL_flag = 1   % 1 if decoder is being used for closed-loop control
        clda_cnt = 0  % # of times CLDA has been performed on this decoder
        dec_perf      % decoder performance (r-value)
    end

%     methods(Abstract)
%         predict_kin(obj,f_buffer)  % returns predicted kinematics
%         get_dec_params(obj)        % returns struct of decoder parameters
%         get_description(obj)       % returns string description of the decoder
%         set_CL_mode(obj,flag)      % set the value of CL_flag
%     end

    methods
        function [PRED_KIN,r_vals] = run(obj,Y,TRUE_KIN)
            N = size(Y,2);

            % create ringbuffer for storing features
            f_buffer_t = 1;  % in secs
            f_buffer_size = round(f_buffer_t/obj.dt);
            f_buffer = ringbuffer(obj.n_features,f_buffer_size);

            PRED_KIN = zeros(4,N);
            for i = 1:N
                f_buffer.insert(Y(:,i));
                PRED_KIN(:,i) = obj.predict_kin(f_buffer);
            end

            r_vals = zeros(obj.n_kin_vars,1);
            if nargin == 3  % only do if TRUE_KIN was passed
                for k = 1:obj.n_kin_vars
                    R = corrcoef(TRUE_KIN(k,:),PRED_KIN(k,:));
                    r_vals(k) = R(2,1);
                end
            end
        end
    end
   
end
