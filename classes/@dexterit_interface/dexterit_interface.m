classdef dexterit_interface < handle
    properties
        connection  % the connection object
    end
    
    methods
        function obj = dexterit_interface(ip)           
            obj.connection = udp(ip,'RemotePort', 22301);    
            fopen(obj.connection);            
        end
        
        function sendPosVelXY(obj,kn)
            % rescale the kinematics back to dexterit coords
%             origin = [0.04 ; 0.14];
%             origin = [0.0375 ; 0.1325];
            origin = [0.0378 ; 0.1333];
            scale = 0.08;
            
            kn(1:2) = kn(1:2)*scale + origin;
            kn(3:4) = kn(3:4)*scale;            
%             disp(kn)
            fwrite( obj.connection, typecast([kn(:)' 0], 'uint8') );     
        end        
        
        function sendGo(obj)
            % unnecessary for this interface
        end

        function close(obj)
            % close connection
            fclose(obj.connection);
        end                
    end
    
end        