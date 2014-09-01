classdef task_interface < handle
    properties
        connection  % the connection object
    end
    
    methods
        function obj = task_interface(ip)
            % initiate connection to python
            obj.connection = udp(ip,'LocalPort',9091,'RemotePort',9090);
            fopen(obj.connection);
        end
        
        function send(obj,msg)
            % send a (string) message to python
            fprintf(obj.connection,msg);
        end
        
        function sendPosXY(obj,kn)
            % convert position to string
            msg = sprintf('%d %d',kn(1),kn(2));
            obj.send(msg);
        end
            
        function sendGo(obj)
            % start signal to initiate task
            fprintf(obj.connection,'1');
        end
        
        function events = getEvents(obj)
            % get event codes from python
            events = [];
            while( get(obj.connection,'BytesAvailable') > 0 )
                events(end+1) = str2num( fscanf(obj.connection) );
            end
        end

        function close(obj)
            % close connection
            fclose(obj.connection);
        end                
    end
    
end