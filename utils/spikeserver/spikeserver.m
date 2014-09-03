function [] = spikeserver(port,testing)
addpath('tcp_udp_ip');
addpath('ClientSDK_leq2007');
port_pause=0.100;
if testing
    testind = 0;
    flagtest=0;
    for j = 1:20
        testdata{j}=[];
    end
end
%totalSpikesSent=0; for debugging, compare with totalSpikesReceived in
%spikeclient.m

% test_v = zeros(1000,1);
% testmat = zeros(1000,length(chan_inds));
% for i =1:length(chan_inds)
%     testmat(:,i) = test_v+chan_inds(i);
% end
%

timeout = 2;

pnet('closeall');

plx = PL_InitClient(0);
packetnum=1;
if plx == 0
    error('Could not connect to Plexon server');
else
    %Listen for an incoming connection on port #
    sock=pnet('udpsocket',port);

    if sock == -1
        error('Port %d is blocked',port);
    end

    %Only wait for 100 ms before giving up
    pnet(sock,'setreadtimeout',.002);
    pnet(sock,'setwritetimeout',1);

    disp('Waiting for client requests');

    clientisconnected = 0;
    connecttime = clock + 1;

    while 1
        t_it = tic;
        msglen = pnet(sock,'readpacket');
        %Received a message
        if msglen > 0
            %Read instruction
            instruction = pnet(sock,'readline');
            fprintf('Received message from client... %s\n',instruction);
            switch instruction
                case 'MARCO'
                    if clientisconnected
                        disp('Client was already previously connected');
                    end
                    clear wrapper;
                    plx = PL_InitClient(0);
                    %Flush buffer
                    [ndatapoints, ts, junk2] = PL_GetAD(plx);

                    %Handshake request
                    %Read IP
                    clientip = pnet(sock,'readline');
                    clientport = pnet(sock,'read',[1,1],'uint16');
                    n_chan = pnet(sock,'read',[1,1],'double');
                    chan_inds = pnet(sock,'read',[1,n_chan],'double');
                    include_kin = pnet(sock,'read',[1,1],'double');

                    if include_kin
                        chan_inds = [1:6 chan_inds];
                    end
                    %Get the current time from the Plexon server
                    %Currently hacky: wait for a message, then set
                    %currenttime to the timestamp within this
                    %message plus the polling interval for the Plexon
                    %server
                    PL_WaitForServer(plx,100);
                    [ndatapoints, ts, junk2] = PL_GetAD(plx); %2009a compatibility issue
                    allpars = PL_GetPars(plx);
                    currenttime = ts + ndatapoints/allpars(14);


                    %connect to client and send a payload containing
                    %the current time on the server
                    pnet(sock,'printf',['POLO' char(10)]);
                    pnet(sock,'write',currenttime);
                    %pnet(sock,'printf','Random garbagey garbage');
                    pnet(sock,'writepacket',clientip,double(clientport));

                    fprintf('Client %s:%d connected\n',clientip,clientport);

                    clientisconnected = 1;

                    connecttime = clock;

                    %Clear the spike buffer
                    [nspks,ts] = PL_GetTS(plx);
                case 'KEEPALIVE'
                    connecttime = clock;
                    flagtest=1;
                case 'DISCONNECT'
                    clientisconnected = 0;
                    connecttime = clock + 1;

                    %close port and reopen
                    pnet(sock,'close');
                    sock=pnet('udpsocket',port);
                    while sock==0
                        sock=pnet('udpsocket',port);
                    end

            end

        end

        %Don't send spikes for no reason
        if etime(clock,connecttime) > timeout
            disp('client disconnected');
            clientisconnected = 0;
            connecttime = clock + 1;

            %close port and reopen
            %                 pnet(sock,'close');
            %                 sock=pnet('udpsocket',port)
            %                 while sock==0
            %                 	sock=pnet('udpsocket',port)
            %                 end
        end

        maxpacketsize = 1e3;

        if clientisconnected
            %Send spikes
            %Stream data
            %PL_WaitForServer(plx);

            if exist('tread','var') 
                pause(max([port_pause - toc(tread),0]))
            else
                pause(port_pause)
            end
            
            
            [nad, tad, AD] = PL_GetAD(plx);
            [n_ts, ts] = PL_GetTS(plx);
            tread=tic;
           
            event_ind = ts(:,1)==4;
            events = ts(event_ind,[3:4]);
            if ~isempty(events)
                send_events = size(events,1);
                pnet(sock,'printf',['EVENTS' char(10)]);
                pnet(sock,'write',send_events);
                pnet(sock,'write',events);
                pnet(sock,'writepacket',clientip,clientport);
            end
            
                        
            if nad > 0
                %fprintf('Sending %d LFP pts \n', nad);
                 rg = 1:min(size(AD,1),maxpacketsize);
                pnet(sock,'printf',['SPIKES' char(10)]);
                pnet(sock,'write',packetnum);
                pnet(sock,'write',nad);
                %                         pnet(sock,'write',nev);
                %                         pnet(sock,'write',events);
                if rg>0
                    
                    packetnum=packetnum+1;
                    
                    for chan = 1:length(chan_inds)
                        pnet(sock,'write',AD(rg, chan_inds(chan)));
                        %pnet(sock,'write',testmat(1:10, :));
                        if testing
                            if testind<100 & flagtest==1
                                testdata{chan}=[testdata{chan} AD(rg,chan_inds(chan))'];
                                length(rg)
                            elseif testind==100
                                save('testdat','testdata')
                            end
                        end
                    end
                    if testing
                        if flagtest==1
                            testind=testind+1;
                        end
                    end
                    
                    %Send events
                    pnet(sock,'writepacket',clientip,clientport);
                    %ad_test = [ad_test;AD(rg,chan_inds(chan-1:chan))]; %Test with last 2 channels
                    %end

                end
            end
%             tm = toc(t_it)
        end
    end
end
end
