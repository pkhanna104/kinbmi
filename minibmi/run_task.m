function run_task(handles)

handles.t =0;

for t=1:600
    
    %Get neural data
    D = handles.Nexus.inst.getDataPacket;
    Data = D.getData;
    
    y = mean(Data);
    
    %Calculate Stuff
    
    %Update Display
    
    %Save Data
    
    %How long has it been? Wait? 
    
end