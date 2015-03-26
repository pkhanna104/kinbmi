function handles = init_Nexus(handles)

inst = mdt.neuro.nexus.NexusInstrument.getInstance; 

% Find current serial ports:
ser = instrfind;
for i = 1:length(ser)
	print ser{i}.name
end

%Start Serial Connection
prompt = 'Select Index of Serial Connection: ';
i = input(prompt);

s = mdt.neuro.nexus.SerialConnection(ser{i}.name);
inst.connect(s);

% Get Status: 
status = inst.getNexusStatus; 

% Ensure status is good: 
if status.getState == 0
	disp('INS not connected')
end

%Set Timeouts High
inst.setNexusConfiguration(30,15); 

%Start Sensing:
inst.startSensing;
inst.startDataSession;

%Attach to handles:
handles.Nexus.serial_connection = s;
handles.Nexus.inst = inst;
