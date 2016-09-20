% Scr_init

%% Init NatNet Client
% Output: 
% Struct of Client:				theClient
display('Initialize Tracking System...');
dllPath = fullfile('c:','NatNet_SDK_2.6','lib','x64','NatNetML.dll');
assemblyInfo = NET.addAssembly(dllPath);
theClient = NatNetML.NatNetClientML(0);
hst = java.net.InetAddress.getLocalHost;
HostIP = char(hst.getHostAddress);
flg = theClient.Initialize(HostIP, HostIP);
if (flg == 0)
    display('Initialization Succeeded');
else
    display('Initialization Failed');
end
display('Client Data:');
GetDataDescriptions(theClient)
clear dllPath assemblyInfo hst HostIP



%% Init Controller on Agent
% Input: 
% BluetoothID for Controller 	btPort(agent,:)
% Number of Agent				AgentNumber
% Bluetooth Buffer Size 		btBuffer;
% Output:
% Bluetooth devices file		mbed(agent)
display('Initialize Bluetooth Connection...');
for agent = 1 : AgentNumber
	% Find devices
	btInst(agent) = instrhwinfo('Bluetooth',btPort(agent,:));
	btChannel(agent) = str2double( btInst(agent).Channels );
	mbed(agent) = Bluetooth( btPort(agent,:) , btChannel(agent) );
	pause;%Pause to confirm..
	% Open devices
	mbed(agent).InputBufferSize = btBuffer;
	mbed(agent).ReadAsyncMode = 'Manual';
	fopen( mbed(agent) );
end
clear btInst btChannel btPort
display('Connection Status:');
mbed



%% Agent Calibration
% Input:
% Number of Agent				AgentNumber
% Bluetooth devices file		mbed(agent)
% Pause time during calibration	calPause
% Init values of sensor 		iniValue
% Output:
% Calibration data from sensor 	sensorCal( counter , agent )
% Backgroud info about sensor 	sensorBG( agent )
display('Calibrate CO2 Sensor...');
for cCounter = 1 : calibNum
	for agent = 1 : AgentNumber
		if cCounter == 1
			sensorCal( cCounter , agent ) = ...
				fun_SensorReader( mbed(agent) , iniValue );
		else
			sensorCal( cCounter , agent ) = ...
				fun_SensorReader( mbed(agent) , sensorCal( cCounter-1 , agent ) );
		end
	end
	pause( calPause );
end
sensorBG = mean( sensorCal );
display('Sensor Background:');
clear cCounter
sensorBG



%% Init Khepera IV
% Input:
% IP address for Khepera 		agentAddr1/agentAddr2/.../agentaddr4
% Output:
% Input Stream for Khepera		kheperaInput( agent )
% Output Stream for Khepera 	kheperaOutput( agent )
display('Connect Khepera Robot...');
import java.net.Socket
import java.io.*
kheperaPort = 344;
for agent = 1 : AgentNumber
	% switch the address of the khepera robot
	switch( agent )
        case 1
            ipAddress = agentAddr1;
        case 2
            ipAddress = agentAddr2;
        case 3
            ipAddress = agentAddr3;
        case 4
            ipAddress = agentAddr4;
        otherwise
            display('IP Address ERROR!');
    end
	% Connect the robot
	AgentSocket( agent ) = Socket( ipAddress , kheperaPort );
    kheperaInput( agent ) = DataInputStream( AgentSocket(agent).getInputStream );
    kheperaOutput( agent ) = DataOutputStream( AgentSocket(agent).getOutputStream );
end
clear ipAddress agentAddr1 agentAddr2 agentAddr3 agentAddr4 AgentSocket
display('Finish Connection.');



%% Measure Init Orientation
% Input: 
% Number of Agent 					AgentNumber
% NatNet Client 					theClient
% Initial Movement steps 			iniMovement
% Pause time for move 				movPause
% Initial speed for khepera robot  	iniSpeed
% Output:
% Initial Orientation of robot 		iniOri
display('Measure Initial Orientation of Agent.');
% Get first group of location
for agent = 1 : AgentNumber
    [ x , y , rot ] = ...
		fun_trackInterface( theClient , agent );
    kheperaLocation( agent , : ) = [ x , y , rot ];
end
% Prepare stop message as initial value
inst2Khepera = fun_int2instruction( iniSpeed , iniSpeed );
% Move the khepera robot
for mcounter = 1 : iniMovement
	for agent = 1 : AgentNumber
		kheperaOutput( agent ).writeBytes( inst2Khepera );
	end
	pause(movPause);
end
% Stop the Robot
inst2Khepera = fun_int2instruction( 0 , 0 );
for agent = 1 : AgentNumber
	kheperaOutput( agent ).writeBytes( inst2Khepera );
end
pause;
% Get second group of location
for agent = 1 : AgentNumber
	[ x , y , rot ] = ...
		fun_trackInterface( theClient , agent );
    kheperaLocation( agent+AgentNumber , : ) = [ x , y , rot ]; 
end
% Calculate Orientation
for agent = 1 : AgentNumber
	kheperaLocation( agent+2*AgentNumber , : ) = ...
		kheperaLocation( agent + AgentNumber , : ) - ...
		kheperaLocation( agent , : );
	% Orientation in x-y coordinate
	iniOri(agent,1) = atan2( ...
		kheperaLocation(agent+2*AgentNumber,1) ,...
		kheperaLocation(agent+2*AgentNumber,2) ) * 180/pi ;
	% Orientation measure by tracking system
	iniOri(agent,2) = kheperaLocation(agent+AgentNumber,3); 
	% iniOri from -180~180 degree...
end
display('Orientation of Agent:');
iniOri



%% Calculate the Formation of agents
% Input:
% Location of Khepera Robot 			kheperaLocation
% Output:
% Distance between agent and center 	initDist
display('Calculate Initial Formation.');
initCenter = mean( kheperaLocation( 1+AgentNumber:2*AgentNumber , 1:2 ) );
for agent = 1 : AgentNumber
	initDist( agent , : ) = kheperaLocation( agent+AgentNumber , 1:2 ) - initCenter;
end
clear kheperaLocation



%% Wait for Diffusion
display('Pause for diffusion...');
pause;
display('Pause twice...');
pause;


%% Collect Necessary Data for Algorithm / Init DataSet
% Output:
% Location and sensor values for agent 	DataSet( counter , dataLen )
% counter for loop 						counter(counter = 2)
% First group of data
counter = 1;
DataSet( counter , 1 : dataLen*agent ) = ...
	zeros( 1 ,  AgentNumber * dataLen );
for agent = 1 : AgentNumber
	% DataSet( counter , dataLen*(agent-1)+1 : dataLen*agent )
    [ x , y , rot ]= ...
		fun_trackInterface( theClient , agent );% Location
    DataSet( counter , dataLen*(agent-1)+1 : dataLen*(agent-1)+3 )  = [ x , y , rot ];
	DataSet( counter , dataLen*(agent-1)+4 ) = ...
		fun_SensorReader( mbed(agent) , iniValue );% Sensor reading
	DataSet( counter , dataLen*(agent-1)+5 ) = ...
		DataSet( counter , dataLen*(agent-1)+4 ) - sensorBG(agent) + iniBG;% No filter here
	DataSet( counter , dataLen*(agent-1)+6 : dataLen*(agent-1)+7 ) = ...
		DataSet( counter , dataLen*(agent-1)+1 : dataLen*(agent-1)+2 );% No Target
	DataSet( counter , dataLen*(agent-1)+8 : dataLen*(agent-1)+12 ) = ...
		[ 0 , 0 , 0 , 0 , 0 ];% l/r speed, flag
end
% Sencond group of data
counter = 2;
DataSet( counter , 1 : dataLen*agent ) = ...
	zeros( 1 ,  AgentNumber * dataLen );
for agent = 1 : AgentNumber
	% DataSet( counter , dataLen*(agent-1)+1 : dataLen*agent )
    [ x , y , rot ] = ...
		fun_trackInterface( theClient , agent );% Location
    DataSet( counter , dataLen*(agent-1)+1 : dataLen*(agent-1)+3 )  = [ x , y , rot ];
	DataSet( counter , dataLen*(agent-1)+4 ) = ...
		fun_SensorReader( mbed(agent) , DataSet( counter-1 , dataLen*(agent-1)+4 ) );% Sensor reading
	DataSet( counter , dataLen*(agent-1)+5 ) = ...
		DataSet( counter , dataLen*(agent-1)+4 ) - sensorBG(agent) + iniBG;% No filter here
	DataSet( counter , dataLen*(agent-1)+6 : dataLen*(agent-1)+7 ) = ...
		DataSet( counter , dataLen*(agent-1)+1 : dataLen*(agent-1)+2 );% No Target
	DataSet( counter , dataLen*(agent-1)+8 : dataLen*(agent-1)+12 ) = ...
		[ 0 , 0 , 0 , 0 , 0 ];% l/r speed, flag
end
clear x y rot



%% Init filterOut
% Output: filterOut(counter,data)
filterOut = zeros( 2 , filOLen );
for agent = 1 : AgentNumber
	meanx(agent) = DataSet( counter , dataLen*(agent-1)+1 );
	meany(agent) = DataSet( counter , dataLen*(agent-1)+2 );
end
cTarx = mean(meanx);
cTary = mean(meany);
filterOut(1,:) = [ cTarx , cTary , 0 , 0 , 0 , iniEst , 0 ];
filterOut(2,:) = [ cTarx , cTary , 0 , 0 , 0 , iniEst , 0 ];
clear cTarx cTary meanx meany


%% Init states(s,r) and kalState
% kalState usually to store data not to handle data
% in this program,I prefer use s instead...
% Output: 
% Record of state: 						kalState(counter).r/x
% state now: 							s
% RLS:									r
for agent = 1 : AgentNumber
	centerreading1(agent) = DataSet( 1 , dataLen*(agent-1)+5 );
	centerreading2(agent) = DataSet( 2 , dataLen*(agent-1)+5 ); 
end
R = mean( centerreading2 );
Rk = mean( centerreading1 );
px = DataSet( 2 , dataLen*(1-1)+5 ) - DataSet( 2 , dataLen*(3-1)+5 );%px = 1-3
py = DataSet( 2 , dataLen*(2-1)+5 ) - DataSet( 2 , dataLen*(4-1)+5 );%py = 2-4
clear centerreading1 centerreading2
s.x = [ R ; px ; py ; iniEst ; Rk ; px ; py ];
s.P = 0;
s.H=0.01*[1 1 1 1 1 1 1 1]';
clear R Rk px py
kalState(1).s = s;
kalState(2).s = s;
r.x = iniEst;
r.P = 4;
kalState(1).r = r;
kalState(2).r = r;



%% Init Algorithm
% there should be something, e.g. state for estimation