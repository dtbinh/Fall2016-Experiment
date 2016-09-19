% Number of Agent in the Experiment
AgentNumber = 4;
% Bluetooth Buffer Size
btBuffer = 512;
% Pause time during calibtration
calPause = 0.25;
% Number for calibration
calibNum = 50;
% Init values of sensor during calibration
iniValue = 850;
% Bluetooth devices
btPort(1,:) = 'RNBT-6AD1';
btPort(2,:) = 'RNBT-6B35';
btPort(3,:) = 'RNBT-8B44';
btPort(4,:) = 'RNBT-6AB7';
% IP Address of Agent
agentAddr1 = '192.168.1.50';
agentAddr2 = '192.168.1.55';
agentAddr3 = '192.168.1.60';
agentAddr4 = '192.168.1.121';
% Initial Movement when calculate the orientation
iniMovement = 50;
% Initial Speed for Khepera Agent
iniSpeed = 50;
% Pause to let the robot move
movPause = 0.1;
% Length of Data in DataSet for a robot. 
% length of DataSet should be AgentNumber * dataLen
% location of agent: 						x , y , rot
% reading of sensor and filtered reading: 	value , fValue
% target position of agent: 				xTar , yTar
% velocity and turing angle for agent: 		velocity , angle
% wheel speed: 								lspeed , rspeed
% flag for single agent: 					flg
dataLen = 12;%[x,y,rot,value,fValue,xTar,yTar,velocity,angle,lspeed,rspeed,flg]
% Factor for filter to deal with the raw data from sensor
ffactor = [ 0.1 , 0.3 , 0.6 ];
% Initial Background data for filtered reading
iniBG = 0;
% Length for filterOut
% filterOut should incluede(7): 
% target center of four agent:  			cTarx , cTary
% center concentration estimation error: 	error
% parameter estimation: 					est
% gradient for motion: 						gradx , grady
% laplacian: 								laplacian
% filterOut = [ cTarx , cTary , gradx , grady , error , est , laplacian ];
filOLen = 7;
% Initial Estimation Value 
iniEst = 0.8;
% dt to estimate error
dt = 0.05;
% Coefficient for center movement after calculate the gradient.
% Also, speed for center.
gradCoe = 2;
% Threshold for parameter estimation
estThreshold = 0.05;
% Number for Loop
loop = 40;
% Lenght for agent
agentLen = 0.15;
% Multiplier for velicity and angle
vmulti = 10;% velocity
wmulti = 10;% angle