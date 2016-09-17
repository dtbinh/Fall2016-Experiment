% Frame

%% Init Data Frame
DataSet( counter , dataLen*(agent-1)+1 : dataLen*agent ) = ...
	zeros( 1 ,  AgentNumber * dataLen );


	
	%% Location of agent
for agent = 1 : AgentNumber
	DataSet( counter , dataLen*(agent-1)+1 : dataLen*(agent-1)+3 ) = ...
		fun_trackInterface( theClient , agent );% Location
end



%% Reading of Sensor
for agent = 1 : AgentNumber
	DataSet( counter , dataLen*(agent-1)+4 ) = ...
		fun_SensorReader( mbed(agent) , DataSet( counter-1 , dataLen*(agent-1)+4 ) );
end



%%  Simple Filter for Reading
for agent = 1 : AgentNumber
	DataSet( counter , dataLen*(agent-1)+5 ) = ...
		ffactor(1) * DataSet( counter-2 , dataLen*(agent-1)+4 ) + ...
		ffactor(2) * DataSet( counter-1 , dataLen*(agent-1)+4 ) + ...
		ffactor(3) * DataSet( counter-0 , dataLen*(agent-1)+4 ) - ...
		sensorBG(agent) + iniBG;
	% prepare to show
	values( agent ) = DataSet( counter , dataLen*(agent-1)+5 );
end



%% Show Values
display('Sensor Reading:')
values
clear values



%% Run Algorithm
% Input DataSet / s
% Output filterOut / s
% prepare data
for agent = 1 : AgentNumber
	% sensor reading for this frame
	values( 1 , agent ) = DataSet( counter-0 , dataLen*(agent-1)+5 );
	% sensor reading for last frame
	values( 2 , agent ) = DataSet( counter-1 , dataLen*(agent-1)+5 );
	% location of agent this frame
	r( : , agent ) = DataSet( counter , dataLen*(agent-1)+1 : dataLen*(agent-1)+2 )';
	% location of agent last frame
	rk( : , agent ) = DataSet( counter-1 , dataLen*(agent-1)+1 : dataLen*(agent-1)+2 )';
end
rc = mean( r' )';
rck = mean( rk' )';
% Estimate Laplacian ( Last Frame )
l = ( 1 / ( (r(2,1)+r(1,2)-r(2,3)-r(1,4))/4 )^2 ) *...
    ( ( sum( Values(2,:) ) ) - AgentNumber * s.x( 5 , 1 ) );
% Call cooperative Kalman filter
s = fun_kalmanf3...
	( s , values , r(:,1) , r(:,2) , r(:,3) , r(:,4) , ...
	rc , rk(:,1) , rk(:,2) , rk(:,3) , rk(:,4) , rck , l );
% Estimate Laplacian Again ( This Frame )
l = ( 1 / ( (r(2,1)+r(1,2)-r(2,3)-r(1,4))/4 )^2 ) *...
    ( ( sum( Values(1,:) ) ) - AgentNumber * s.x( 5 , 1 ) );
% Estimate Error in the Center of Agent Group
error = s.x(1,1) - s.x(5,1);
errort = error / dt;
% Center Gradient
grad = [ s.x(2,1) , s.x(3,1) ] / norm([ s.x(2,1) , s.x(3,1) ]);
% Next Center
rc = rc + gradCoe * dt*grad;
% Collect Data to DataSet
filterOut( counter , : ) = zeros( 1 , filOLen );
filterOut( counter , 1 ) = rc(1);
filterOut( counter , 2 ) = rc(2);
filterOut( counter , 3 ) = grad(1);
filterOut( counter , 4 ) = grad(2);
filterOut( counter , 5 ) = error;
filterOut( counter , 7 ) = l;
clear rc rck r rk  values grad error l
	

	
%% Formation Control for Khepera Robot
% Input filterOut
% Output DataSet
[r1,r2,r3,r4] = fun_jacobi( iniDist(1,:)' , iniDist(2,:)' , ...
	iniDist(3,:)' , iniDist(4,:)' , filterOut( counter , 1:2 )' );
DataSet( counter , dataLen*(1-1)+6:dataLen*(1-1)+7 ) = r1';
DataSet( counter , dataLen*(2-1)+6:dataLen*(2-1)+7 ) = r2';
DataSet( counter , dataLen*(3-1)+6:dataLen*(3-1)+7 ) = r3';
DataSet( counter , dataLen*(4-1)+6:dataLen*(4-1)+7 ) = r4';
clear r1 r2 r3 r4



%% Parameter Estimation
r = fun_RLS1 (r,errort,filterOut(counter,7));
est = r.x;
if est < estThreshold
	est = estThreshold;
end
s.x(4,1) = est;
filterOut( counter , 6 ) = est;
clear est



%% Generate and Send Instruction to Khepera
%....



%% Draw Figure



%% Pause
pause(movPause);