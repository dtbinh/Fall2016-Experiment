inst2Khepera = 'S0000S0000';
for agent = 1 : AgentNumber
	kheperaOutput( agent ).writeBytes( inst2Khepera );
end
for agent = 1 : AgentNumber
	fclose( mbed(agent) );
end
clear functions;