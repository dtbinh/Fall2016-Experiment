% Main.m

%% Global Parameter
Scr_para;

%% Initialize System
Scr_init;

%% Loop
while(1)
	counter = counter + 1;
	Scr_frame;
	Scr_trig;
end

%% Clean Up
Scr_Clean;