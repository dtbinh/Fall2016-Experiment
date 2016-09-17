function [ x , y , rot ] = fun_trackInterface( client , agent )
% Interface layer between tracking system and algorithm
% Input:
% agent number - agent
% natnet client - client
% Output:
% x , y , rot(in degree format...)
data = client.GetLastFrameOfData();
x = -data.RigidBodies(agent).x;
y = data.RigidBodies(agent).z;%!

q = [ 	data.RigidBodies(agent).qx, ...
		data.RigidBodies(agent).qy, ...
        data.RigidBodies(agent).qz, ...
		data.RigidBodies(agent).qw ];
angles = quaternion( q );
rot = -angles(2) * 180.0 / pi;% Assume this is correct.

end
