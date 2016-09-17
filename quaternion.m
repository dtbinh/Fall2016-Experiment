function angles = quaternion( q )
%

qx = q(1);
qy = q(2);
qz = q(3);
qw = q(4);

yaw = atan2(2*qy*qw-2*qx*qz,1-2*qy^2-2*qz^2);
pitch = asin(2*qx*qy+2*qz*qw);
roll = atan2(2*qx*qw-2*qy*qz,1-2*qx^2-2*qz^2);
angles=[pitch;yaw;-roll];
end
