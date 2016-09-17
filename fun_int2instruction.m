function [ instruction ] = fun_int2instruction( lspeed , rspeed )

% check threshold
threshold = 500;
if abs(lspeed) > threshold
    lspeed = threshold * sign(lspeed);
end
if abs(rspeed) > threshold
    rspeed = threshold * sign(rspeed);
end
% double to int first
lspeed = int16(lspeed);
rspeed = int16(rspeed);
% to string
lstr = fun_3digitChar(lspeed);
rstr = fun_3digitChar(rspeed);
% assemble
instruction = [ 'L' , lstr , 'R' , rstr ];

end

