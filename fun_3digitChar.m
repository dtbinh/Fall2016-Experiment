function [ output ] = fun_3digitChar( input )

if abs(input) < 10
    output = ['00',int2str(abs(input))];
elseif abs(input) < 100
    output = ['0',int2str(abs(input))];
else
    output = int2str(abs(input));
end
% sign
if input < 0 
    isign = '-';
else
    isign = '+';
end
output = [isign,output];
end

