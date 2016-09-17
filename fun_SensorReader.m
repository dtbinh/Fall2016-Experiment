% function [ resultNum ] = fun_SensorReader( mbed, prev_sensorCal )
% %UNTITLED Summary of this function goes here
% %   Detailed explanation goes here
% flushPause = 0.05;
% counter = 1;
% flg = 0;
% counterMax = 10;
% 
% % flushinput(mbed);
% pause(flushPause);
% % if mbed.BytesAvailable >= 4
% %     fread(mbed, mbed.BytesAvailable);
% % end
% % while( mbed.BytesAvailable <= 16 )
% %     pause(flushPause);
% %     counter = counter + 1;
% %     if counter >= 10
% %         display('Sensor Error!\r\n');
% %         flg = 1;
% %         break;
% %     end
% % end
% feed = fgetl(mbed);
% if isempty(feed)
%     resultNum = prev_sensorCal;
% else
%     if flg == 1
%         feed = '0';
%     end
%     resultNum = str2num(feed);
% end
% % resultNum = feed;
% 
% end
% 
function [ resultNum ] = fun_SensorReader( mbed, prev_sensorCal )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
flushPause = 0.05;
counter = 1;
flg = 0;
counterMax = 10;

if mbed.BytesAvailable
    fread(mbed, mbed.BytesAvailable);
end
readasync(mbed,32);
% if mbed.BytesAvailable > 0
%     %fread(mbed, mbed.BytesAvailable);
% %     for i = 1 : 10
% %         if mbed.BytesAvailable <=2
% %             break;
% %         end
%         fread(mbed, mbed.BytesAvailable-1);
% %     end
% end
%pause(flushPause);
% while( mbed.BytesAvailable <= 16 )
%     pause(flushPause);
%     counter = counter + 1;
%     if counter >= 10
%         display('Sensor Error!\r\n');
%         flg = 1;
%         break;
%     end
% end
fgetl(mbed);
feed = fgetl(mbed);
if isempty(feed)
    resultNum = prev_sensorCal;
else
    if flg == 1
        feed = '0';
    end
    resultNum = str2num(feed);
end
% resultNum = feed;
if resultNum > 10000
    resultNum = prev_sensorCal;
end

end


