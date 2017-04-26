function [data] = receiveData(s,dataLength)
% RECEIVEDATA Receive Data
% 
% [DATA] = receiveData(S,DATALENGTH) receives an API package sent over the xBee.
% S is the opened serial port.
% DATALENGTH is the expected length of the data, not the length of the
% package or the length in the second and third bytes of the package.
% A package containing the data 'Hello' will have dataLength = 5.
% DATA is the unpackaged data message.
%
% Examples
%
% Example 1: Hello World
% [s] = setupSerial('COM12');
% [message] = receiveData(s,13);
% fclose(s);
% disp(message);
%
% Example 2: SN Voltage Polling
% [~]=sendData(s,send_package); %Request information from Arduinos
% %Receive 2 bytes from each car, combine the bytes, and save in an array
% for car=1:8 
%     [packages(car,:)]=receiveData(s,2); 
%     voltages(car,1)=bitshift(packages(car,1),8)+packages(car,2); 
% end
% fclose(s);
%
% version 1.0 by R. Dunn at the University of Houston on 3/28/17

% Wait for package
while (s.BytesAvailable==0)
end
%got a package
data=zeros(1,dataLength);
package=fread(s,dataLength+9);
%extract data from package
for i=1:dataLength
    data(i)=package(8+i);
end
flushinput(s);
%address=package(6)+256*package(5);
end

