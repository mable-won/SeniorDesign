function [s] = setupSerial(comPort)
% Initialize the serialpush port communication between Arduino and MATLAB
% [s] = setupSerial(comPort)
% The input value is the COMPORT should be changed as per requirement
% We ensure that the arduino is also communicatiing with MATLAB at this
% time. A predefined code on the arduino acknowledges this. 
% if setup is complete then the value of setup is returned as 1 else 0


s=serial(comPort,'BaudRate',57600,'Terminator',0,'StopBit',1,'Parity','None');
set(s,'InputBufferSize',110);
fopen(s);
end

