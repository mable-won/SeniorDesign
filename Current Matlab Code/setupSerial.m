function [s] = setupSerial(comPort)
% SETUPSERIAL Setup Serial
%
% [S] = setupSerial(COMPORT) initializes the serial port communication
% between Arduino and MATLAB
% 
% Examples
%
% Example: Windows 'COM#'
% s = setupSerial('COM12');
%
% Example: Mac OS '/dev/tty.KeySerial#'
% s = setupSerial('/dev/tty.KeySerial1');
%
% version 1.0 by R. Dunn at the University of Houston on 3/28/17

s=serial(comPort,'BaudRate',57600,'Terminator',0,'StopBit',1,'Parity','None');
set(s,'InputBufferSize',110);
fopen(s);
end

