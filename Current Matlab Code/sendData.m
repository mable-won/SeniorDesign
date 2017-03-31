function [package] = sendData(s,data,address)
% SENDDATA Send Serial Data
%
% [PACKAGE] = sendData(S,DATA,ADDRESS) packages the data and transmits the
% API package over the xBee.
% S is the opened serial port.
% DATA should be an array of positive decimal integers.
% ADDRESS (optional) should be a 16-bit hex. (default: '0xFFFF' for
% broadcast)
%
% Examples
%
% Example 1: Hello World
% [s] = setupSerial('COM12');
% [~] = sendData(s,'Hello, World!','000D');
% fclose(s);
%
% Example 2: SN Movement (forward, stop, forward, right, forward, stop,
% forward, left)
% data = [67 100 100 0 0 100 100 100 228 100 100 0 0 100 100 228 100 77];
% [~] = sendData(s,data);
%
% Example 3: SN Voltage Request (data must be exactly as below)
% data = [86 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 63];
% [~] = sendData(s,data);
%
% Example 4: CR Movement (forward, stop, left, right)
% data = [77 100 100 0 0 228 100 100 228 67];
% [~] = sendData(s,data);
%
% Example 5: CR Gripper (open, closed, open, closed)
% data = [71 1 0 1 0 0 0 0 0 82];
% [~] = sendData(s,data);
%
% version 1.0 by R. Dunn at the University of Houston on 3/28/17

%Check inputs
if(nargin > 3 || nargin < 1)
 disp('Error: Check inputs.');
 return;
end
if(nargin == 2)
 address='FFFF';
end
l=length(data);
if l>100
    disp('Error: Too much data.');
    return;
end

%Package formatting: header byte, length of package, package type,
%frame ID, address, options, data, checksum

%Create constant values for 16-bit transmission (in decimal)
header=126;
packLength=l+5;
packType=1;
frameID=1;
options=1; %0x01 disables ACK, speeds up communication time
%if response desired, set to 0x00 and uncomment last 10 lines

%create empty package
package=zeros(1,packLength+4);
%fill package
package(1)=header;
package(2)=bitshift(bitand(packLength,65280),-8);
package(3)=bitand(packLength,255);
package(4)=packType;
package(5)=frameID;
package(6)=bitshift(bitand(hex2dec(address),65280),-8);
package(7)=bitand(hex2dec(address),255);
package(8)=options;
checksum=packType+frameID+bitshift(bitand(hex2dec(address),65280),-8)...
+bitand(hex2dec(address),255)+options;
for n=1:l
    package(8+n)=data(n);
    checksum=checksum+data(n);
end
checksum=bitand(checksum,255);
package(packLength+4)=255-checksum;
%send package
fwrite(s,package);

%%Receive response package
%response=fread(s,[1,7]);
%counter=0;
%%Check that the package was successfully received.
%while ((numel(response)~=0&&counter>4)||(response(6)~=0&&counter>4))
%   fwrite(s,package);
%   response=fread(s,[1,7]);
%   counter=counter+1;
%   if counter==5
%       disp('Error: Package not received.');
%   end
%end
end
