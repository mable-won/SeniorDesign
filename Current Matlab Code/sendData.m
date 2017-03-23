function [package] = sendData(s,data,address)
% sendData packages the data and transmits the API package over the xBee.
% [package] = sendData(s,data,address)
% [package] = sendData(s,data)
% s is the opened serial port.
% data should be an array of decimal integers.
% address (optional) should be a 16-bit hex. (default: 'FFFF')

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
