function [data] = receiveData(s,dataLength)
% RECEIVEDATA receives an API package sent over the xBee.
% [data] = receiveData(s,dataLength)
% s is the opened serial port.
% dataLength is the expected length of the data, not the length of the
% package or the length in the second and third bytes of the package.
% A package containing the data 'Hello' will have dataLength = 5.
% Note that all inputs are required.

% Wait for package
while (s.BytesAvailable==0)
end
%got a package
data=zeros(1,dataLength);
package=fread(s,[1,dataLength+9]);
%extract data from package
for i=1:dataLength
    data(i)=package(8+i);
end
%address=package(6)+256*package(5);
end

