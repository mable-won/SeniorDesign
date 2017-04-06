% TEST Collection of Tests to Run on Vehicles and Modules
%
% Uncomment the brackets to use one (and only one) test. Included in this
% version is a test for the charging robot tracking, a test for the
% charging robot movement and gripper, and a test for sensor nodes movement
% and request voltage.
%
% version 1.0 by R. Dunn at the University of Houston on 4/06/17

%% Test for CRTracking

%{
clc
global voltList;
global outVector;
mov_package = [77 0 0 0 0 0 0 0 0 67];
voltList = [1 9 2 10 3 11 4 12 5 0 6 0 7 0 8 0]; %define a test voltList
s = setupSerial('COM5');
for i=1:12 %define a test outVector
    outVector(i*4-3)=i; %carID
    outVector(i*4)=0; %orientation
    if i<9
        outVector(i*4-2)=0; %x-coordinate
        outVector(i*4-1)=100/8*(i-0.5); %y-coordinate
    else
        outVector(i*4-2)=100; %x-coordinate
        outVector(i*4-1)=100/4*(i-8.5); %y-coordinate  
    end
end
counter = 0;
while(counter < 50)
    for carID = 9:12 %for every charging robot
        t = createCRTimer(carID);
        % use if you don't want a simulation
        [vLeft,vRight] = CRTracking(carID, t);
        % use if you want the simulation
        %[vLeft,vRight] = CRTracking(carID, t, 1);
        mov_package((carID-8)*2) = vLeft;
        mov_package((carID-8)*2+1) = vRight;
        UpdateLocation(vLeft,vRight,carID);
    end
    counter = counter + 1;
    %disp(mov_package);
    [~] = sendData(s,mov_package);
    pause(0.1);
end
[~] = sendData(s,[77 0 0 0 0 0 0 0 0 67]);
fclose(s);
delete(s);
%}

%% CR Test for Movement and Gripper (aka Pacman)

%{
s = setupSerial('COM5');
counter = 0;
while(counter < 20)
    p = [71 0 0 0 0 0 0 0 0 82];
    [~] = sendData(s,p);
    p = [77 100 100 100 100 100 100 100 100 67];
    [~] = sendData(s,p);
    pause(0.3);
    p = [77 0 0 0 0 0 0 0 0 67];
    [~] = sendData(s,p);
    p = [71 1 1 1 1 0 0 0 0 82];
    [~] = sendData(s,p);
    counter = counter + 1;
    pause(0.2);
end
fclose(s);
delete(s);
%}

%% SN Test for Movement and Request Voltage

%{
s = setupSerial('COM5');
%[voltPackage] = receiveData(s,10); %voltPackage is array of targets ordered by voltage
%for i = 1:8 %voltList contains the array of sensor nodes and its assigned charging robot
%    voltList(i*2-1)= voltPackage(i); 
%end
p = [67 228 100 228 100 228 100 228 100 228 100 228 100 228 100 228 100 77];
[~] = sendData(s,p);
pause(0.3);
p = [67 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 77];
[~] = sendData(s,p);
pause(0.02);
p = [67 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 77];
[~] = sendData(s,p); % duplicate stop package in case of packet loss
fclose(s);
delete(s);
%}