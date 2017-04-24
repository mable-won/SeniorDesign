% TEST Collection of Tests to Run on Vehicles and Modules
%
% Uncomment the brackets to use one (and only one) test. Included in this
% version is a test for the charging robot tracking, a test for the
% charging robot movement and gripper, and a test for sensor nodes movement
% and request voltage.
%
% version 1.1 by R. Dunn at the University of Houston on 4/12/17

%% Test for CRTracking
% Do not use this test. Will be deleted in a later version.

%{
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
vLeft = 0; vRight = 0;
while((vLeft~=0 && vRight~=0) || counter==0)
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
    %disp(mov_package);
    [~] = sendData(s,mov_package);
    counter = counter + 1;
    pause(0.1);
end
[~] = sendData(s,[77 0 0 0 0 0 0 0 0 67]);
fclose(s);
delete(s);
%}

%% CR Test for Movement and Gripper (aka Pacman)
% Test for 4 charging robots

%{
s = setupSerial('COM5');
counter = 0;
while(counter < 2)
    p1 = [71 0 0 0 0 0 0 0 0 82];
    p2 = [77 100 100 100 100 100 100 100 100 67];
    time1 = tic;
    while (toc(time1) < 0.3)
        [~] = sendData(s,p1);
        [~] = sendData(s,p2);
    end
    p1 = [77 0 0 0 0 0 0 0 0 67];
    p2 = [71 1 1 1 1 0 0 0 0 82];
    time2 = tic;
    while (toc(time2) < 0.2)
        [~] = sendData(s,p1);
        [~] = sendData(s,p2);
    end
    counter = counter + 1;
    % Delete all timers from memory.
    listOfTimers = timerfindall;
    if ~isempty(listOfTimers)
        delete(listOfTimers(:));
    end
end
fclose(s);
delete(s);
%}

%% SN Test for Movement
% Test for 5 sensor nodes

%{
s = setupSerial('COM5');
p = [67 100 100 100 100 100 100 100 100 100 100 77];
time1 = tic;
while (toc(time1) < 0.34)
    [~] = sendData(s,p);
end
p = [67 0 0 0 0 0 0 0 0 0 0 77];
time2 = tic;
while (toc(time2) < 0.1)
    [~] = sendData(s,p);
end
fclose(s);
delete(s);
% Delete all timers from memory.
listOfTimers = timerfindall;
if ~isempty(listOfTimers)
    delete(listOfTimers(:));
end
%}

%% SN Test for Voltage Polling
% This tests the voltage polling. The unreliability of XBee package
% retrieval prevents this from being a reliable method of obtaining
% voltages.

%{
clear
clc
s = setupSerial('COM7');
request_package = [86 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 63];
timer = tic;
while toc(timer) < 0.01
    [~] = sendData(s,request_package);
end
[package,data]=receiveData(s,2); 
% Delete all timers from memory.
    listOfTimers = timerfindall;
    if ~isempty(listOfTimers)
        delete(listOfTimers(:));
    end
%     voltages(car,1)=bitshift(packages(car,1),8)+packages(car,2); 
fclose(s);
%}