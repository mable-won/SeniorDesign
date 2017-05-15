% TEST Collection of Tests to Run on Vehicles and Modules
%
% Uncomment the brackets to use one (and only one) test. Included in this
% version is a test for the charging robot movement and gripper, a test
% for sensor nodes movement and request voltage, a test for the camera
% and the demo shown in our video.
%
% version 1.2 by R. Dunn at the University of Houston on 5/12/17

%% CR Test for Movement and Gripper (aka Pacman)
% Test for 4 charging robots

%{
s = setupSerial('COM7');
counter = 0;
while(counter < 10)
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
    while (toc(time2) < 0.3)
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
s = setupSerial('COM7');
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
%   1. Run test.m first, will wait to read package.
%   2. Run SNGUI.m on another computer.

%{
SNNumber = 5;
s = setupSerial('COM7');
voltages = zeros(1,SNNumber);
voltList = zeros(1,SNNumber*2);
for car=1:SNNumber
    package=receiveData(s,2);
    voltages(car) = bitshift(package(1),8)+package(2);
end
[~,index]=sort(voltages);
for i=1:SNNumber
    voltList(i*2-1)=index(i);
end
fclose(s);
%}

%% Senior Design Demo
% This demo moves four charging robots forward a specified set of distances
% before latching onto four sensor nodes. Sensor nodes must be aligned with
% charging robots. This demo doesn't use the overhead camera, so that the
% video could be taken, but this demo can easily be rewritten to include
% the camera information.

%{
s = setupSerial('COM7');

CRNumber = 4;
SNNumber = 5;

mov_package = [77 0 0 0 0 0 0 0 0 67];
grip_package = [71 0 0 0 0 0 0 0 0 82];
distances = [18 12 37 26]; %in cm
threshhold = 0.6; %in cm
speed = 14; %cm/s
time = 0.1; %in s
counter = 0; %dummy counter, will be replaced
while (counter < 50) %to be replaced by get(hObject,'Value')
    if counter==1
        elapsedtime=tic;
    elseif counter==2
         time=toc(elapsedtime);
    end
    for carID = SNNumber+1:SNNumber+CRNumber
        if distances(carID-SNNumber)>threshhold
            vLeft = 100;
            vRight = 100;
            gripper = 1;
            distances(carID-SNNumber)=distances(carID-SNNumber)-time*speed;
        else
            vLeft = 0;
            vRight = 0;
            gripper = 0;
        end
        mov_package((carID-SNNumber)*2) = vLeft;
        mov_package((carID-SNNumber)*2+1) = vRight;
        grip_package((carID-SNNumber+1)) = gripper;
    end
    t=tic;
    while toc(t)<0.1
        [~]=sendData(s,mov_package);
        [~]=sendData(s,grip_package);
    end
    % Delete all timers from memory.
    listOfTimers = timerfindall;
    if ~isempty(listOfTimers)
        delete(listOfTimers(:));
    end
    counter = counter + 1; %counter will be removed in a later version
end
for index = 2:2*CRNumber+1
     mov_package(index) = 0; %stop
     %grip_package(index) = 1; %open
end
t=tic;
while toc(t)<0.1
    sendData(s,mov_package);
    %sendData(s,grip_package);
end
fclose(s);
delete(s);
% Delete all timers from memory.
listOfTimers = timerfindall;
if ~isempty(listOfTimers)
    delete(listOfTimers(:));
end
%}