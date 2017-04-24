%CRMAIN Charging Robot Main
%
% CRMain controls all charging robot functions, including assignment, 
% tracking, alignment, attachment, and charging.
%
% version 0.2 by R. Dunn, M.C. Lalata, and M. Wan at the University of
% Houston on 4/18/17

%% Global Variables
global voltList;
global outVector;
global mov_package;
global grip_package;
global SNNumber;
global CRNumber;
%global collision; %#ok<NUSED>
SNNumber = 8;
CRNumber = 4;
%voltList = zeros(2*SNNumber);
%outVector = zeros(4*(CRNumber+SNNumber)); %#ok<PREALL>
mov_package = zeros(2*CRNumber+2);
grip_package = zeros(2*CRNumber+2);

%% Initialization
s = setupSerial('COM7');

%Use this for voltList until ready to coordinate with SNGUI on another
%computer, then uncomment the code below, which will wait until SNGUI has
%begun.
for i = 1:SNNumber
    voltList(i*2-1) = i;
end
%[voltPackage] = receiveData(s,SNNumber); %voltPackage is array of targets ordered by voltage
%for i = 1:SNNumber %voltList contains the array of sensor nodes and its assigned charging robot
%    voltList(i*2-1) = voltPackage(i); 
%end

% Create timers (doesn't start them yet)
t1 = createCRTimer(9);
t2 = createCRTimer(10);
t3 = createCRTimer(11);
t4 = createCRTimer(12);

% setup camera
vid = webcam(2); %connect to webcam
samp1 = snapshot(vid); %take a photo
outVector = TrackingChevron_RealTime(samp1);

% Image processing
CRTargetInit;

%% Main Loop
counter = 0; %dummy counter, will be replaced
while (counter < 20) %to be replaced by get(hObject,'Value')
    mov_package(1) = 77; mov_package(2*CRNumber+2) = 67;
    grip_package(1) = 71; grip_package(2*CRNumber+2) = 82;
    for index = 2:2*CRNumber+1
        mov_package = 0; %stop
        grip_package = 1; %open
    end
    for carID = SNNumber+1:SNNumber+CRNumber
        switch carID %assign timer
            case 9
                if strcmp(get(t1,'Running'),'off')
                    [vLeft,vRight] = CRTracking(carID,t1);
                else % charging
                    vLeft = 0;
                    vRight = 0; 
                    grip_package(carID-SNNumber+1) = 0; %close gripper
                end
            case 10
                if strcmp(get(t2,'Running'),'off')
                    [vLeft,vRight] = CRTracking(carID,t2);
                else % charging
                    vLeft = 0;
                    vRight = 0; 
                    grip_package(carID-SNNumber+1) = 0; %close gripper
                end
            case 11
                if strcmp(get(t3,'Running'),'off')
                    [vLeft,vRight] = CRTracking(carID,t3);
                else % charging
                    vLeft = 0;
                    vRight = 0; 
                    grip_package(carID-SNNumber+1) = 0; %close gripper
                end
            case 12
                if strcmp(get(t4,'Running'),'off')
                    [vLeft,vRight] = CRTracking(carID,t4);
                else % charging
                    vLeft = 0;
                    vRight = 0; 
                    grip_package(carID-SNNumber+1) = 0; %close gripper
                end
        end
        mov_package((carID-SNNumber)*2) = vLeft;
        mov_package((carID-SNNumber)*2+1) = vRight;
    end
    sendData(s,mov_package);
    sendData(s,grip_package);
    % image processing
    samp1 = snapshot(vid); %take a photo
    outVector = TrackingChevron_RealTime(samp1);
    counter = counter + 1;
end
%}
fclose(s);
delete(s);
delete(vid);
delete(t1);
delete(t2);
delete(t3);
delete(t4);