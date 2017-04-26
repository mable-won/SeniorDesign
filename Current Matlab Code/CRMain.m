%CRMAIN Charging Robot Main
%
% CRMain controls all charging robot functions, including assignment, 
% tracking, alignment, attachment, and charging.
%
% version 0.2 by R. Dunn, M.C. Lalata, and M. Wan at the University of
% Houston on 4/18/17

%% Global Variables
global voltList;
global outVector; %#ok<NUSED>
global mov_package;
global grip_package;
global SNNumber;
global CRNumber;
%global collision; %#ok<NUSED>
SNNumber = 8;
CRNumber = 4;

%Preallocation
voltList = zeros(2*SNNumber);
%outVector = zeros(4*(CRNumber+SNNumber)); %#ok<PREALL>
mov_package = zeros(2*CRNumber+2);
grip_package = zeros(2*CRNumber+2);
packages=zeros(SNNumber,2);
voltages=zeros(SNNumber,1);

%% Initialization
s = setupSerial('COM7');

%Use this for voltList until ready to coordinate with SNGUI on another
%computer, then uncomment the code below, which will wait until SNGUI has
%begun.
for i = 1:SNNumber %use if voltage polling doesn't work
    voltList(i*2-1) = i;
end

%for car=1:SNNumber %use if voltage polling works
%    [packages(car,:)]=receiveData(s,2);
%    voltages(car,1)=bitshift(packages(car,1),8)+packages(car,2);
%end
%[~,index]=sort(voltages);
%for i=1:SNNumber
%    voltList(i*2-1)=index;
%end

% Create timers (doesn't start them yet)
t1 = createCRTimer(9);
t2 = createCRTimer(10);
t3 = createCRTimer(11);
t4 = createCRTimer(12);

% setup camera
vid = webcam(2); %connect to webcam
vid.Resolution = '1024x768';
TrackingChevron_RealTime(vid);

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
    TrackingChevron_RealTime(vid); %image processing
    counter = counter + 1; %counter will be removed in a later version
end

fclose(s);
delete(s);
delete(vid);
delete(t1);
delete(t2);
delete(t3);
delete(t4);