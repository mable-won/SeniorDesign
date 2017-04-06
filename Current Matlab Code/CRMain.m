%CRMAIN Charging Robot Main
%
% CRMain controls all charging robot functions, including assignment, 
% tracking, alignment, attachment, and charging.
%
% version 0.1 by R. Dunn, M.C. Lalata, and M. Wan at the University of
% Houston on 4/06/17

%% Global Variables
global voltList;
global outVector;
global mov_package;
global grip_package;
global collision; %#ok<NUSED>
%voltList = zeros(16);
outVector = zeros(48);

%% Initialization
s = setupSerial('COM7');

%Use this for voltList until ready to coordinate with SNGUI on another
%computer, then uncomment the code below, which will wait until SNGUI has
%begun.
voltList = [1 0 2 0 3 0 4 0 5 0 6 0 7 0 8 0];
%[voltPackage] = receiveData(s,10); %voltPackage is array of targets ordered by voltage
%for i = 1:8 %voltList contains the array of sensor nodes and its assigned charging robot
%    voltList(i*2-1) = voltPackage(i); 
%end

% Create timers (doesn't start them yet)
t1 = createCRTimer(9);
t2 = createCRTimer(10);
t3 = createCRTimer(11);
t4 = createCRTimer(12);

% setup camera

% Image processing
CRTargetInit;

%% Main Loop
counter = 0; %dummy counter, will be replaced
while (counter < 10) %to be replaced by get(hObject,'Value')
    mov_package = [77 0 0 0 0 0 0 0 0 67];
    grip_package = [71 1 1 1 1 1 1 1 1 82];
    for carID = 9:12
        switch carID %assign timer
            case 9
                if strcmp(get(t1,'Running'),'off')
                    [vLeft,vRight] = CRTracking(carID,t1);
                else % charging
                    vLeft = 0;
                    vRight = 0; 
                    grip_package(carID-7) = 0; %close gripper
                end
            case 10
                if strcmp(get(t2,'Running'),'off')
                    [vLeft,vRight] = CRTracking(carID,t2);
                else % charging
                    vLeft = 0;
                    vRight = 0; 
                    grip_package(carID-7) = 0; %close gripper
                end
            case 11
                if strcmp(get(t3,'Running'),'off')
                    [vLeft,vRight] = CRTracking(carID,t3);
                else % charging
                    vLeft = 0;
                    vRight = 0; 
                    grip_package(carID-7) = 0; %close gripper
                end
            case 12
                if strcmp(get(t4,'Running'),'off')
                    [vLeft,vRight] = CRTracking(carID,t4);
                else % charging
                    vLeft = 0;
                    vRight = 0; 
                    grip_package(carID-7) = 0; %close gripper
                end
        end
        mov_package((carID-8)*2) = vLeft;
        mov_package((carID-8)*2+1) = vRight;
    end
    sendData(s,mov_package);
    sendData(s,grip_package);
    % image processing
    counter = counter + 1;
end
fclose(s);
fdelete(s);
delete(t1);
delete(t2);
delete(t3);
delete(t4);