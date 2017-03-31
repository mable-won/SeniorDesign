%CRMAIN Charging Robot Main
%
% CRMain controls all charging robot functions, including assignment, 
% tracking, alignment, attachment, and charging.
%
% version 0.0 by R. Dunn, M.C. Lalata, and M. Wan at the University of
% Houston on 3/31/17

%% Global Variables
global voltList; %#ok<NUSED>
global outVector; %#ok<NUSED>
global mov_package;
global grip_package;
global collision; %#ok<NUSED>

%% Initialization
s = setupSerial('COM#');
% get voltages
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
                if (get(t1,'Running') == 'off')
                    [vLeft,vRight] = CRTracking(carID,t1);
                else % charging
                    vLeft = 0;
                    vRight = 0; 
                    grip_package(carID-8) = 0; %close gripper
                end
            case 10
                if (get(t2,'Running') == 'off')
                    [vLeft,vRight] = CRTracking(carID,t2);
                else % charging
                    vLeft = 0;
                    vRight = 0; 
                    grip_package(carID-8) = 0; %close gripper
                end
            case 11
                if (get(t3,'Running') == 'off')
                    [vLeft,vRight] = CRTracking(carID,t3);
                else % charging
                    vLeft = 0;
                    vRight = 0; 
                    grip_package(carID-8) = 0; %close gripper
                end
            case 12
                if (get(t4,'Running') == 'off')
                    [vLeft,vRight] = CRTracking(carID,t4);
                else % charging
                    vLeft = 0;
                    vRight = 0; 
                    grip_package(carID-8) = 0; %close gripper
                end
        end
        mov_package((carID-8)*2) = vLeft;
        mov_package((carID-8)*2) = vRight;
    end
    sendData(s,mov_package);
    sendData(s,grip_package);
    % image processing
    counter = counter +1;
end
fclose(s);
fdelete(s);
delete(t1);
delete(t2);
delete(t3);
delete(t4);