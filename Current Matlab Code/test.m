% TEST Test for CRTracking

global voltList;
global outVector;
mov_package = [77 0 0 0 0 0 0 0 0 67]; %default movement array
voltList = [1 9 2 10 3 11 4 12 5 0 6 0 7 0 8 0]; %define a test voltList
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
for carID = 9:12 %for every charging robot
    for i = 1:8 %find targetID
        if voltList(i*2)== carID
            break;
        end
    end
    targetID = voltList(i*2-1);
    %for carIndex = 1:12 %find camera data for charging robot
    %    if outVector(carIndex*2-3) == carID
    %        break;
    %    end
    %end
    %for targetIndex = 1:12 %find camera data for sensor node target
    %    if outVector(targetIndex*2-3) == targetID
    %        break;
    %    end
    %end
    % use if you don't want a simulation
    [vLeft,vRight] = CRTracking(carID, targetID);
    % use if you want the simulation
    %[vLeft,vRight] = CRTracking(carID, targetID, 1);
    mov_package((carID-8)*2) = vLeft;
    mov_package((carID-8)*2+1) = vRight;
end
disp(mov_package);


