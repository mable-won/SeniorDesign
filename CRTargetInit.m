%% Charging Robot Target Initialization
%Assign initial targets for each charging robot based on voltage and
%distance.


%% Compute all distances between CRs and 4 lowest voltage SNs
for carID=9:12
    for carIndex=1:12 %find carID in camera info
        if output(carIndex*4-3)==carID
            break;
        end
    end
    initX = output(carIndex*4-2);
    initY = output(carIndex*4-1);
    currentLocation = [initX,initY];
    for target=1:4
        targetID=voltList(target*2-1); %find carID of 4 lowest voltage SN
        for targetIndex=1:12 %find targetID in camera info
            if output(targetIndex*4-3)==targetID
                break;
            end
        end
        finalX = output(targetIndex*4-2);
        finalY = output(targetIndex*4-1);
        goal = [finalX,finalY];
        distanceArray(carID-8,target) = norm(currentLocation - goal);
    end
end
%% Assign targets based on minimum distances
rMax = [1000,1000,1000,1000];
cMax = [1000;1000;1000;1000];
for i=1:1:4
    if i~=1
        distanceArray(I_row,:)=rMax; %Replace row
        distanceArray(:,I_col)=cMax; %Replace column
    end
    [~,I]=min(distanceArray(:)); %Find location of new min distance
    [I_row, I_col] = ind2sub(size(distanceArray),I);
    voltList(I_col*2)=I_row+8; %Assign carID to target SN
end

