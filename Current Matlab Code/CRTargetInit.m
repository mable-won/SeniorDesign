% CRTARGETINIT Charging Robot Target Initialization
%
% Assign initial targets for each charging robot based on voltage and
% distance. Uses the global variables outVector and voltList and the 
% function munkres.
%
% version 1.2 by R. Dunn at the University of Houston on 4/18/17

%% Global Variables
global outVector;
global voltList;
global SNNumber;
global CRNumber;
pairs = min(SNNumber,CRNumber);
distanceArray=zeros(pairs,pairs);

%Testing code: will delete in a later version
%voltList=[1 0 2 0 3 0 4 0 5 0 6 0 7 0 8 0];
%for i=1:12
%    outVector(i*4-3)=i;
%    outVector(i*4)=0;
%    if i<9
%        outVector(i*4-2)=0;
%        outVector(i*4-1)=100/8*(i-0.5);
%    else
%        outVector(i*4-2)=100;
%        outVector(i*4-1)=100/4*(i-8.5);   
%    end
%end

%% Compute all distances between CRs and 4 lowest voltage SNs
for carID=SNNumber+1:CRNumber
    for carIndex=1:SNNumber+CRNumber %find carID in camera info
        if outVector(carIndex*4-3)==carID
            break;
        end
    end
    initX = outVector(carIndex*4-2);
    initY = outVector(carIndex*4-1);
    currentLocation = [initX,initY];
    for target=1:pairs
        targetID=voltList(target*2-1); %find carID of 4 lowest voltage SN
        for targetIndex=1:CRNumber+SNNumber %find targetID in camera info
            if outVector(targetIndex*4-3)==targetID
                break;
            end
        end
        finalX = outVector(targetIndex*4-2);
        finalY = outVector(targetIndex*4-1);
        goal = [finalX,finalY];
        distanceArray(carID-SNNumber,target) = norm(currentLocation - goal);
    end
end

%% Assign targets based on minimum distances
[assignment,cost] = munkres(distanceArray);
[assignedrows,~]=find(assignment);
for i=1:pairs
    voltList(i*2)=assignedrows(i)+SNNumber;
end

%Older Version: NON-OPTIMAL ASSIGNMENT
%{
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
%}
