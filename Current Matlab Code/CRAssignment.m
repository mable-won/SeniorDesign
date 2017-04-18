function [ target ] = CRAssignment( carID, prevTarget )
% CRASSIGNMENT Charging Robot Assignment
%
% [ TARGET ] = CRAssignment( CARID, PREVTARGET ) returns a target carID in
% TARGET to a charging robot given its CARID and PREVTARGET based on an
% order of priority given by the global variable voltList. This function
% also alters voltList. Note that prevTarget is optional.
%
% Example:
%
% global voltList;
% voltList = [1 9 2 10 3 11 4 12 5 0 6 0 7 0 8 0];
% for carID = 9:12 %for every charging robot
%     for targetID = 1:8 %search for prevTarget in voltList
%        if voltList(targetID * 2) == carID
%            break;
%        end
%     end
%     prevTarget = voltList(targetID * 2 - 1);
%     target = CRAssignment(carID,prevTarget);
% end
% disp(voltList);
%
%
% version 1.2 by R. Dunn at the University of Houston on 4/18/17


%% Move previous target to end of voltList
global voltList;
global SNNumber;

list=voltList;
if nargin==1
    for i = 1:SNNumber %find prevTarget
        if voltList(i*2) == carID 
            break;
        end
    end
    prevTarget = voltList(i*2-1);
end

for index=1:SNNumber
    if list(index*2-1)==prevTarget
        break;
    elseif index==SNNumber
        target=prevTarget;
        disp('Error: Previous target not found.');
        return;
    end
end
list(index*2)=[];
list(index*2-1)=[];
list(2*SNNumber-1)=prevTarget;
list(2*SNNumber)=0;
%% Assign target
for index=1:SNNumber
    if list(index*2)==0
        break;
    elseif index==SNNumber
        target=prevTarget;
        disp('Error: No unassigned sensor nodes.');
        return;
    end
end
list(index*2)=carID;
target=list(index*2-1);
voltList=list;
end

