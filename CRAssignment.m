function [ target ] = CRAssignment( carID, prevTarget )
%CRAssignment assigns a target to a robot to track based on an order of
%priority given by the global variable voltList.
%   The carID is the identifier of the charging robot, ie 9-12; prevTarget
%   is the identifier of the previous target sensor node; target is the new
%   target sensor node identifier.

global voltList;
list=voltList;
%% Move previous target to end of voltList
for index=1:8
    if list(index*2-1)==prevTarget
        break;
    elseif index==8
        target=0;
        disp('Error: Previous target not found.');
        return;
    end
end
list(index*2)=[];
list(index*2-1)=[];
list(15)=prevTarget;
list(16)=0;
%% Assign target
for index=1:8
    if list(index*2)==0
        break;
    elseif index==8
        target=0;
        disp('Error: No unassigned sensor nodes.');
        return;
    end
end
list(index*2)=carID;
target=list(index*2-1);
voltList=list;
end

