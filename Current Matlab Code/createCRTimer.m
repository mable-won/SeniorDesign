function [t] = createCRTimer(carID)
% CREATECRTIMER Create Charging Robot Timer
%
% [T] = createCRTimer(CARID) creates a timer T associated with a given ID
% of a charging robot, CARID, which is saved under T.UserData.
% Note that this does not start the timer, but merely sets it.
%
% Examples
%
% Example 1: Timer
% t1=createCRTimer(9);
% start(t1);
% %timer will automatically execute finishCRTimer followed by stopCRTimer
% %after t.StartDelay seconds
%
% Example 2: Stopping a timer early
% t1=createCRTimer(9);
% start(t1);
% stop(t1);
%
% Example 3: Querying the status of the timer
% t1=createCRTimer(9);
% get(t1,'Running');
% %ans = 'off' or 'on'
%
% version 1.0 by R. Dunn at the University of Houston on 3/30/17

t=timer;
t.ExecutionMode = 'singleShot';
t.UserData = carID;
t.StartDelay = 60; %time in seconds
t.BusyMode = 'queue';
t.TimerFcn = @finishCRTimer;
t.StopFcn = @stopCRTimer;
end

function finishCRTimer(t,~)
% FINISHCRTIMER Finish Charging Robot Timer
%
% finishCRTimer(T,~) is executed for timer T after T.StartDelay seconds.
global grip_package;
global mov_package;
grip_package(t.UserData-8) = 1; %open gripper
mov_package((t.UserData-8)*2) = 228; %move backward until next loop
mov_package((t.UserData-8)*2+1) = 228;
[~] = CRAssignment(t.UserData);
fprintf('Car %d is finished.\n',t.UserData);
end

function stopCRTimer(t,~)
% STOPCRTIMER Stop Charging Robot Timer
%
% stopCRTimer(T,~) is executed for timer T after the finishCRTimer executes. 
delete(t);
end