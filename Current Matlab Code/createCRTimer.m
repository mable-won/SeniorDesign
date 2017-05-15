function [t] = createCRTimer(carID,time)
% CREATECRTIMER Create Charging Robot Timer
%
% [T] = createCRTimer(CARID, TIME) creates a timer T associated with a
% given ID of a charging robot, CARID, which is saved under T.UserData.
% TIME is an optional time in seconds to set on the timer. Default is 60 s.
% Note that this does not start the timer, but merely sets it. Note that
% there is a warning issued when running multiple timers in CRMain. We
% never found a way to bypass the warning.
%
% Examples
%
% Example 1: Creating a timer
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
% version 1.3 by R. Dunn at the University of Houston on 4/18/17

if nargin == 1
    time = 60;
end
t=timer;
t.ExecutionMode = 'singleShot';
t.UserData = carID;
t.StartDelay = time; %time in seconds
t.BusyMode = 'queue';
t.TimerFcn = @finishCRTimer;
%t.StopFcn = @stopCRTimer;
end

function finishCRTimer(t,~)
% FINISHCRTIMER Finish Charging Robot Timer
%
% finishCRTimer(T,~) is executed for timer T after T.StartDelay seconds.

global grip_package;
global mov_package;
global SNNumber
grip_package(t.UserData-SNNumber) = 1; %open gripper
mov_package((t.UserData-SNNumber)*2) = 228; %move backward until next loop
mov_package((t.UserData-SNNumber)*2+1) = 228;
[~] = CRAssignment(t.UserData);
end

%function stopCRTimer(~,~)
% STOPCRTIMER Stop Charging Robot Timer
%
% stopCRTimer(T,~) is executed for timer T after the finishCRTimer executes. 

%delete(t);
%end