function [ vLeft, vRight ] = CRTracking( carID, target, simulation )
%CRTracking computes the wheel thrusts required for a particular car to
%track a target robot.
%   Assuming camera information is a global variable called outVector; carID 
%   is the number for a charging robot, ie 9 to 12; target contains the 
%   carID of the target; and simulation is a optional boolean stating 
%   whether the simulation is requested.
%   Also assumes the main function contains a loop for all vehicles and a 
%   togglebutton controlled loop around that. Note that wheel thrusts are
%   not sent wirelessly in this function.
%% Check inputs
if nargin > 3 || nargin < 1
    disp('Error: Check number of inputs.');
    vRight=0; vLeft=0;
    return;
elseif nargin == 2
    simulation=0;
end
%% Charging Robot
global outVector;
for i=1:12 %used if camera array does not have robots in numerical order
    if output(i*4-3)==carID
        break;
    elseif i==12
        vLeft=0; vRight=0;
        disp('Initial data not found.');
        return;
    end
end
initX = outVector(i*4-2);
initY = outVector(i*4-1);
Theta = outVector(i*4);
CurrentLocation = [initX,initY];
CurrentPose = [CurrentLocation Theta];
%% Sensor Node
for i=1:12 %used if camera array does not have robots in numerical order
    if outVector(i*4-3)==target
        break;
    elseif i==12
        vLeft=0; vRight=0;
        disp('Target data not found.');
        return;
    end
end
finalX = outVector(i*4-2);
finalY = outVector(i*4-1);
Goal = [finalX,finalY];
%% Other parameters
wheelDist = 3; %in cm
goalRadius = 10; %in cm
throttle = 127; %max value, do not change
slowdown = 0.8; %safety factor
vmax = round(throttle*slowdown); %needs to be an int
distanceToGoal = norm(CurrentLocation - Goal);

%% Initialize robot simulator
% Not really necessary as far as implementing in the physical robots
if simulation
    linVel = vmax;
    chargingRobotRadius = 8; %radius of vehicle
    chargingRobot = ExampleHelperRobotSimulator('emptyMap',2);
    chargingRobot.enableLaser(false);
    chargingRobot.setRobotSize(chargingRobotRadius);
    chargingRobot.showTrajectory(true);
    chargingRobot.setRobotPose(CurrentPose);
    plot(initX,initY,'b*'); hold on
    plot(finalX,finalY,'rx');
    controlRate = robotics.Rate(10);
end
%% Alignment and Attachment Algorithm
if (distanceToGoal > goalRadius) 
    %eventually, the alignment code will go here, but for now...
    vLeft=0; vRight=0;
    return;
else
%% Tracking Algorithm
    dx = Goal(1) - CurrentPose(1);
    dy = Goal(2) - CurrentPose(2);
    theta = -(CurrentPose(3) - pi/2);
    tx = dx*cos(theta) - dy*sin(theta);
    ty = dx*sin(theta) + dy*cos(theta);
    if abs(tx) < 0.005*distanceToGoal && ty > 0
        vRight = vmax;
        vLeft = vRight;
        angVel = 0; %angVel is not necessary for the robots. ignore all instances
    elseif abs(tx) < distanceToGoal && ty < 0
        if tx > 0
            vLeft = 1;
            vRight = bitxor(vmax,128);
            angVel = (bitxor(vRight,128) - vLeft) / wheelDist;
        else 
            vLeft = bitxor(vmax,128);
            vRight = 1;
            angVel = (vRight - bitxor(vLeft,128)) / wheelDist;
        end
    else
        r = distanceToGoal^2 / (2*abs(tx));
        if tx > 0
            vLeft = vmax;
            if r-wheelDist/2<0
                vRight = bitxor(vmax*(r - wheelDist/2)/(r + wheelDist/2),128);
                angVel = (bitxor(vRight,128) - vLeft) / wheelDist;
            else
                vRight = vmax*(r - wheelDist/2)/(r + wheelDist/2);
                angVel = (vRight - vLeft) / wheelDist;
            end    
        else
            vRight = vmax;
            if r-wheelDist/2<0
                vLeft = bitxor(vmax*(r - wheelDist/2)/(r + wheelDist/2),128);
                angVel = (vRight - bitxor(vLeft,128)) / wheelDist;
            else
                vLeft = vmax*(r - wheelDist/2)/(r + wheelDist/2);
                angVel = (vRight - vLeft) / wheelDist;
            end
        end 
    end
    %Simulation update
    if simulation
        drive(chargingRobot, linVel, angVel);
        plot(Goal(1),Goal(2),'rx');
        distanceToGoal = norm(CurrentPose(1:2) - Goal);
        waitfor(controlRate);
    end
end
end

