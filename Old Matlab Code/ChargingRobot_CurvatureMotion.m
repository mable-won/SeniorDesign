
% initial location is at (0,0). current location and final location are 
% constantly moving and are independent
% current location is constantly following final location until current 
% location is within a specified radius of of the final location
clear
clearvars
clc
tic
%% Charging Robot
initX = 1;
initY = 1;
Theta = 0;
CurrentLocation = [initX,initY];
CurrentPose = [CurrentLocation Theta];
%% Sensor Node
finalX = 20;
finalY = 15;
Goal = [finalX,finalY];
%% Other parameters
wheelDist = 0.5;
mult = 2;
goalRadius = 0.1;
throttle = 3;
slowdown = 0.4;
vmax = throttle*slowdown;
distanceToGoal = norm(CurrentLocation - Goal);
linVel = vmax;
lookaheadDistance = wheelDist*mult; % point where robot goes to next
chargingRobotRadius = 0.8;
counter = 0;
%% Initialize robot simulator
% Not really necessary as far as implementing in the physical robots
chargingRobot = ExampleHelperRobotSimulator('emptyMap',2);
chargingRobot.enableLaser(false);
chargingRobot.setRobotSize(chargingRobotRadius);
chargingRobot.showTrajectory(true);
chargingRobot.setRobotPose(CurrentPose);
plot(initX,initY,'b*'); hold on
plot(finalX,finalY,'rx');
controlRate = robotics.Rate(10); % This might be necessary because this defines
                                 % the frequency of the while loop

while (distanceToGoal > goalRadius) 
    %% pseudocode
    % distance to goal already calculated
    % calculate required curvature of charging robot to go to Goal
    % charging robot go to location
    % look at updated Goal
    % if Goal is not updated, continue moving towards Goal
    % if Goal is changed, compute distanceToGoal
    %lookahead = sqrt((CurrentLocation(1)-Goal(1))^2 + (CurrentLocation(2)-Goal(2))^2);
  
    counter = counter + 1;
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
            vRight = -vmax;
            angVel = (vRight - vLeft) / wheelDist;
        else 
            vLeft = -vmax;
            vRight = 1;
            angVel = (vRight - vLeft) / wheelDist;
        end
    else
        r = distanceToGoal^2 / (2*abs(tx));
        if tx > 0
            vLeft = vmax;
            vRight = vmax*(r - wheelDist/2)/(r + wheelDist/2);
            angVel = (vRight - vLeft) / wheelDist;
        else
            vLeft = vmax*(r - wheelDist/2)/(r + wheelDist/2);
            vRight = vmax;
            angVel = (vRight - vLeft) / wheelDist;
        end
        
    end
    
    drive(chargingRobot, linVel, angVel); % not necessary. this is for the simulator
    CurrentPose = chargingRobot.getRobotPose; % this is necessary but 
                                              %instead of chargingRobot.getRobotPose
                                              % we need to get CurrentPose
                                              % from the camera. FYI this
                                              % is the an array with
                                              % coordinates and orientation
                                              % info
    % should check location of Goal somewhere here. If Goal is the same
    % then continue. If not compute new distanceToGoal
    % The if statements is just a crude method to get Goal to move in different
    % locations. For our purposes, this chunk of code will be replace by a
    % shorter code that gets information about the coordinates of the
    % sensor nodes from the camera
    if counter == 100 
        finalX = 7;
        finalY = 12;
        Goal = [finalX,finalY];
        plot(Goal(1),Goal(2),'rx');
    end
    if counter == 150
        finalX = 10;
        finalY = 9;
        Goal = [finalX,finalY];
        plot(Goal(1),Goal(2),'rx');
    end
    if counter == 200
        finalX = 19;
        finalY = 21;
        Goal = [finalX,finalY];
        plot(Goal(1),Goal(2),'rx');
    end
    distanceToGoal = norm(CurrentPose(1:2) - Goal); % this is necessary
    waitfor(controlRate); %not sure. might be necessary
    
    
    
end
elapsedTime = toc;

