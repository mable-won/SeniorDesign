function [ vLeft, vRight ] = CRTracking( carID, t, simulation ) %#ok<INUSL>
% CRTRACKING Charging Robot Tracking
%
% [ VLEFT, VRIGHT ] = CRTracking( CARID, T, SIMULATION ) returns the
% left and right wheel thrusts, VLEFT and VRIGHT, required for a particular
%  car to track a target robot.
% Assuming camera information is a global variable called outVector; CARID 
% is the number for a charging robot, ie 9 to 12; T is the timer associated
% with CARID, and SIMULATION is an optional boolean stating whether the
% simulation is requested.
% CRTracking also assumes the main function contains a loop for all vehicles and a 
% togglebutton controlled loop around that. Note that wheel thrusts are
% not sent wirelessly in this function.
%
% Example:
%{
global voltList;
global outVector;
mov_package = [77 0 0 0 0 0 0 0 0 67];
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
    t = createCRTimer(carID);
    % use if you don't want a simulation
    [vLeft,vRight] = CRTracking(carID, t);
    % use if you want the simulation
    %[vLeft,vRight] = CRTracking(carID, t, 1);
    mov_package((carID-8)*2-1) = vLeft;
    mov_package((carID-8)*2) = vRight;
end
%}
%
% version 1.1 by M.C. Lalata and R. Dunn at the University of Houston on
% 3/31/17
%% Check inputs
global outVector;
global voltList;
if nargin > 2 || nargin < 1
    disp('Error: Check number of inputs.');
    vRight=0; vLeft=0;
    return;
elseif nargin == 1
    simulation=0;
end
%% Charging Robot
for i=1:12 %used if camera array does not have robots in numerical order
    if outVector(i*4-3)==carID
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
for i = 1:8 %find targetID
    if voltList(i*2)== carID
        break;
    end
end
target = voltList(i*2-1);
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
    rad = 3;
    mag = 5;
    hold on
    xlim([-5 60]) %length of arena
    ylim([-5 60]) %width of arena
    angs = 0:pi/10:2*pi;
    x = CurrentPose(1) + rad*cos(angs);
    y = CurrentPose(2) + rad*sin(angs);
    circ = plot(x,y,'b');
    angx1 = [CurrentPose(1), CurrentPose(1)+mag*cos(CurrentPose(3)+ pi + pi/8)];
    angy1 = [CurrentPose(2), CurrentPose(2)+mag*sin(CurrentPose(3)+ pi + pi/8)];
    angx2 = [CurrentPose(1), CurrentPose(1)+mag*cos(CurrentPose(3)+ pi - pi/8)];
    angy2 = [CurrentPose(2), CurrentPose(2)+mag*sin(CurrentPose(3)+ pi - pi/8)]; 
    ang1 = plot(angx1,angy1,'r');
    ang2 = plot(angx2,angy2,'r');
    plot(CurrentPose(1),CurrentPose(2),'-og','LineWidth',2);
    pause(1);
end
%% Alignment and Attachment Algorithm
if (distanceToGoal <= goalRadius)
    %eventually, the alignment code will go here, but for now...
    %remember that after attached,
    %start(t);
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
        %angVel = 0; %angVel is not necessary for the robots. ignore all instances
    elseif abs(tx) < distanceToGoal && ty < 0
        if tx > 0
            vLeft = 1;
            vRight = bitxor(vmax,128);
            %angVel = (bitxor(vRight,128) - vLeft) / wheelDist;
        else 
            vLeft = bitxor(vmax,128);
            vRight = 1;
            %angVel = (vRight - bitxor(vLeft,128)) / wheelDist;
        end
    else
        r = distanceToGoal^2 / (2*abs(tx));
        if tx > 0
            vLeft = vmax;
            if r-wheelDist/2<0
                vRight = bitxor(vmax*(r - wheelDist/2)/(r + wheelDist/2),128);
                %angVel = (bitxor(vRight,128) - vLeft) / wheelDist;
            else
                vRight = vmax*(r - wheelDist/2)/(r + wheelDist/2);
                %angVel = (vRight - vLeft) / wheelDist;
            end    
        else
            vRight = vmax;
            if r-wheelDist/2<0
                vLeft = bitxor(vmax*(r - wheelDist/2)/(r + wheelDist/2),128);
                %angVel = (vRight - bitxor(vLeft,128)) / wheelDist;
            else
                vLeft = vmax*(r - wheelDist/2)/(r + wheelDist/2);
                %angVel = (vRight - vLeft) / wheelDist;
            end
        end 
    end
    %Simulation update
    if simulation
        delete(circ)
        delete(ang1)
        delete(ang2)
    end
end
end

