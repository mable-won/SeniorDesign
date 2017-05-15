%%Multiple Charging Robot Simulation
%
% This simulation works for four charging robots and four sensor nodes.
% Initial locations for charging robots and sensor nodes are randomized.
% Current location and final location are constantly moving and are
% independent. Current location is constantly finding final location for
% each iteration until current location is within a specified radius of 
% the final location. Does not take into account assignment algorithm and
% does not send data to robots.
% 
% version 1.0 by M.C. Lalata on 5/12/17

%% Initialization

initX = zeros(4,1);
initY = zeros(4,1);
Theta = zeros(4,1);
finalX = zeros(4,1);
finalY = zeros(4,1);
distanceToGoal = zeros(4,1);
vLeft = zeros(4);
vRight = zeros(4);
circ = zeros(1,4);
ang1 = zeros(1,4);
ang2 = zeros(1,4);

%% Parameters
rad = 5;
mag = 8;
wheelDist = 5;
goalRadius = 0.2;
throttle = 127;
slowdown = 0.6;
counter = 0;
t = 0.04; %time driving at these velocities
vmax = throttle*slowdown;
%% Charging Robots
%initial location of four charging robots. randomized. all of them have one orientation
for init=1:4
    initX(init) = 0;
    initY(init) = (rad*init) + 2*rad*(init-1);
    Theta(init) = 0;
end
CurrentLocation = [initX,initY];
CurrentPose = [CurrentLocation Theta];
%% Sensor Node
%initial location of four sensor nodes. randomized. for this program orientation of sensor nodes not important
for fin = 1:4
    finalX(fin) = 5 + 85*rand;
    finalY(fin) = 5 + 85*rand;
end
Goal = [finalX,finalY];
% calculates distance between charging robot and sensor node. 
% charging robot and sensor node are assigned to each other such that charging robot 1 -> sensor node 1, 
% charging robot 2 -> sensor node 2,...
for dist=1:4
    distanceToGoal(dist) = norm(CurrentLocation(dist,:) - Goal(dist,:));
end
plot(Goal(:,1),Goal(:,2),'xk','LineWidth',2,'MarkerSize',10);
%% Draw Robot
hold on
title('Charging Robots Movement Simulation');
xlabel('X distance');
ylabel('Y distance');
xlim([-10 150])
ylim([-10 100])
angs = 0:pi/10:2*pi;
for sim=1:4
    x = CurrentPose(sim,1) + rad*cos(angs);
    y = CurrentPose(sim,2) + rad*sin(angs);
    circ(sim) = plot(x,y,'b');
    angx1 = [CurrentPose(sim,1), CurrentPose(sim,1)+mag*cos(CurrentPose(sim,3)+ pi + pi/8)];
    angy1 = [CurrentPose(sim,2), CurrentPose(sim,2)+mag*sin(CurrentPose(sim,3)+ pi + pi/8)];
    angx2 = [CurrentPose(sim,1), CurrentPose(sim,1)+mag*cos(CurrentPose(sim,3)+ pi - pi/8)];
    angy2 = [CurrentPose(sim,2), CurrentPose(sim,2)+mag*sin(CurrentPose(sim,3)+ pi - pi/8)]; 
    ang1(sim) = plot(angx1,angy1,'r');
    ang2(sim) = plot(angx2,angy2,'r');
    plot(CurrentPose(sim,1),CurrentPose(sim,2),'-*g','LineWidth',1);
end








while ((distanceToGoal(1) > goalRadius) || (distanceToGoal(2) > goalRadius) || (distanceToGoal(3) > goalRadius) || (distanceToGoal(4) > goalRadius)) 
    %% pseudocode
    % distance to goal already calculated
    % calculate required curvature of charging robot to go to Goal
    % charging robot go to location
    % look at updated Goal
    % if Goal is not updated, continue moving towards Goal
    % if Goal is changed, compute distanceToGoal
    counter = counter + 1;
    for bot=1:4
        dx = Goal(bot,1) - CurrentPose(bot,1);
        dy = Goal(bot,2) - CurrentPose(bot,2);
        theta = -(CurrentPose(bot,3) - pi/2);
        tx = dx*cos(theta) - dy*sin(theta);
        ty = dx*sin(theta) + dy*cos(theta);
        if abs(tx) < 0.005*distanceToGoal(bot) && ty > 0
            vRight(bot) = vmax;
            vLeft(bot) = vRight(bot);
            %angVel = 0; %angVel is not necessary for the robots. ignore all instances
        elseif abs(tx) < distanceToGoal(bot) && ty < 0
            if tx > 0
                vLeft(bot) = 1;
                vRight(bot) = -vmax;
                %angVel = (vRight - vLeft) / wheelDist;
            else 
                vLeft(bot) = -vmax;
                vRight(bot) = 1;
                %angVel = (vRight - vLeft) / wheelDist;
            end
        else
            r = distanceToGoal(bot)^2 / (2*abs(tx));
            if tx > 0
                vLeft(bot) = vmax;
                vRight(bot) = vmax*(r - wheelDist/2)/(r + wheelDist/2);
                %angVel = (vRight - vLeft) / wheelDist
            else
                vLeft(bot) = vmax*(r - wheelDist/2)/(r + wheelDist/2);
                vRight(bot) = vmax;
                %angVel = (vRight - vLeft) / wheelDist;
            end
        
        end
        % update location calculation since no camera input
        vSum = vLeft(bot)+vRight(bot);
        vDiff = vRight(bot)-vLeft(bot);
        Theta(bot) = CurrentPose(bot,3) + vDiff*t/wheelDist;
        if(vDiff==0)
            CurrentLocation(bot,1) = vRight(bot)*t*cos(CurrentPose(bot,3)) + CurrentPose(bot,1);
            CurrentLocation(bot,2) = vLeft(bot)*t*sin(CurrentPose(bot,3)) + CurrentPose(bot,2);
        else
            CurrentLocation(bot,1) = CurrentLocation(bot,1) + wheelDist*vSum/(2*vDiff)*(sin((vDiff)*t/wheelDist + CurrentPose(bot,3)) - sin(CurrentPose(bot,3)));
            CurrentLocation(bot,2) = CurrentLocation(bot,2) - wheelDist*vSum/(2*vDiff)*(cos((vDiff)*t/wheelDist + CurrentPose(bot,3)) - cos(CurrentPose(bot,3)));
        end
            
        
        
    end
    
    % should check location of Goal somewhere here. If Goal is the same
    % then continue. If not compute new distanceToGoal
    % The if statements is just a crude method to get Goal to move in different
    % locations. For our purposes, this chunk of code will be replace by a
    % shorter code that gets information about the coordinates of the
    % sensor nodes from the camera
    
    
    CurrentPose = [CurrentLocation Theta];
    pause(0.6)
    delete(circ)
    delete(ang1)
    delete(ang2)
    hold on


    for sim=1:4
        x = CurrentPose(sim,1) + rad*cos(angs);
        y = CurrentPose(sim,2) + rad*sin(angs);
        circ(sim) = plot(x,y,'b');
        angx1 = [CurrentPose(sim,1), CurrentPose(sim,1)+mag*cos(CurrentPose(sim,3)+ pi + pi/8)];
        angy1 = [CurrentPose(sim,2), CurrentPose(sim,2)+mag*sin(CurrentPose(sim,3)+ pi + pi/8)];
        angx2 = [CurrentPose(sim,1), CurrentPose(sim,1)+mag*cos(CurrentPose(sim,3)+ pi - pi/8)];
        angy2 = [CurrentPose(sim,2), CurrentPose(sim,2)+mag*sin(CurrentPose(sim,3)+ pi - pi/8)]; 
        ang1(sim) = plot(angx1,angy1,'r');
        ang2(sim) = plot(angx2,angy2,'r');
        plot(CurrentPose(sim,1),CurrentPose(sim,2),'-*g','LineWidth',2);
    end
    
    
        

      
       
     
        if counter == 10 
            for new = 1:4
                finalX(new) = 5 + 85*rand;
                finalY(new) = 5 + 85*rand;
            end
            Goal = [finalX, finalY];
            plot(Goal(:,1),Goal(:,2),'mx','LineWidth',3,'MarkerSize',18);
        end
        
        
       for len=1:4
            distanceToGoal(len) = norm(CurrentPose(len,1:2) - Goal(len,:)); % this is necessary
       end
   
    
end


