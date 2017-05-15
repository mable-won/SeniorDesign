% initial location is at (0,0). current location and final location are 
% constantly moving and are independent
% current location is constantly following final location until current 
% location is within a specified radius of of the final location

%% Initialization

initX = zeros(4,1);
initY = zeros(4,1);
Theta = zeros(4,1);
finalX = zeros(4,1);
finalY = zeros(4,1);
distanceToGoal = zeros(4,1);

%% Default parameters
rad = 5;
wheelDist = 5;
goalRadius = 20; % cm
throttle = 127; % maximum velocity in cm/s
slowdown = 0.6; % safety factor
t = 0.07; %time driving at these velocities
vmax = throttle*slowdown;
counter = 0;

%% Charging Robot
for init=1:4
    initX(init) = 0;
    initY(init) = (rad*init) + 2*rad*(init-1);
    Theta(init) = 0;
end

CurrentLocation = [initX,initY];
CurrentPose = [CurrentLocation Theta];
%% Sensor Node
for fin = 1:4
    finalX(fin) = 5 + 85*rand;
    finalY(fin) = 5 + 85*rand;
end
Goal = [finalX,finalY]
for dist=1:4
    distanceToGoal(dist) = norm(CurrentLocation(dist,:) - Goal(dist,:));
end
CRSimulation(CurrentPose(1,:),Goal(1,:))
CRSimulation(CurrentPose(2,:),Goal(2,:))
CRSimulation(CurrentPose(3,:),Goal(3,:))
CRSimulation(CurrentPose(4,:),Goal(4,:))

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
    
    
    CurrentPose = [CurrentLocation Theta]
    CRSimulation(CurrentPose(1,:),Goal(1,:),1)
    CRSimulation(CurrentPose(2,:),Goal(2,:),1)
    CRSimulation(CurrentPose(3,:),Goal(3,:),1)
    CRSimulation(CurrentPose(4,:),Goal(4,:),1)
    if counter == 10 
        for new = 1:4
            finalX(new) = 5 + 85*rand;
            finalY(new) = 5 + 85*rand;
        end
        Goal = [finalX, finalY];
        plot(Goal(:,1),Goal(:,2),'mx','LineWidth',3,'MarkerSize',18);
    end
        
    for len=1:4
        distanceToGoal(len) = norm(CurrentPose(len,1:2) - Goal(len,:)) % this is necessary
    end
   
    
end