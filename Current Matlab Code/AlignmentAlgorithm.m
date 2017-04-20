clear
clearvars
clc
tic
CR_Pose = [3 2 2*pi/4]; %initial location of charging robot: [xpos ypos orientation]
SN_Pose = [20 38 2*pi/3]; %location of sensor node. for this code we assume it's stationary
hold on
%% this is just drawing the charging robot and sensor node. for simulation purposes
mag = 7;
CR_angx1 = [CR_Pose(1), CR_Pose(1)+mag*cos(CR_Pose(3)+ pi + pi/8)];
CR_angy1 = [CR_Pose(2), CR_Pose(2)+mag*sin(CR_Pose(3)+ pi + pi/8)];
CR_angx2 = [CR_Pose(1), CR_Pose(1)+mag*cos(CR_Pose(3)+ pi - pi/8)];
CR_angy2 = [CR_Pose(2), CR_Pose(2)+mag*sin(CR_Pose(3)+ pi - pi/8)];
CR_ang1 = plot(CR_angx1,CR_angy1,'r');
CR_ang2 = plot(CR_angx2,CR_angy2,'r');
SN_angx1 = [SN_Pose(1), SN_Pose(1)+mag*cos(SN_Pose(3)+ pi + pi/8)];
SN_angy1 = [SN_Pose(2), SN_Pose(2)+mag*sin(SN_Pose(3)+ pi + pi/8)];
SN_angx2 = [SN_Pose(1), SN_Pose(1)+mag*cos(SN_Pose(3)+ pi - pi/8)];
SN_angy2 = [SN_Pose(2), SN_Pose(2)+mag*sin(SN_Pose(3)+ pi - pi/8)];
SN_ang1 = plot(SN_angx1,SN_angy1,'b');
SN_ang2 = plot(SN_angx2,SN_angy2,'b');
xlim([-10 55])
ylim([-20 55])

%% Parameters
rad = 3;
%mag = 5;
wheelDist = 3;
mult = 2;
goalRadius = 0.1;
throttle = 8;
slowdown = 0.5;
t = 0.5; %time driving at these velocities
vmax = throttle*slowdown;
bodRad = sqrt((CR_angx1(2) - CR_angx2(2))^2 + ((CR_angy1(2) - CR_angy2(2))^2))+3 ;
collRad = bodRad+4;
botCenter = [SN_Pose(1) + (mag/2)*cos(SN_Pose(3) + pi), SN_Pose(2) + (mag/2)*sin(SN_Pose(3) + pi)];
%% Charging Robot
CurrentPose = CR_Pose;
%% Sensor Node
Goal = SN_Pose(1:2);
plot(Goal(1),Goal(2),'xk','LineWidth',2,'MarkerSize',10);
%% plot center of robot and head of robot
angs = 0:pi/10:2*pi;
x = botCenter(1) + collRad*cos(angs);
y = botCenter(2) + collRad*sin(angs);
circArray = [x' y'];
circ=plot(x,y,'k'); 
plot(botCenter(1),botCenter(2),'m*', 'MarkerSize',6);
%% calculates where charging robot needs to go to in order to have charging robot and sensor node face to face
dist=zeros(21);
for cc = 1:21
    dist(cc) = norm(Goal-circArray(cc,:));
end
dist = dist';
[minDist,rowInd] = min(dist);
plot(circArray(rowInd,1),circArray(rowInd,2),'gx','MarkerSize',11);
Goal = circArray(rowInd,:);
distanceToGoal = norm(CurrentPose(1:2) - Goal);

%% calculates left and right wheel thrusts
while (distanceToGoal > goalRadius) 
    
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
                vRight = -vmax;
                %angVel = (vRight - vLeft) / wheelDist;
            else 
                vLeft = -vmax;
                vRight = 1;
                %angVel = (vRight - vLeft) / wheelDist;
            end
        else
            r = distanceToGoal^2 / (2*abs(tx));
            if tx > 0
                vLeft = vmax;
                vRight = vmax*(r - wheelDist/2)/(r + wheelDist/2);
                %angVel = (vRight - vLeft) / wheelDist
            else
                vLeft = vmax*(r - wheelDist/2)/(r + wheelDist/2);
                vRight = vmax;
                %angVel = (vRight - vLeft) / wheelDist;
            end
        
        end
        
        vSum = vLeft+vRight;
        vDiff = vRight-vLeft;
        Theta = CurrentPose(3) + vDiff*t/wheelDist;
        if(vDiff==0)
            CurrentLocation(1) = vRight*t*cos(CurrentPose(3)) + CurrentPose(1);
            CurrentLocation(2) = vLeft*t*sin(CurrentPose(3)) + CurrentPose(2);
        else
            CurrentLocation(1) = CurrentPose(1) + wheelDist*vSum/(2*vDiff)*(sin((vDiff)*t/wheelDist + CurrentPose(3)) - sin(CurrentPose(3)));
            CurrentLocation(2) = CurrentPose(2) - wheelDist*vSum/(2*vDiff)*(cos((vDiff)*t/wheelDist + CurrentPose(3)) - cos(CurrentPose(3)));
        end
            
        
        

    
    % should check location of Goal somewhere here. If Goal is the same
    % then continue. If not compute new distanceToGoal
    % The if statements is just a crude method to get Goal to move in different
    % locations. For our purposes, this chunk of code will be replace by a
    % shorter code that gets information about the coordinates of the
    % sensor nodes from the camera
    
    
    CurrentPose = [CurrentLocation Theta];
    pause(0.8)
    
    delete(CR_ang1)
    delete(CR_ang2)
    hold on

    CR_angx1 = [CurrentPose(1), CurrentPose(1)+mag*cos(CurrentPose(3)+ pi + pi/8)];
    CR_angy1 = [CurrentPose(2), CurrentPose(2)+mag*sin(CurrentPose(3)+ pi + pi/8)];
    CR_angx2 = [CurrentPose(1), CurrentPose(1)+mag*cos(CurrentPose(3)+ pi - pi/8)];
    CR_angy2 = [CurrentPose(2), CurrentPose(2)+mag*sin(CurrentPose(3)+ pi - pi/8)];
    CR_ang1 = plot(CR_angx1,CR_angy1,'r');
    CR_ang2 = plot(CR_angx2,CR_angy2,'r');
   
        
     
        
       
    distanceToGoal = norm(CurrentPose(1:2) - Goal);
         
    
end
%% PID controller should be here to make final orientation of charging robot equal to
% SN_Pose(3)+pi (CurrentPose(3)==SN_Pose(3)+pi
%{
if (CurrentPose(3) ~= SN_Pose(3)+pi)
    while (CurrentPose(3) < SN_Pose(3)+(pi-pi/5)) || (CurrentPose(3) > SN_Pose(3)+(pi+pi/5))
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
                        vRight = -vmax;
                        %angVel = (vRight - vLeft) / wheelDist;
                    else 
                        vLeft = -vmax;
                        vRight = 1;
                        %angVel = (vRight - vLeft) / wheelDist;
                    end
                else
                    r = distanceToGoal^2 / (2*abs(tx));
                    if tx > 0
                        vLeft = vmax;
                        vRight = vmax*(r - wheelDist/2)/(r + wheelDist/2);
                        %angVel = (vRight - vLeft) / wheelDist
                    else
                        vLeft = vmax*(r - wheelDist/2)/(r + wheelDist/2);
                        vRight = vmax;
                        %angVel = (vRight - vLeft) / wheelDist;
                    end
        
                end
        
                vSum = vLeft+vRight;
                vDiff = vRight-vLeft;
                Theta = CurrentPose(3) + vDiff*t/wheelDist;
                if(vDiff==0)
                    CurrentLocation(1) = vRight*t*cos(CurrentPose(3)) + CurrentPose(1);
                    CurrentLocation(2) = vLeft*t*sin(CurrentPose(3)) + CurrentPose(2);
                else
                    CurrentLocation(1) = CurrentPose(1) + wheelDist*vSum/(2*vDiff)*(sin((vDiff)*t/wheelDist + CurrentPose(3)) - sin(CurrentPose(3)));
                    CurrentLocation(2) = CurrentPose(2) - wheelDist*vSum/(2*vDiff)*(cos((vDiff)*t/wheelDist + CurrentPose(3)) - cos(CurrentPose(3)));
                end
            
        
        

    
    % should check location of Goal somewhere here. If Goal is the same
    % then continue. If not compute new distanceToGoal
    % The if statements is just a crude method to get Goal to move in different
    % locations. For our purposes, this chunk of code will be replace by a
    % shorter code that gets information about the coordinates of the
    % sensor nodes from the camera
    
    
            CurrentPose = [CurrentLocation Theta]
            pause(0.8)
    
            delete(CR_ang1)
            delete(CR_ang2)
            hold on

            CR_angx1 = [CurrentPose(1), CurrentPose(1)+mag*cos(CurrentPose(3)+ pi + pi/8)];
            CR_angy1 = [CurrentPose(2), CurrentPose(2)+mag*sin(CurrentPose(3)+ pi + pi/8)];
            CR_angx2 = [CurrentPose(1), CurrentPose(1)+mag*cos(CurrentPose(3)+ pi - pi/8)];
            CR_angy2 = [CurrentPose(2), CurrentPose(2)+mag*sin(CurrentPose(3)+ pi - pi/8)];
            CR_ang1 = plot(CR_angx1,CR_angy1,'r');
            CR_ang2 = plot(CR_angx2,CR_angy2,'r');

    end
end
%}


elapsedTime = toc;




