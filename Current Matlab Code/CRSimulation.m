function CRSimulation(crpos,snpos,start)
% CRSIMULATION Charging Robot Simulation
% 
% CRSimulation( CRPOS, SNPOS, START ) shows a simulation of the charging robot and
% its assigned sensor node given the coordinates and the orientation of the charging
% robot and the sensor node. This function is for testing purposes.
% CRPOS is a vector containing the x coordinate, the y coordinate, and the
% orientation in radians of the charging robot; SNPOS is a vector
% containing the x coordinate, the y coordinate, and the orientation in
% radians of the sensor node; START is a boolean variable where 1 signifies
% the start of the tracking algorithm and 0 signifies only the
% initialization
%
% version 1.2 by M.C. Lalata at the University of Houston on 5/12/17

% Check inputs
if nargin > 3 || nargin < 2
    disp('Error: Check number of inputs.');
    return;
elseif nargin == 2
    start = 0;
end

if length(crpos) ~= 3
    disp('Error: Chargng Robot input must have an x-coordinate, a y-ccordinate, and an orientation (radians)');
    return;
end
%{
if length(snpos) ~= 3
    disp('Error: Sensor Node input must have an x-coordinate, a y-ccordinate, and an orientation (radians)');
    return;
end
%}

%% Default parameters
rad = 5;
mag = 8;
goalRadius = 0.2;

% Robot parameters
%% Charging Robots
initX = crpos(1);
initY = crpos(2);
Theta = crpos(3);

CurrentLocation = [initX,initY];
CurrentPose = [CurrentLocation Theta];

%% Sensor Node
finalX = snpos(1);
finalY = snpos(2);

Goal = [finalX,finalY]
distanceToGoal = norm(CurrentPose(1:2) - Goal);
plot(Goal(:,1),Goal(:,2),'xk','LineWidth',2,'MarkerSize',10);

if start == 0
%% Draw Robot
    hold on
    title('Charging Robots Movement Simulation');
    xlabel('X distance');
    ylabel('Y distance');
    xlim([-10 140])
    ylim([-10 130])
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
    plot(CurrentPose(1),CurrentPose(2),'-*g','LineWidth',1);

elseif distanceToGoal > goalRadius && start == 1
    pause(0.6);
    delete(circ);
    delete(ang1);
    delete(ang2);
    hold on
    x = CurrentPose(1) + rad*cos(angs);
    y = CurrentPose(2) + rad*sin(angs);
    circ = plot(x,y,'b');
    angx1 = [CurrentPose(1), CurrentPose(1)+mag*cos(CurrentPose(3)+ pi + pi/8)];
    angy1 = [CurrentPose(2), CurrentPose(2)+mag*sin(CurrentPose(3)+ pi + pi/8)];
    angx2 = [CurrentPose(1), CurrentPose(1)+mag*cos(CurrentPose(3)+ pi - pi/8)];
    angy2 = [CurrentPose(2), CurrentPose(2)+mag*sin(CurrentPose(3)+ pi - pi/8)]; 
    ang1 = plot(angx1,angy1,'r');
    ang2 = plot(angx2,angy2,'r');
    plot(CurrentPose(1),CurrentPose(2),'-*g','LineWidth',1);
end

