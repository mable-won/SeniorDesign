function UpdateLocation(vLeft, vRight, carID, t)
% UPDATELOCATION Update Charging Robot Location
%
% UpdateLocation( VLEFT, VRIGHT, CARID, T ) updates the global variable
% outVector without using camera input data.
% VLEFT is the wheel thrust of the car robot's left wheel; VRIGHT is the
% wheel thrust of the car robot's right wheel; CARID is the numerical ID 
% of the charging robot; and T is an optional experimental time variable.
% It could be user-defined, but the program also has a default value.
% wheelDist is the distance between the left and right wheels. 
%
% version 1.0 by M.C. Lalata and R. Dunn at the University of Houston on
% 4/06/17

global outVector
wheelDist = 5;
if nargin > 4 || nargin < 3
    disp('Error: Check number of inputs.');
    return;
elseif nargin == 3
    t = 0.1;
end

for i=1:12 %used if camera array does not have robots in numerical order
    if outVector(i*4-3)==carID
        break;
    elseif i==12
        disp('Initial data not found.');
        return;
    end
end
if vLeft > 127
    vLeft = -bitxor(vLeft,128);
end
if vRight > 127
    vRight = -bitxor(vRight,128);
end
initX = outVector(i*4-2);
initY = outVector(i*4-1);
Theta = outVector(i*4);
CurrentLocation = [initX,initY];
CurrentPose = [CurrentLocation Theta];
vSum = vLeft + vRight;
vDiff = vRight - vLeft;
angle = CurrentPose(3) + vDiff*t/wheelDist;
if(vDiff==0)
   xpos = vRight*t*cos(CurrentPose(3)) + CurrentPose(1);
   ypos = vLeft*t*sin(CurrentPose(3)) + CurrentPose(2);
else
   xpos = CurrentPose(1) + wheelDist*vSum/(2*vDiff)*(sin((vDiff)*t/wheelDist + CurrentPose(3)) - sin(CurrentPose(3)));
   ypos = CurrentPose(2) - wheelDist*vSum/(2*vDiff)*(cos((vDiff)*t/wheelDist + CurrentPose(3)) - cos(CurrentPose(3)));
end

outVector(i*4-2) = xpos;
outVector(i*4-1) = ypos;
outVector(i*4) = angle;
end