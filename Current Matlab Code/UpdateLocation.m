function UpdateLocation(vLeft, vRight, t)
% UPDATELOCATION Update Charging Robot Location without Camera Input
%
% UpdateLocation( VLEFT, VRIGHT, T ) updates global variable outVector without
% using camera input data.
% VLEFT is the wheel thrust of the car robot's left wheel; VRIGHT is the
% wheel thrust of the car robot's right wheel; T is an experimental time
% variable. It could be user-defined, but the program also has a default
% value.
% wheelDist is the distance between the left and right wheels. 
%
global outVector

if nargin > 3 || nargin < 2
    disp('Error: Check number of inputs.');
    return;
elseif nargin == 2
    t = 0.5;
end

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
wheelDist = 3;
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