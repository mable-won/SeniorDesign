%Robot_Main collect location and orientation information from the webcam
%and send specific instructions for each robot to move

%reference RegionCode.m

close all
clear all
%set up camera
cam = webcam(2);

while(1)
arenaImage = snapshot(cam); %image of the arena
imshow(arenaImage);
[centers, radii, metric]=imfindcircles(arenaImage,[45 55]);
pause(2);
end
%d=imdistline;
%arena = imcrop(arenaImage,[x y z w]); %crop the image

%set up threholds


%assign different color to different robot


%instruct specific robot to move forward