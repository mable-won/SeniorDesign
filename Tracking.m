imaqreset;
close all
clear all
clc

% Capture the video frames using the videoinput function

% You have to replace the resolution & your installed adaptor name.
vid=webcam(2);
%%vid = videoinput('winvideo', 2, 'YUY2_320x240');

% Set the properties of the video object
%%set(vid, 'FramesPerTrigger', Inf);
%%set(vid, 'ReturnedColorspace', 'rgb')
%%vid.FrameGrabInterval = 1;

%start the video aquisition here
%%start(vid)

% Set a loop that runs until interrupter externally (ctrl+c)
while(1)
    
    % Get the snapshot of the current frame
    data = snapshot(vid);
    
    % Now to track red objects in real time
    % we have to subtract the red component 
    % from the grayscale image to extract the red components in the image.
    diff_im = imsubtract(data(:,:,1), rgb2gray(data));
    %Use a median filter to filter out noise
    diff_im = medfilt2(diff_im, [3 3]);
    
%     figure(5),imshow(diff_im);
    % Convert the resulting grayscale image into a binary image.
    diff_im = im2bw(diff_im,0.10);
    
    % Remove all those pixels less than 300px
    diff_im = bwareaopen(diff_im,300);
    
    % Label all the connected components in the image.
    bw = bwlabel(diff_im, 8);
    
    % Here we do the image blob analysis.
    % We get a set of properties for each labeled region.
    stats = regionprops(bw, 'BoundingBox', 'Centroid');
    
    %---------------------------
    % Now to track blue objects in real time
    % we have to subtract the blue component 
    % from the grayscale image to extract the red components in the image.
    diff_im_b = imsubtract(data(:,:,3), rgb2gray(data));
    %Use a median filter to filter out noise
    diff_im_b = medfilt2(diff_im_b, [3 3]);
    
%     figure(5),imshow(diff_im);
    % Convert the resulting grayscale image into a binary image.
    diff_im_b = im2bw(diff_im_b,0.10);
    
    % Remove all those pixels less than 300px
    diff_im_b = bwareaopen(diff_im_b,300);
    
    % Label all the connected components in the image.
    bw_b = bwlabel(diff_im_b, 8);
    
    % Here we do the image blob analysis.
    % We get a set of properties for each labeled region.
    stats = regionprops(bw, 'BoundingBox', 'Centroid');
    stats_b = regionprops(bw_b, 'BoundingBox', 'Centroid');
    %---------------------------
    
    % Display the image
    figure(7),imshow(data)
    
    hold on
    
    %This is a loop to bound the red objects in a rectangular box.
    for object = 1:length(stats)
        bb = stats(object).BoundingBox;
        bc = stats(object).Centroid;
        rectangle('Position',bb,'EdgeColor','r','LineWidth',2)
        plot(bc(1),bc(2), '-m+')
        a=text(bc(1)+15,bc(2), strcat('X: ', num2str(round(bc(1))), '    Y: ', num2str(round(bc(2)))));
        set(a, 'FontName', 'Arial', 'FontWeight', 'bold', 'FontSize', 12, 'Color', 'yellow');
    end
    
        %This is a loop to bound the blue objects in a rectangular box.
    for object = 1:length(stats_b)
        bb_b = stats_b(object).BoundingBox;
        bc_b = stats_b(object).Centroid;
        rectangle('Position',bb_b,'EdgeColor','r','LineWidth',2)
        plot(bc_b(1),bc_b(2), '-m+')
        a_b=text(bc_b(1)+15,bc_b(2), strcat('X: ', num2str(round(bc_b(1))), '    Y: ', num2str(round(bc_b(2)))));
        set(a_b, 'FontName', 'Arial', 'FontWeight', 'bold', 'FontSize', 12, 'Color', 'yellow');
    end
    
    hold off
    %%flushdata(vid);
end
% Both the loops end here.

% Stop the video aquisition.
%%stop(vid);

% Flush all the image data stored in the memory buffer.
flushdata(vid);

% Clear all variables
clear all
sprintf('%s','That was all about Image tracking, Guess that was pretty easy :) ')
