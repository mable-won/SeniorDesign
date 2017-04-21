%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Global Variables


    vid=webcam(1); %connect to webcam
    samp1 = snapshot(vid); %take a photo
    outputArray=TrackingChevron_RealTime(samp1);
    
   % delete(vid);