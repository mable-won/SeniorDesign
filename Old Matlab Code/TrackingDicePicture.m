BW2 = imread('ovalobjects.png');
BW = im2bw(BW2,0.8);


 BW=bwareaopen(BW,500);
figure, imshow(BW);

s = regionprops(BW,'centroid','area','orientation');
centroids = cat(1, s.Centroid);

hold on
plot(centroids(:,1),centroids(:,2), 'b*')


 bb = regionprops(BW,'BoundingBox');
 bboxes = cat(1, bb.BoundingBox);
 for i = 1:size(bboxes,1)
    rectangle('Position',bboxes(i,:),'EdgeColor','r')
 end
 
 ch = regionprops(BW,'ConvexHull','centroid','orientation','MajorAxisLength');

 outgoing=zeros(numel(ch),1);
 %outVector = zeros(1,numel(ch)*3);
  
 for i = 1:numel(ch)
     data = ch(i).ConvexHull;
     cent=ch(i).Centroid;
     orient=ch(i).Orientation;
    line(data(:,1),data(:,2),'color','g')
    roip=roipoly(BW, data(:,1),data(:,2));
    roip2=roip&BW;
    roip2_inv=imcomplement(roip2); 
    bw3=bwconncomp(roip2_inv);
    numDots = bw3.NumObjects-1;
    text(mean(data(:,1)),mean(data(:,2)),num2str(numDots),'color','b','fontsize',16);   
%     outgoing(i,:)=[numDots,cent(1),cent(2),orient];
    
    %%%%%%%%%%%%%%%%%%%%%
%         hlen = ch(i).MajorAxisLength/2;
%         xCentre = ch(i).Centroid(1);
%         yCentre = ch(i).Centroid(2);
%         cosOrient = cosd(ch(i).Orientation);
%         sinOrient = sind(ch(i).Orientation);
%         xcoords = xCentre + hlen * [cosOrient -cosOrient];
%         ycoords = yCentre + hlen * [-sinOrient sinOrient];
%         line(xcoords, ycoords);
    %%%%%%%%%%%%%%%%%%%%%
    
    
    
    
 end
 
 %assign robot->node
 %find center point(CP) of the two centroids 
%  X2=outgoing(1,2);
%  Y2=outgoing(1,3);
%  X5=outgoing(2,2);
%  Y5=outgoing(2,3);
%  CPX25=(X2+X5)/2;
%  CPY25=(Y2+Y5)/2;
%  CP25=[CPX25,CPY25]; %CP of robots 2 & 5
%  plot(CPX25,CPY25, 'w*');
%  plot([X2,CPX25],[Y2,CPY25],'w');
%  %calculate angle between each centroid and CP
% 
%  diffYCP25 = CPY25-Y2;
%  diffXCP25 = CPX25-X2;
%  angle25=atan2(diffYCP25,diffXCP25)*180;
%  
 
 %compare that angle with the robot angle
 %rotate robot CW until the angles are the same +- some error
 
 
 
 
 
outVector=reshape(outgoing.',1,[]);
 
hold off

 