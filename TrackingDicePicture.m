BW2 = imread('Dice_1.png');
BW = im2bw(BW2,0.5);


BW=bwareaopen(BW,1000);
figure, imshow(BW);

s = regionprops(BW,'centroid','area');
centroids = cat(1, s.Centroid);

hold on
plot(centroids(:,1),centroids(:,2), 'b*')
hold off

 bb = regionprops(BW,'BoundingBox');
 bboxes = cat(1, bb.BoundingBox);
 for i = 1:size(bboxes,1)
    rectangle('Position',bboxes(i,:),'EdgeColor','r')
 end
 
 ch = regionprops(BW,'ConvexHull','centroid');

 outgoing=zeros(numel(ch),3);
 %outVector = zeros(1,numel(ch)*3);
  
 for i = 1:numel(ch)
     data = ch(i).ConvexHull;
     cent=ch(i).Centroid;
    line(data(:,1),data(:,2),'color','g')
    % this convex hull outlines one dice.  You need to run bwconncomp(BW)
    % inside this convex hull, count the dots , and write text
    roip=roipoly(BW, data(:,1),data(:,2));
    roip2=roip&BW;
    %figure
    %title(i);
    %imshow(roip2);
    roip2_inv=imcomplement(roip2); 
    %YESSSS next step: numobjects=numobjects-1;
    bw3=bwconncomp(roip2_inv);
    numDots = bw3.NumObjects-1;
    text(mean(data(:,1)),mean(data(:,2)),num2str(numDots),'color','b','fontsize',16);   
    outgoing(i,1)=numDots;
    outgoing(i,2)=cent(1);
    outgoing(i,3)=cent(2);
 end
 
outVector=reshape(outgoing.',1,[]);
 

 