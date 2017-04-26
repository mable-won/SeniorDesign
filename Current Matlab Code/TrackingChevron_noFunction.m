global outVector;
%pass = 0;
%counter = 0;
CRNumber = 4;
SNNumber = 5;
while(1)
    %---------Setting up webcam--------------%
    vid=webcam(2); %connect to webcam
    vid.Resolution = '1024x768';
    samp1 = snapshot(vid); %take a photo
    pass = 1;

    sampInfo = regionprops(samp1); %find connected components
    
    if (isempty(sampInfo)) %if nothing is seen, take another snapshot

    figure(1),imshow(samp1);

    else
        %------Filtering image-------------%
        BW1 = snapshot(vid);
        BW2=im2bw(BW1,.8); %#ok<IM2BW> %contrast
        BW = bwareaopen(BW2,256); %deleting small images

        figure(2),imshow(BW);
        
        %-------Gathering image information---------%
        s = regionprops(BW,'centroid','area','perimeter');
        centroids = cat(1, s.Centroid);
        
        %------Drawing----------%
        if(size(centroids))
            hold on
            plot(centroids(:,1),centroids(:,2), 'b*')

             bb = regionprops(BW,'BoundingBox');
             bboxes = cat(1, bb.BoundingBox);
             for i = 1:size(bboxes,1)
                rectangle('Position',bboxes(i,:),'EdgeColor','r')
             end

             ch = regionprops(BW,'ConvexHull','centroid','orientation','MajorAxisLength','perimeter');

             avgAxisLength=0; 
             for i = 1:numel(ch)
                 avgAxisLength=avgAxisLength+ch(i).MajorAxisLength;
             end
             avgAxisLength=avgAxisLength/numel(ch);
             halfLen=avgAxisLength*2/5;

             %----Calculating the orientation of the chevron-----------%
             %Calcuation is made by sampling points on opposite sides of
             %the centroid along the 

             leftEdge=zeros(numel(ch),2);
             rightEdge=zeros(numel(ch),2);
             for i = 1:numel(ch)
                 centOrient=(ch(i).Orientation)*pi/180;
                 centX=ch(i).Centroid(1);
                 centY=ch(i).Centroid(2);
                 if centOrient>0
                     rightEdge(i,1)=centX+halfLen*abs(cos(centOrient));
                     rightEdge(i,2)=centY-halfLen*abs(sin(centOrient));
                     leftEdge(i,1)=centX-halfLen*abs(cos(centOrient));
                     leftEdge(i,2)=centY+halfLen*abs(sin(centOrient));
                 else 
                     rightEdge(i,1)=centX+halfLen*abs(cos(centOrient));
                     rightEdge(i,2)=centY+halfLen*abs(sin(centOrient));
                     leftEdge(i,1)=centX-halfLen*abs(cos(centOrient));
                     leftEdge(i,2)=centY-halfLen*abs(sin(centOrient));
                 end
             end
             
             %%------compare density of ends
             directionPoint=zeros(12,2);
              for i = 1:numel(ch)
                  leftIntY=round(leftEdge(i,2));
                  leftIntX=round(leftEdge(i,1));   
                  if(BW(leftIntY,leftIntX)==1)
                      directionPoint(i,1)=leftEdge(i,1);
                      directionPoint(i,2)=leftEdge(i,2);
                  else 
                      directionPoint(i,1)=rightEdge(i,1);
                      directionPoint(i,2)=rightEdge(i,2); 
                  end
              end

             plot(directionPoint(:,1),directionPoint(:,2), 'r*');

             Orient=zeros(12,i);
             %---calculate orientation
             for i = 1:numel(ch)
                 Xdiff= directionPoint(i,1)-ch(i).Centroid(1);
                 Ydiff=ch(i).Centroid(2)-directionPoint(i,2);
                 Orient(i)=mod(atan2(Ydiff,Xdiff)*180/pi,360);

             end

             outgoing=zeros(CRNumber+SNNumber,4);

             %outVector = zeros(1,numel(ch)*3);

             for i = 1:numel(ch)
                 data = ch(i).ConvexHull;
                 cent=ch(i).Centroid;
                 orient=Orient(i);
                line(data(:,1),data(:,2),'color','g')
                roip=roipoly(BW, data(:,1),data(:,2));
                roip2=roip&BW;
                roip2_inv=imcomplement(roip2); 
                bw3=bwconncomp(roip2_inv,4);
                numDots = bw3.NumObjects-1;
                text(mean(data(:,1)),mean(data(:,2)),num2str(numDots),'color','b','fontsize',16);   
                %convert x&y coordinates from pixels to cm
                outgoing(i,:)=[numDots,cent(1)*1024/143.5,cent(2)*768/104.5,orient]; 

                %%%%%%%%%%%%%%%%%%%%%
                    hlen = ch(i).MajorAxisLength/2;
                    xCentre = ch(i).Centroid(1);
                    yCentre = ch(i).Centroid(2);
                    cosOrient = cosd(ch(i).Orientation);
                    sinOrient = sind(ch(i).Orientation);
                    xcoords = xCentre + hlen * [cosOrient -cosOrient];
                    ycoords = yCentre + hlen * [-sinOrient sinOrient];
                    line(xcoords, ycoords);
                %%%%%%%%%%%%%%%%%%%%%

             end

            outVector=reshape(outgoing.',1,[]);
            %for carID = 1:CRNumber+SNNumber
            %    for i = 1:CRNumber+SNNumber
            %        if outVector(i*4-3) == carID
            %           break;
            %        elseif i==CRNumber+SNNumber
            %            pass=0;
            %        end
            %    end
            %end
            hold off
    %         flushdata(vid);

        end
       delete(vid);
       %counter = counter+1;
    end
end
