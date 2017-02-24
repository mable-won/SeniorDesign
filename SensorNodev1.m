clear 
clc
i = 1;
%This is the Random Walk with Drift Model
colorMap = hsv(8); % just to make the 8 paths have different colors in the graph
rad = 0.5; % for collision 
randwalk=10*rand(8,2); %8 sensor nodes starting coordinate is at random locations between [0 10] on all directions
delay=1.0; %delay in loop time in s (as a float value)
%preallocation
drift=zeros(8,2);
line=zeros(8,1);
randwalk_new=zeros(8,2);
randwalk_prev=zeros(8,2);
orientation=zeros(8,1);
orientation_new=zeros(8,1);
packages=zeros(8,2);
voltages=zeros(8,1);
rot_package=zeros(1,18);
mov_package=zeros(1,18);
stop_package=zeros(1,18); stop_package(1)=67; stop_package(18)=77;
send_package=zeros(1,18); send_package(1)=86; send_package(10)=63;
volt_package=zeros(1,10); volt_package(1)=86; volt_package(10)=63;

%open Serial port
s=setupSerial('COM16');

%keep commented until all cars in use
%[~]=sendData(s,send_package);
%for car=1:8
%    [~,packages(car,:)]=receiveData(s,2);
%    voltages(car,1)=bitshift(packages(car,1),8)+packages(car,2);
%end
%[~,index]=sort(voltages);
%for car=1:8
%    volt_package(car+1)=index;
%end
%[~]=sendData(s,volt_package,'000C'); %send to COORDINATOR 2

% for each time step, a new random walk path will be generated for all 8
% sensor nodes
for t=1:20 
    compare=rand(8,2); % makes 8 random arrays filled with values ranging from 0 to 1
    for i=1:8
        drift(i,:)=compare(i,:)>[0.2 0.5]; %fills drift array with 1s and 0s 
    end
    % based on the random drift generated, the 8 sensor nodes will each pick a
    % random direction each time step
    for k=1:8
        if drift(k,:)==1
            randwalk_new(k,:) = randwalk(k,:) + [2 0];
        elseif drift(k,1)==1
            randwalk_new(k,:) = randwalk(k,:) + [0 2];
        elseif drift(k,2)==1
            randwalk_new(k,:) = randwalk(k,:) + [-2 0];
        else
            randwalk_new(k,:) = randwalk(k,:) + [0 -2];
        end
        %this code is to ensure that randwalk does not go beyond 250 units
        %in either direction
        if randwalk_new(k,1)>250
            randwalk_new(k,:) = randwalk_new(k,:) + [-2 0];
        end
        if randwalk_new(k,2)>250
            randwalk_new(k,:) = randwalk_new(k,:) + [0 -2];
        end
    end
    
    % finding collisions algorithm. still needs work
    %d=pdist2(randwalk_new,randwalk_new);
    %d(d==0) = 100; %make diagonal huge
    %[ix,iy] = find(d<rad);%repmat(rad,1,numel(ix))
    %if isempty(ix)== 0
    %    dist(i,:,:) = randwalk_new(ix,:);
    %    i = i + 1;
    %    fprintf('collision is in = %d.\n',randwalk_new(ix,:));
    %    viscircles(randwalk_new(ix,:),repmat(rad,numel(ix),1),'Color','k')
    %end
   
    hold on
    set(gcf, 'units','normalized','outerposition',[0 0 1 1]);
    grid on
    title('Random Walk with Drift model for Sensor Nodes', 'FontSize', 15);
    xlabel('X distance', 'FontSize', 15);
    ylabel('Y distance', 'FontSize', 15);
    
    for p=1:8
        line(p)=plot([randwalk(p,1) randwalk_new(p,1)],[randwalk(p,2) randwalk_new(p,2)],'-o','Color',colorMap(p,:),'LineWidth', 4); 
        %line(p)=scatter([randwalk(p,1) randwalk_new(p,1)],[randwalk(p,2) randwalk_new(p,2)],20,colorMap(p,:),'fill');   
    end
    drawnow
    if t==1 %initial case: no prev orientation values, so move everything straight
        mov_package=[67 100 100 100 100 100 100 100 100 100 100 100 100 100 100 100 100 77];
        [~]=sendData(s,mov_package);
        pause(0.357143); %move 5 cm at 14cm/s
        [~]=sendData(s,stop_package);
        pause(delay);
    else
    for car=1:8
        %calculate original orientation
        if randwalk_prev(car,1)==randwalk(car,1)
            if randwalk_prev(car,2)==randwalk(car,2)+2
                orientation(car,1)=90;%up
            else
                orientation(car,1)=-90; %down
            end
        elseif randwalk_prev(car,1)==randwalk(car,1)+2
            orientation(car,1)=0; %right
        else
            orientation(car,1)=180; %left
        end
        %calculate final orientation
        if randwalk(car,1)==randwalk_new(car,1)
            if randwalk(car,2)==randwalk_new(car,2)+2
                orientation_new(car,1)=90;%up
            else
                orientation_new(car,1)=-90; %down
            end
        elseif randwalk(car,1)==randwalk_new(car,1)+2
            orientation_new(car,1)=0; %right
        else
            orientation_new(car,1)=180; %left
        end
        %calculate rotation step & move step (subject to calibration)
        if orientation_new(car,1)-orientation(car,1)==0
            rot_package(1,car*2)=0; %forward
            rot_package(1,car*2+1)=0;
            mov_package(1,car*2)=100;
            mov_package(1,car*2+1)=100;
        elseif orientation_new(car,1)-orientation(car,1)==180||orientation_new(car,1)-orientation(car,1)==-180
            rot_package(1,car*2)=0; %backward
            rot_package(1,car*2+1)=0;
            mov_package(1,car*2)=228;
            mov_package(1,car*2+1)=228;
        elseif orientation_new(car,1)-orientation(car,1)==90||orientation_new(car,1)-orientation(car,1)==-270
            rot_package(1,car*2)=100; %right turn
            rot_package(1,car*2+1)=228;
            mov_package(1,car*2)=100;
            mov_package(1,car*2+1)=100;
        else
            rot_package(1,car*2)=228; %left turn
            rot_package(1,car*2+1)=100;
            mov_package(1,car*2)=100;
            mov_package(1,car*2+1)=100;
        end
    end
    %create header and footer for packages
    rot_package(1)='C'; mov_package(1)='C';
    rot_package(18)='M'; mov_package(18)='M';
    %send packages
    [~]=sendData(s,rot_package);
    pause(0.2807005); %time for 90 degree turn
    [~]=sendData(s,stop_package);
    pause(0.020);
    [~]=sendData(s,mov_package);
    pause(0.357143); %move 5 cm at 14cm/s
    [~]=sendData(s,stop_package);
    pause(delay);
    end
    %UPDATE the new position
    randwalk_prev=randwalk;
    randwalk=randwalk_new;
end
%close port
fclose(s);
delete(s);

