% SENSORNODE Sensor Node Code
%
% SensorNode computes and graphs the movements of 8 vehicles or sensor
% nodes based on a random walk model with drift. This code also sends the
% commands to the vehicles.
% Note: Do not run this code directly. Run the SNGUI.m file instead. With
% the GUI, you can start or stop the loop at any time, but DO NOT CLOSE THE
% GUI WITHOUT STOPPING THE SIMULATION. Doing so does not close the serial
% port and MATLAB will always complain that the serial port is being used
% by another application. If this happens, restart MATLAB and reset your
% serial port, and the problem should go away.
%
% version 2.0 by M.C. Lalata and R. Dunn at the University of Houston on
% 4/10/17

colorMap = hsv(8); % just to make the 8 paths have different colors in the graph
%% Tunable Variables
rad = 5.0; % for collision in cm
delay = 1.0; %delay in loop time in s (as a float value)
step = 5; %in cm
times = 10; %loop iterations
length = 100; %x-direction in cm
width = 100; %y-direction in cm
%sensor nodes starting coordinates spaced out along left side of the arena
randwalk = width/8*[0,0.5; 0,1.5; 0,2.5; 0,3.5; 0,4.5; 0,5.5; 0,6.5; 0,7.5];

%% Preallocation
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
s=setupSerial('COM5');

%% Poll sensor nodes for voltage, sort, then send to coordinator 2
% note: keep commented until all cars in use
%[~]=sendData(s,send_package);
%for car=1:8
%    [packages(car,:)]=receiveData(s,2);
%    voltages(car,1)=bitshift(packages(car,1),8)+packages(car,2);
%end
%[~,index]=sort(voltages);
%for car=1:8
%    volt_package(car+1)=index;
%end
%[~]=sendData(s,volt_package,'000D'); %send to COORDINATOR 2

%% For each time step, a new random walk path will be generated for all 8
% sensor nodes
t=1;
while (get(hObject,'Value'))
    if t==1
        for k=1:8
        randwalk_new(k,:)=randwalk(k,:)+[step 0];
        end
    else
    compare=rand(8,2); % makes 8 random arrays filled with values ranging from 0 to 1
    for i=1:8
        drift(i,:)=compare(i,:)>[0.2 0.5]; %fills drift array with 1s and 0s 
    end
    % based on the random drift generated, the 8 sensor nodes will each pick a
    % random direction each time step
    for k=1:8
        if drift(k,:)==1
            randwalk_new(k,:) = randwalk(k,:) + [step 0];
        elseif drift(k,1)==1
            randwalk_new(k,:) = randwalk(k,:) + [0 step];
        elseif drift(k,2)==1
            randwalk_new(k,:) = randwalk(k,:) + [-step 0];
        else
            randwalk_new(k,:) = randwalk(k,:) + [0 -step];
        end
        %this code is to ensure that randwalk does not go beyond boundaries
        %in any direction
        if randwalk_new(k,1)>length
            randwalk_new(k,:) = randwalk_new(k,:) + [-step 0];
        end
        if randwalk_new(k,1)<0
            randwalk_new(k,:) = randwalk_new(k,:) + [step 0];    
        end
        if randwalk_new(k,2)>width
            randwalk_new(k,:) = randwalk_new(k,:) + [0 -step];
        end
        if randwalk_new(k,2)<0
            randwalk_new(k,:) = randwalk_new(k,:) + [0 step];
        end
    end
    
    % finding collisions algorithm. still needs work
    %i=1;
    %d=pdist2(randwalk_new,randwalk_new);
    %d(d==0) = 100; %make diagonal huge
    %[ix,iy] = find(d<rad);%repmat(rad,1,numel(ix))
    %if isempty(ix)== 0
    %    dist(i,:,:) = randwalk_new(ix,:);
    %    i = i + 1;
    %    fprintf('collision is in = %d.\n',randwalk_new(ix,:));
    %    viscircles(randwalk_new(ix,:),repmat(rad,numel(ix),1),'Color','k')
    %end
    end
    hold on
    set(gcf, 'units','normalized','outerposition',[0 0 1 1]);
    grid on
    title('Random Walk with Drift model for Sensor Nodes', 'FontSize', 15);
    xlabel('X distance', 'FontSize', 10);
    ylabel('Y distance', 'FontSize', 10);
    axis([0,length,0,width]);
    
    for p=1:8
        line(p)=plot([randwalk(p,1) randwalk_new(p,1)],[randwalk(p,2) randwalk_new(p,2)],'-o','Color',colorMap(p,:),'LineWidth', 4); 
    end
    drawnow
    if t==1 %initial case: no prev orientation values, so move everything straight
        mov_package=[67 100 100 100 100 100 100 100 100 100 100 100 100 100 100 100 100 77];
        [~]=sendData(s,mov_package);
        pause(step/14);
        [~]=sendData(s,stop_package);
        [~]=sendData(s,stop_package);
        pause(delay);
    else
    for car=1:8
        %calculate original orientation
        orientation(car,1)=180; % orientation always ends at 0
        %if randwalk_prev(car,1)==randwalk(car,1)
        %    if randwalk_prev(car,2)==randwalk(car,2)+step
        %        orientation(car,1)=90;%up
        %    else
        %        orientation(car,1)=-90; %down
        %    end
        %elseif randwalk_prev(car,1)==randwalk(car,1)+step
        %    orientation(car,1)=0; %right
        %else
        %   orientation(car,1)=180; %left
        %end
        %calculate final orientation
        if randwalk(car,1)==randwalk_new(car,1)
            if randwalk(car,2)==randwalk_new(car,2)+step
                orientation_new(car,1)=90;%up
            else
                orientation_new(car,1)=-90; %down
            end
        elseif randwalk(car,1)==randwalk_new(car,1)+step
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
            rot_package(1,car*2)=228; %left turn
            rot_package(1,car*2+1)=100;
            mov_package(1,car*2)=100;
            mov_package(1,car*2+1)=100;
        else
            rot_package(1,car*2)=100; %right turn
            rot_package(1,car*2+1)=228;
            mov_package(1,car*2)=100;
            mov_package(1,car*2+1)=100;
        end
    end
    %create header and footer for packages
    rot_package(1)='C'; mov_package(1)='C';
    rot_package(18)='M'; mov_package(18)='M';
    %send packages
    [~]=sendData(s,rot_package);
    pause(0.34); %time for 90 degree turn
    [~]=sendData(s,stop_package);
    [~]=sendData(s,stop_package);
    pause(0.020);
    [~]=sendData(s,mov_package);
    pause(step/13); %move 5 cm at 10cm/s
    [~]=sendData(s,stop_package);
    [~]=sendData(s,stop_package);
    %reorientation to align at 0
    for car=2:17 %flip sign bit of nonzero values
        if rot_package(car)~= 0
            rot_package(car)=bitxor(128,rot_package(car));
        end
    end
    [~]=sendData(s,rot_package);
    pause(0.34); %time for 90 degree turn
    [~]=sendData(s,stop_package);
    [~]=sendData(s,stop_package);
    pause(delay);
    end
    %UPDATE the new position
    randwalk_prev=randwalk;
    randwalk=randwalk_new;
    t=t+1;
end
%close port
fclose(s);
delete(s);