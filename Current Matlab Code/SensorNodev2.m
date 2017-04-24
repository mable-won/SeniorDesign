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
% version 2.3 by M.C. Lalata and R. Dunn at the University of Houston on
% 4/22/17

colorMap = hsv(8); % just to make the 8 paths have different colors in the graph
%% Tunable Variables
SNNumber = 5;
rad = 7.0; % for collision in cm
delay = 1.0; %delay in loop time in s (as a float value)
step = 5; %in cm
times = 10; %loop iterations
length = 130; %x-direction in cm
width = 105; %y-direction in cm

%sensor nodes starting coordinates spaced out along left side of the arena
randwalk = zeros(SNNumber,2);
for i = 1:SNNumber
    randwalk(i,1) = 0; %x-coordinate
    randwalk(i,2)= width/SNNumber*(i-0.5); %y-coordinate
end

%% Preallocation
drift=zeros(SNNumber,2);
line=zeros(SNNumber,1);
randwalk_new=zeros(SNNumber,2);
randwalk_prev=zeros(SNNumber,2);
orientation=zeros(SNNumber,1);
orientation_new=zeros(SNNumber,1);
packages=zeros(SNNumber,2);
voltages=zeros(SNNumber,1);
rot_package=zeros(1,2*SNNumber+2);
mov_package=zeros(1,2*SNNumber+2);
stop_package=zeros(1,2*SNNumber+2); stop_package(1)=67; stop_package(2*SNNumber+2)=77;
send_package=zeros(1,2*SNNumber+2); send_package(1)=86; send_package(2*SNNumber+2)=63;
volt_package=zeros(SNNumber);

%open Serial port
s=setupSerial('COM5');

%% Poll sensor nodes for voltage, sort, then send to coordinator 2
% Keep commented until all cars in use. Note: XBee package receival rate
% notoriously low, meaning they may not transmit their voltage back, and
% the program will likely either timeout or wait indefinitely.
%[~]=sendData(s,send_package);
%for car=1:SNNumber
%    [packages(car,:)]=receiveData(s,2);
%    voltages(car,1)=bitshift(packages(car,1),8)+packages(car,2);
%end
%[~,index]=sort(voltages);
%for car=1:SNNumber
%    volt_package(car)=index;
%end
%[~]=sendData(s,volt_package,'000D'); %send to COORDINATOR 2

%% For each time step, a new random walk path will be generated for all 8
% sensor nodes
t=1;
while (get(hObject,'Value'))
    if t==1
        for k=1:SNNumber
            randwalk_new(k,:) = randwalk(k,:) + [step 0];
        end
    else
        compare=rand(SNNumber,2); % makes 8 random arrays filled with values ranging from 0 to 1
        for i=1:SNNumber
            drift(i,:)=compare(i,:)>[0.2 0.5]; %fills drift array with 1s and 0s 
        end
        % based on the random drift generated, the 8 sensor nodes will each pick a
        % random direction each time step
        for k=1:SNNumber
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
            %dist(i,:,:) = randwalk_new(ix,:);
            %i = i + 1;
            %fprintf('collision is in = %d.\n',randwalk_new(ix,:));
            %viscircles(randwalk_new(ix,:),repmat(rad,numel(ix),1),'Color','k')
        %end
    end
    hold on
    set(gcf, 'units','normalized','outerposition',[0 0 1 1]);
    grid on
    title('Sensor Node Movement Simulation', 'FontSize', 16);
    xlabel('X distance [cm]', 'FontSize', 12);
    ylabel('Y distance [cm]', 'FontSize', 12);
    axis([0,length,0,width]);
    
    for p=1:SNNumber
        line(p)=plot([randwalk(p,1) randwalk_new(p,1)],[randwalk(p,2) randwalk_new(p,2)],'-o','Color',colorMap(p,:),'LineWidth', 4); 
    end
    drawnow
    if t==1 %initial case: no prev orientation values, so move everything straight
        mov_package(1)='C'; mov_package(2*SNNumber+2)='M';
        for index = 2:2*SNNumber+1
            mov_package(index)=100;
        end
        time1 = tic;
        while (toc(time1) < step/13) %move 5 cm at 13cm/s
            [~]=sendData(s,mov_package);
        end
        time2 = tic;
        while (toc(time2) < delay)
            [~]=sendData(s,stop_package);
        end
    else
        for car=1:SNNumber
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
            %calculate rotation step & move step
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
        rot_package(2*SNNumber+2)='M'; mov_package(2*SNNumber+2)='M';
        %send packages
        time1 = tic;
        while (toc(time1) < 0.34) %time for 90 degree turn
            [~]=sendData(s,rot_package);
        end
        time2 = tic;
        while (toc(time2) < 0.02)
            [~]=sendData(s,stop_package);
        end
        time3 = tic;
        while (toc(time3) < step/13) %move 5 cm at 13cm/s
            [~]=sendData(s,mov_package);
        end
        time4 = tic;
        while (toc(time4) < 0.02)
            [~]=sendData(s,stop_package);
        end
        %reorientation to align at 0
        for index=2:2*SNNumber+1 %flip sign bit of nonzero values
            if rot_package(index)~= 0
                rot_package(index)=bitxor(128,rot_package(index));
            end
        end
        time5 = tic;
        while (toc(time5) < 0.34) %time for 90 degree turn
            [~]=sendData(s,rot_package);

        end
        time6 = tic;
        while (toc(time6) < delay)
            [~]=sendData(s,stop_package);
        end
    end
    %UPDATE the new position
    randwalk_prev=randwalk;
    randwalk=randwalk_new;
    t=t+1;
    % Delete all timers from memory.
    listOfTimers = timerfindall;
    if ~isempty(listOfTimers)
        delete(listOfTimers(:));
    end
end
% close port
fclose(s);
delete(s);
