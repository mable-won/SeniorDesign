<<<<<<< HEAD
clear 
clc
i = 1;
%This is the Random Walk with Drift Model
colorMap = hsv(8); % just to make the 8 paths have different colors in the graph
rad = 0.5; % for collision 
randwalk=10*rand(8,2); %8 sensor nodes starting coordinate is at random locations between [0 10] on all directions
drift=zeros(8,2);
% for each time step, a new random walk path will be generated for all 8
% sensor nodes
for t=1:200 
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
    d=pdist2(randwalk_new,randwalk_new);
    d(d==0) = 100; %make diagonal huge
    [ix,iy] = find(d<rad);%repmat(rad,1,numel(ix))
    if isempty(ix)== 0
        dist(i,:,:) = randwalk_new(ix,:);
        i = i + 1;
        fprintf('collision is in = %d.\n',randwalk_new(ix,:));
        viscircles(randwalk_new(ix,:),repmat(rad,numel(ix),1),'Color','k')
    end
   
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
    %UPDATE the new position
    randwalk=randwalk_new ;
end

=======
clear 
clc
i = 1;
%This is the Random Walk with Drift Model
colorMap = hsv(8); % just to make the 8 paths have different colors in the graph
rad = 0.5; % for collision 
randwalk=10*rand(8,2); %8 sensor nodes starting coordinate is at random locations between [0 10] on all directions
drift=zeros(8,2);
% for each time step, a new random walk path will be generated for all 8
% sensor nodes
for t=1:200 
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
    d=pdist2(randwalk_new,randwalk_new);
    d(d==0) = 100; %make diagonal huge
    [ix,iy] = find(d<rad);%repmat(rad,1,numel(ix))
    if isempty(ix)== 0
        dist(i,:,:) = randwalk_new(ix,:);
        i = i + 1;
        fprintf('collision is in = %d.\n',randwalk_new(ix,:));
        viscircles(randwalk_new(ix,:),repmat(rad,numel(ix),1),'Color','k')
    end
   
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
    %UPDATE the new position
    randwalk=randwalk_new ;
end

>>>>>>> origin/master
