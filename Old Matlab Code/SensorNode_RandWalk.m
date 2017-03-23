clear;
%close all
clc;
%clearvars;
%Two-dimensional Random Walk model with multiple paths and with initial positions at random places

type RandDir %function to simulate random direction 

K = 8; % Number of paths to simulate
N = 2;  % number of dimensions
M = 75; % number of steps
X = -5+10*rand(M,N,K); % initial position for all paths is random
colorMap = hsv(K);
figure;
for path = 1:K
    for step = 2:M
        X(step,:,path) = X(step-1,:,path)+ RandDir(N)';
        xdir = X(1:step,1,path);
        ydir = X(1:step,2,path);
        plot(xdir, ydir,'o-', 'Color', colorMap(path,:),'LineWidth', 2);
        hold on
        textLabel = sprintf('%d', step);
        text(xdir(end), ydir(end), textLabel, 'fontSize', 8);
        if path==2 || path==3 || path==4 || path==5 || path==6 || path==7 || path==8
            i=path
            if X(step,1,path)==X(step,1,path-1) && X(step,2,path)==X(step,2,path-1)
                plot(X(step,1,path),X(step,2,path),'rx','LineWidth', 2,'MarkerSize',15);
            end
        end
                
    end
end
hold on
set(gcf, 'units','normalized','outerposition',[0 0 1 1]);
grid on
title('Random Walk model for Sensor Nodes', 'FontSize', 15);
xlabel('X distance', 'FontSize', 15);
ylabel('Y distance', 'FontSize', 15);
for path=1:K
    plot(X(1,1,path), X(1,2,path),'ks', 'LineWidth', 2,'MarkerFaceColor',colorMap(path,:),'MarkerSize',15);
    textLabel = sprintf('%d', path);
    text(X(1,1,path), X(1,2,path), textLabel, 'fontSize', 12);
    plot(X(end,1,path), X(end,2,path),'kp', 'LineWidth', 2,'MarkerFaceColor',colorMap(path,:),'MarkerSize',15);
    %textLabel = sprintf('%d', path);
    %text(X(end,1,path), X(end,2,path), textLabel, 'fontSize', 15);
end
% Collision-finding algorithm
collision=0;
for path=1:K-1
    if path==1
        for other=2:K
            for step=2:M
                %X(step,1,path)==X(step,1,other) && X(step,2,path)==X(step,2,other)
                if X(step,1,path)==X(step,1,other) && X(step,2,path)==X(step,2,other)
                    collision=collision+1;
                end
            end
        
        end
    elseif path==2
        for other=3:K
            for step=2:M
                %X(step,1,path)==X(step,1,other) && X(step,2,path)==X(step,2,other)
                if X(step,1,path)==X(step,1,other) && X(step,2,path)==X(step,2,other)
                    collision=collision+1;
                end
            end
        end
    
     elseif path==3
        for other=4:K
            for step=2:M
                %X(step,1,path)==X(step,1,other) && X(step,2,path)==X(step,2,other)
                if X(step,1,path)==X(step,1,other) && X(step,2,path)==X(step,2,other)
                    collision=collision+1;
                end
            end
        
        end
    
    elseif path==4
        for other=5:K
            for step=2:M
                %X(step,1,path)==X(step,1,other) && X(step,2,path)==X(step,2,other)
                if X(step,1,path)==X(step,1,other) && X(step,2,path)==X(step,2,other)
                    collision=collision+1;
                end
            end
        
        end
    
    elseif path==5
        for other=6:K
            for step=2:M
                %X(step,1,path)==X(step,1,other) && X(step,2,path)==X(step,2,other)
                if X(step,1,path)==X(step,1,other) && X(step,2,path)==X(step,2,other)
                    collision=collision+1;
                end
            end
        
        end
    
    elseif path==6
        for other=7:K
            for step=2:M
                %X(step,1,path)==X(step,1,other) && X(step,2,path)==X(step,2,other)
                if X(step,1,path)==X(step,1,other) && X(step,2,path)==X(step,2,other)
                    collision=collision+1;
                end
            end
        
        end
    
    elseif path==7
        for other=8:K
            for step=2:M
                %X(step,1,path)==X(step,1,other) && X(step,2,path)==X(step,2,other)
                if X(step,1,path)==X(step,1,other) && X(step,2,path)==X(step,2,other)
                    collision=collision+1;
                end
            end
        
        end
    
    else
        collision=0;
    end
end





