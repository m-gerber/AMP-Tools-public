% Max Gerber
% ASEN 5254
% 16 December 2023

close all
clear
clc

% Script to plot workspace and solutions from OMPL for control based
% planning.

%% Load Data

workspace       = fileread("Problem.yml");
workspace       = yaml.load(workspace);
RRT_data        = load("RRT_ctrl_data.txt");
RRT_smooth_data = load("RRT_ctrl_smooth_data.txt");
SST_data        = load("SST_ctrl_data.txt");
SST_smooth_data = load("SST_ctrl_smooth_data4.txt");


%% Turn ON/OFF Plotting

plot_RRT        = 0;
plot_RRT_smooth = 0;
plot_SST        = 0;
plot_SST_smooth = 1;


%% Load Agent Properties

carWidth  = workspace.Agents.agent0.Shape{1};
carHeight = workspace.Agents.agent0.Shape{1};


%% Set Bounds and Obstacles

xmin = 0;
xmax = workspace.Map.Dimensions{1};
ymin = 0;
ymax = workspace.Map.Dimensions{2};

start_x = workspace.Agents.agent0.Start{1};
start_y = workspace.Agents.agent0.Start{2};

end_x   = workspace.Agents.agent0.Goal{1};
end_y   = workspace.Agents.agent0.Goal{2};

obs_names = fieldnames(workspace.Map.Obstacles);
num_obstacles = numel(obs_names);

obstacles = zeros(num_obstacles, 4);


%% Plot RRT_data

%{
Info:    RRT: Starting planning with 1 states already in datastructure
Info:    RRT: Created 180789 states
Info:    Solution found in 904.473483 seconds
%}

if (plot_RRT)

figure, axis equal, hold on, axis([xmin xmax ymin ymax])
title('Control RRT Robot Path')
set(gca,'FontSize',20)

for i = 1:num_obstacles
    obstacles(i,1) = workspace.Map.Obstacles.(obs_names{i}){1};
    obstacles(i,2) = workspace.Map.Obstacles.(obs_names{i}){2};
    obstacles(i,3) = workspace.Map.Obstacles.(obs_names{i}){3}-...
        obstacles(i,1);
    obstacles(i,4) = workspace.Map.Obstacles.(obs_names{i}){4}-...
        obstacles(i,2);
    rectangle('Position',obstacles(i,:),'FaceColor',[0 0 0])
end

% Plot Path

plot(start_x,start_y,'bo','MarkerSize',10,'LineWidth',3)
plot(end_x,  end_y,  'ro','MarkerSize',10,'LineWidth',3)

RRT_dist = 0;

for i = 1:length(RRT_data)
    cx    = RRT_data(i,1);
    cy    = RRT_data(i,2);
    theta = RRT_data(i,5) + pi/2;

    if (i < size(RRT_data,1))
        u1 = RRT_data(i+1,6);
        u2 = RRT_data(i+1,7);
        [~,x] = ode45(@(t,x) dynamics(t,x,u1,u2,carHeight),...
            [0 RRT_data(i+1,8)],RRT_data(i,1:5));
        for j = 1:size(x,1)-1
            RRT_dist = RRT_dist + pdist([x(j,1) x(j,2); x(j+1,1) x(j+1,2)]);
        end
    end

    % TOP RIGHT VERTEX:
    TR_x = cx + ((carWidth / 2) * cos(theta)) - ((carHeight / 2) *...
        sin(theta));
    TR_y = cy + ((carWidth / 2) * sin(theta)) + ((carHeight / 2) *...
        cos(theta));

    % TOP LEFT VERTEX:
    TL_x = cx - ((carWidth / 2) * cos(theta)) - ((carHeight / 2) *...
        sin(theta));
    TL_y = cy - ((carWidth / 2) * sin(theta)) + ((carHeight / 2) *...
        cos(theta));

    % BOTTOM LEFT VERTEX:
    BL_x = cx - ((carWidth / 2) * cos(theta)) + ((carHeight / 2) *...
        sin(theta));
    BL_y = cy - ((carWidth / 2) * sin(theta)) - ((carHeight / 2) *...
        cos(theta));

    % BOTTOM RIGHT VERTEX:
    BR_x = cx + ((carWidth / 2) * cos(theta)) + ((carHeight / 2) *...
        sin(theta));
    BR_y = cy + ((carWidth / 2) * sin(theta)) - ((carHeight / 2) *...
        cos(theta));

    car = polyshape([TR_x TL_x BL_x BR_x],[TR_y TL_y BL_y BR_y]);
    
    plot(x(:,1),x(:,2),'k','LineWidth',1)
%     plot(car, 'FaceColor','none')
%     plot([TR_x TL_x],[TR_y TL_y],'k','LineWidth',2)
%     pause(0.05)
%     plot(car, 'EdgeColor', 'w','FaceColor','none')
%     plot([TR_x TL_x],[TR_y TL_y],'w','LineWidth',2)
%     plot(x(:,1),x(:,2),'k','LineWidth',1)

end

fprintf('Distance for RRT: %.2f\n', RRT_dist)

end


%% Plot RRT_smooth_data

%{
Info:    RRT: Starting planning with 1 states already in datastructure
Info:    RRT: Created 106853 states
Info:    Solution found in 416.504186 seconds
%}

if (plot_RRT_smooth)

figure, axis equal, hold on, axis([xmin xmax ymin ymax])
title('Control RRT Robot Path (Smooth)')
set(gca,'FontSize',20)

for i = 1:num_obstacles
    obstacles(i,1) = workspace.Map.Obstacles.(obs_names{i}){1};
    obstacles(i,2) = workspace.Map.Obstacles.(obs_names{i}){2};
    obstacles(i,3) = workspace.Map.Obstacles.(obs_names{i}){3}-...
        obstacles(i,1);
    obstacles(i,4) = workspace.Map.Obstacles.(obs_names{i}){4}-...
        obstacles(i,2);
    rectangle('Position',obstacles(i,:),'FaceColor',[0 0 0])
end

% Plot Path

plot(start_x,start_y,'bo','MarkerSize',10,'LineWidth',3)
plot(end_x,  end_y,  'ro','MarkerSize',10,'LineWidth',3)

RRT_smooth_dist = 0;

for i = 1:length(RRT_smooth_data)
    cx    = RRT_smooth_data(i,1);
    cy    = RRT_smooth_data(i,2);
    theta = RRT_smooth_data(i,5) + pi/2;

    if (i < size(RRT_smooth_data,1))
        u1 = RRT_smooth_data(i+1,6);
        u2 = RRT_smooth_data(i+1,7);
        [~,x] = ode45(@(t,x) dynamics(t,x,u1,u2,carHeight),...
            [0 RRT_smooth_data(i+1,8)],RRT_smooth_data(i,1:5));
        for j = 1:size(x,1)-1
            RRT_smooth_dist = RRT_smooth_dist + pdist([x(j,1) x(j,2); x(j+1,1) x(j+1,2)]);
        end
    end

    % TOP RIGHT VERTEX:
    TR_x = cx + ((carWidth / 2) * cos(theta)) - ((carHeight / 2) *...
        sin(theta));
    TR_y = cy + ((carWidth / 2) * sin(theta)) + ((carHeight / 2) *...
        cos(theta));

    % TOP LEFT VERTEX:
    TL_x = cx - ((carWidth / 2) * cos(theta)) - ((carHeight / 2) *...
        sin(theta));
    TL_y = cy - ((carWidth / 2) * sin(theta)) + ((carHeight / 2) *...
        cos(theta));

    % BOTTOM LEFT VERTEX:
    BL_x = cx - ((carWidth / 2) * cos(theta)) + ((carHeight / 2) *...
        sin(theta));
    BL_y = cy - ((carWidth / 2) * sin(theta)) - ((carHeight / 2) *...
        cos(theta));

    % BOTTOM RIGHT VERTEX:
    BR_x = cx + ((carWidth / 2) * cos(theta)) + ((carHeight / 2) *...
        sin(theta));
    BR_y = cy + ((carWidth / 2) * sin(theta)) - ((carHeight / 2) *...
        cos(theta));

    car = polyshape([TR_x TL_x BL_x BR_x],[TR_y TL_y BL_y BR_y]);
    
    plot(x(:,1),x(:,2),'k','LineWidth',1)
%     plot(car, 'FaceColor','none')
%     plot([TR_x TL_x],[TR_y TL_y],'k','LineWidth',2)
%     pause(0.05)
%     plot(car, 'EdgeColor', 'w','FaceColor','none')
%     plot([TR_x TL_x],[TR_y TL_y],'w','LineWidth',2)
%     plot(x(:,1),x(:,2),'k','LineWidth',1)

end

fprintf('Distance for RRT smooth: %.2f\n', RRT_smooth_dist)

end

%% Plot SST_data

%{
Info:    ProblemDefinition: Adding approximate solution from planner SST
Info:    SST: Created 441 states in 212377 iterations
Info:    Solution found in 300.094642 seconds
%}

if (plot_SST)

figure, axis equal, hold on, axis([xmin xmax ymin ymax])
title('Control SST Robot Path')
set(gca,'FontSize',20)

for i = 1:num_obstacles
    obstacles(i,1) = workspace.Map.Obstacles.(obs_names{i}){1};
    obstacles(i,2) = workspace.Map.Obstacles.(obs_names{i}){2};
    obstacles(i,3) = workspace.Map.Obstacles.(obs_names{i}){3}-...
        obstacles(i,1);
    obstacles(i,4) = workspace.Map.Obstacles.(obs_names{i}){4}-...
        obstacles(i,2);
    rectangle('Position',obstacles(i,:),'FaceColor',[0 0 0])
end

% Plot Path

plot(start_x,start_y,'bo','MarkerSize',10,'LineWidth',3)
plot(end_x,  end_y,  'ro','MarkerSize',10,'LineWidth',3)

SST_dist = 0;

for i = 1:size(SST_data,1)
    cx    = SST_data(i,1);
    cy    = SST_data(i,2);
    theta = SST_data(i,5) + pi/2;

    if (i < length(SST_data))
        u1 = SST_data(i+1,6);
        u2 = SST_data(i+1,7);
        [~,x] = ode45(@(t,x) dynamics(t,x,u1,u2,carHeight),...
            [0 SST_data(i+1,8)],SST_data(i,1:5));
        for j = 1:size(x,1)-1
            SST_dist = SST_dist + pdist([x(j,1) x(j,2); x(j+1,1) x(j+1,2)]);
        end
    end

    % TOP RIGHT VERTEX:
    TR_x = cx + ((carWidth / 2) * cos(theta)) - ((carHeight / 2) *...
        sin(theta));
    TR_y = cy + ((carWidth / 2) * sin(theta)) + ((carHeight / 2) *...
        cos(theta));

    % TOP LEFT VERTEX:
    TL_x = cx - ((carWidth / 2) * cos(theta)) - ((carHeight / 2) *...
        sin(theta));
    TL_y = cy - ((carWidth / 2) * sin(theta)) + ((carHeight / 2) *...
        cos(theta));

    % BOTTOM LEFT VERTEX:
    BL_x = cx - ((carWidth / 2) * cos(theta)) + ((carHeight / 2) *...
        sin(theta));
    BL_y = cy - ((carWidth / 2) * sin(theta)) - ((carHeight / 2) *...
        cos(theta));

    % BOTTOM RIGHT VERTEX:
    BR_x = cx + ((carWidth / 2) * cos(theta)) + ((carHeight / 2) *...
        sin(theta));
    BR_y = cy + ((carWidth / 2) * sin(theta)) - ((carHeight / 2) *...
        cos(theta));

    car = polyshape([TR_x TL_x BL_x BR_x],[TR_y TL_y BL_y BR_y]);
    
    plot(x(:,1),x(:,2),'k','LineWidth',1)
%     plot(car, 'FaceColor','none')
%     plot([TR_x TL_x],[TR_y TL_y],'k','LineWidth',2)
%     pause(0.05)
%     plot(car, 'EdgeColor', 'w','FaceColor','none')
%     plot([TR_x TL_x],[TR_y TL_y],'w','LineWidth',2)
%     plot(x(:,1),x(:,2),'k','LineWidth',1)
end

fprintf('Distance for SST: %.2f\n', SST_dist)

end


%% Plot SST_smooth_data

%{
[0.2 0.1]
Info:    ProblemDefinition: Adding approximate solution from planner SST
Info:    SST: Created 2778 states in 1318041 iterations
Info:    Solution found in 1800.063299 seconds

[0.2 0.1]
Info:    ProblemDefinition: Adding approximate solution from planner SST
Info:    SST: Created 429 states in 235444 iterations
Info:    Solution found in 300.006670 seconds

[1.0 0.5]
Info:    ProblemDefinition: Adding approximate solution from planner SST
Info:    SST: Created 291 states in 214731 iterations
Info:    Solution found in 300.106629 seconds

[1.0 0.5]
Info:    ProblemDefinition: Adding approximate solution from planner SST
Info:    SST: Created 1316 states in 1348581 iterations
Info:    Solution found in 1800.128625 seconds

[1.0 0.5]
Info:    ProblemDefinition: Adding approximate solution from planner SST
Info:    SST: Created 15856 states in 10019513 iterations
Info:    Solution found in 18000.012382 seconds
%}

if (plot_SST_smooth)

figure, axis equal, hold on, axis([xmin xmax ymin ymax])
title('Control SST Robot Path (Smooth)')
set(gca,'FontSize',20)

for i = 1:num_obstacles
    obstacles(i,1) = workspace.Map.Obstacles.(obs_names{i}){1};
    obstacles(i,2) = workspace.Map.Obstacles.(obs_names{i}){2};
    obstacles(i,3) = workspace.Map.Obstacles.(obs_names{i}){3}-...
        obstacles(i,1);
    obstacles(i,4) = workspace.Map.Obstacles.(obs_names{i}){4}-...
        obstacles(i,2);
    rectangle('Position',obstacles(i,:),'FaceColor',[0 0 0])
end

% Plot Path

plot(start_x,start_y,'bo','MarkerSize',10,'LineWidth',3)
plot(end_x,  end_y,  'ro','MarkerSize',10,'LineWidth',3)

SST_smooth_dist = 0;

for i = 1:size(SST_smooth_data,1)
    cx    = SST_smooth_data(i,1);
    cy    = SST_smooth_data(i,2);
    theta = SST_smooth_data(i,5) + pi/2;

    if (i < length(SST_smooth_data))
        u1 = SST_smooth_data(i+1,6);
        u2 = SST_smooth_data(i+1,7);
        [~,x] = ode45(@(t,x) dynamics(t,x,u1,u2,carHeight),...
            [0 SST_smooth_data(i+1,8)],SST_smooth_data(i,1:5));
        for j = 1:size(x,1)-1
            SST_smooth_dist = SST_smooth_dist + pdist([x(j,1) x(j,2); x(j+1,1) x(j+1,2)]);
        end
    end

    % TOP RIGHT VERTEX:
    TR_x = cx + ((carWidth / 2) * cos(theta)) - ((carHeight / 2) * sin(theta));
    TR_y = cy + ((carWidth / 2) * sin(theta)) + ((carHeight / 2) * cos(theta));

    % TOP LEFT VERTEX:
    TL_x = cx - ((carWidth / 2) * cos(theta)) - ((carHeight / 2) * sin(theta));
    TL_y = cy - ((carWidth / 2) * sin(theta)) + ((carHeight / 2) * cos(theta));

    % BOTTOM LEFT VERTEX:
    BL_x = cx - ((carWidth / 2) * cos(theta)) + ((carHeight / 2) * sin(theta));
    BL_y = cy - ((carWidth / 2) * sin(theta)) - ((carHeight / 2) * cos(theta));

    % BOTTOM RIGHT VERTEX:
    BR_x = cx + ((carWidth / 2) * cos(theta)) + ((carHeight / 2) * sin(theta));
    BR_y = cy + ((carWidth / 2) * sin(theta)) - ((carHeight / 2) * cos(theta));

    car = polyshape([TR_x TL_x BL_x BR_x],[TR_y TL_y BL_y BR_y]);
    
    plot(x(:,1),x(:,2),'k','LineWidth',1)
%     plot(car, 'FaceColor','none')
%     plot([TR_x TL_x],[TR_y TL_y],'k','LineWidth',2)
%     pause(0.05)
%     plot(car, 'EdgeColor', 'w','FaceColor','none')
%     plot([TR_x TL_x],[TR_y TL_y],'w','LineWidth',2)
%     plot(x(:,1),x(:,2),'k','LineWidth',1)
end

fprintf('Distance for SST smooth: %.2f\n', SST_smooth_dist)

end