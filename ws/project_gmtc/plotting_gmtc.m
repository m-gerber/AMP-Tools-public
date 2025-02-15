% Max Gerber
% ASEN 5254
% 16 December 2023

close all
clear
clc

% Script to plot workspace and solutions from OMPL.

%% Load Data

workspace = fileread("Problem.yml");
workspace = yaml.load(workspace);
RRT_star_data = load("RRT_star_gmtc_data.txt");
PRM_star_data = load("PRM_star_gmtc_data.txt");
PRM_star_data = load("test.txt");


%% Plotting

plot_RRT = 0;
plot_PRM = 1;


%% Load Agent Properties

carWidth = workspace.Agents.agent0.Shape{1};
carHeight = workspace.Agents.agent0.Shape{1};


%% Plot RRT_star

if (plot_RRT)

xmin = 0;
xmax = workspace.Map.Dimensions{1};
ymin = 0;
ymax = workspace.Map.Dimensions{2};

obs_names = fieldnames(workspace.Map.Obstacles);
num_obstacles = numel(obs_names);

obstacles = zeros(num_obstacles, 4);

figure, axis equal, hold on, grid minor, axis([xmin xmax ymin ymax])
title('Geometric RRT* Robot Path')
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

start_x = workspace.Agents.agent0.Start{1};
start_y = workspace.Agents.agent0.Start{2};

end_x   = workspace.Agents.agent0.Goal{1};
end_y   = workspace.Agents.agent0.Goal{2};

plot(start_x,start_y,'bo','MarkerSize',10,'LineWidth',3)
plot(end_x,  end_y,  'ro','MarkerSize',10,'LineWidth',3)

RRT_dist = 0;

for i = 1:length(RRT_star_data)
    cx    = RRT_star_data(i,1);
    cy    = RRT_star_data(i,2);
    theta = RRT_star_data(i,3);

    if (i > 1) 
        RRT_dist  = RRT_dist + pdist([cx cy; cx_prev cy_prev]);
        plot([cx_prev cx],[cy_prev cy],'k','LineWidth',2)
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
    plot(car)
    pause(0.1)

    cx_prev = cx;
    cy_prev = cy;
end

fprintf('Distance for RRT*: %.2f\n', RRT_dist)

end

%% Plot PRM_star

if (plot_PRM)

xmin = 0;
xmax = workspace.Map.Dimensions{1};
ymin = 0;
ymax = workspace.Map.Dimensions{2};

obs_names = fieldnames(workspace.Map.Obstacles);
num_obstacles = numel(obs_names);

obstacles = zeros(num_obstacles, 4);

figure, axis equal, hold on, grid minor, axis([xmin xmax ymin ymax])
title('Geometric PRM* Robot Path')
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

start_x = workspace.Agents.agent0.Start{1};
start_y = workspace.Agents.agent0.Start{2};

end_x   = workspace.Agents.agent0.Goal{1};
end_y   = workspace.Agents.agent0.Goal{2};

plot(start_x,start_y,'bo','MarkerSize',10,'LineWidth',3)
plot(end_x,  end_y,  'ro','MarkerSize',10,'LineWidth',3)

PRM_dist = 0;

for i = 1:length(PRM_star_data)
    cx    = PRM_star_data(i,1);
    cy    = PRM_star_data(i,2);
    theta = PRM_star_data(i,3);

    if (i > 1) 
        PRM_dist  = PRM_dist + pdist([cx cy; cx_prev cy_prev]);
        plot([cx_prev cx],[cy_prev cy],'k','LineWidth',2)
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
    plot(car)
    pause(0.1)

    cx_prev = cx;
    cy_prev = cy;
end

fprintf('Distance for PRM*: %.2f\n', PRM_dist)

end