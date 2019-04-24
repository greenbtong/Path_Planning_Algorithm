% Astar

% clear
clc;
clear;
close all;

t = cputime; % run time

%% Environments
Environ = 1;
MAX_X = 20; % x range
MAX_Y = 40; % y range

% Grid of Map
MAP = zeros(MAX_X,MAX_Y);

% Get the different environment setting depending on values: 1, 2, 3
[start, target, obstacles] = Map(Environ);

%target
target_map = int8(MAP);
target_map(target(1),target(2)) = 1;

% create obstacles
for i = 1:1:size(obstacles,1)
    MAP(obstacles(i,1),obstacles(i,2)) = 1;
end

%% Astar pathfinding algorithm
path = AStarP(start(2),start(1),MAP,target_map); 

%% if path is found visualize

if size(path,2)>1
    
    % visualize environmnet
    hold on
    imagesc((MAP'))
    colormap(flipud(gray));
    axis([1 MAX_X 1 MAX_Y])
    plot(path(end,1),path(end,2),'kh','MarkerFaceColor','g'); % start
    plot(path(1,1),path(1,2),'mh','MarkerFaceColor','m'); % target
    plot(path(:,1),path(:,2),'r'); % path
    legend('Start','Goal', 'Path', 'Location', 'best') % legend
    title(['A Star in Environment ', num2str(Environ)]) % title
    
else
    % if path is not found
    error('No Path!')
end

%% Time and cost function

% time
e = cputime-t;

% cost function
d = diff([path(:,2), path(:,1)]);
total_length = sum(sqrt(sum(d.*d,2)));
fprintf("Processing Time: %.2f sec \t Path Length: %.2f bits \n", e, total_length);
