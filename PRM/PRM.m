% Probabilistic Roadmap Method (PRM)

% clear
clc;
clear;
close all;

t = cputime; % run time

%% Environments
Environ = 2;
MAX_X = 20; % x range
MAX_Y = 40; % y range

% Grid of Map (one to visualize one to calcuate)
MAP = zeros(MAX_X,MAX_Y); % use to visualize 
map = ones(MAX_X,MAX_Y); % use to calculate

% Get the different environment setting depending on values: 1, 2, 3
[start, target, obstacles] = Map(Environ);

% create obstacles
for i = 1: 1:size(obstacles,1)
    MAP(obstacles(i,1),obstacles(i,2)) = 1;
    map(obstacles(i,1),obstacles(i,2)) = 0;
end

map = map';
node = 250; % number of points in the PRM

% visualize environmnet
hold on
imagesc((MAP'))
colormap(flipud(gray));
axis([1 MAX_X 1 MAX_Y])

temp = [start;target]; % start and target as first points
p1 = plot(temp(1,2), temp(1,1),'kh','MarkerFaceColor','g'); %start
p2 = plot(temp(2,2), temp(2,1),'mh','MarkerFaceColor','m'); %target

%% Random Sampling 

% get random nodes until total nodes is reach
while length(temp)<node+2 
    x = double(int32(rand(1,2) .* size(map))); % random value
    
    % if okay random point, put into array (temp)
    if GoodP(x,map)
        temp = [temp;x];
        p3 = plot(x(2),x(1),'b.'); % plot nodes
    end
end

%% create node paths (potential paths)

adjacency = cell(node+2,1); % adjacency list
for i=1:node+2
    for j=i+1:node+2
        if pathCheck(temp(i,:),temp(j,:),map)
            adjacency{i} = [adjacency{i};j];adjacency{j}=[adjacency{j};i];
           % p4 = plot([temp(i,2);temp(j,2)],[temp(i,1);temp(j,1)], 'c', 'LineWidth', 0.1); % plot potentials lines
           % p4.Color(4) = 0.1;
        end
    end
end

%% Determine the best path

% node = index of node in temp, historic cost, heuristic cost, total cost, parent index in temp list (-1 for start)
X = [1 0 heu(temp(1,:),target) 0+heu(temp(1,:),target) -1]; % the process through A* algorihtm
p_index = []; % parent index
path_F = false; % path found
while size(X,1)>0
    [A, I] = min(X,[],1);
    n = X(I(4),:); % check smallest cost element
    X = [X(1:I(4)-1,:);X(I(4)+1:end,:)]; % delete element (currently)
    
    if n(1)==2 % check
        path_F = true;
        break;
    end
    
    % iterate through all adjacency from the node
    for mv=1:length(adjacency{n(1),1}) 
        temp1 = adjacency{n(1),1}(mv);
        
        % if not already in p_index
        if length(p_index)==0 || length(find(p_index(:,1)==temp1))==0
            historic_c = n(2)+hist(temp(n(1),:),temp(temp1,:)); % historic cost
            heuristic_c = heu(temp(temp1,:),target); % heuristic cost
            total_c = historic_c+heuristic_c; % total cost
            
            add = true; % add if better cost
            if length(find(X(:,1)==temp1))>=1
                I = find(X(:,1)==temp1);
                if X(I,4)<total_c
                    add = false;
                else
                    X=[X(1:I-1,:);X(I+1:end,:);];
                    add = true;
                end
            end
            if add
                X = [X;temp1 historic_c heuristic_c total_c size(p_index,1)+1]; % add new nodes
            end
        end
    end
    p_index = [p_index;n]; % update list
end

% if path is not found
if ~path_F
    error('No Path!')
end

%retrieve path from parent index
path = temp(n(1),:); 
prev = n(5);
while prev>0
    path = [temp(p_index(prev,1),:);path];
    prev = p_index(prev,5);
end

% visualize the path
p5 = plot(path(:,2),path(:,1),'color','r');
%legend([p1 p2 p3(1) p4 p5], {'Start','Target', 'Nodes', 'Potental Paths', 'Path'}, 'Location', 'southeast')
legend([p1 p2 p3(1) p5], {'Start','Target', 'Nodes', 'Path'}, 'Location', 'bestoutside')
title(['PRM in Environment ', num2str(Environ)]) % title
hold off

%% Time and cost function (recalculated just in case)

% time
e = cputime-t;

% cost function
d = diff([path(:,2), path(:,1)]);
total_length = sum(sqrt(sum(d.*d,2)));
fprintf("Processing Time: %.2f sec \t Path Length: %.2f bits \n", e, total_length);
%saveas(p5,'image1.png');

%% Cost function calculation

% historic
function h=hist(a,b)
    h = sqrt(sum((a-b).^2));
end

% heuristic
function h=heu(c,d)
    h = sqrt(sum((c-d).^2));
end