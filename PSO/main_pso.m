% Particle Swarm Optimization (PSO)

% clear
clc;
clear;
close all;

t = cputime; % run time
%% Environments
Environ = 2;
MAX_X = 20; % x range
MAX_Y = 40; % y range

% Grid of Map
MAP = zeros(MAX_X,MAX_Y);

% Get the different environment setting depending on values: 1, 2, 3
[start, target, obstacles] = Map(Environ);

% create obstacles
for i = 1: 1:size(obstacles,1)
    MAP(obstacles(i,1),obstacles(i,2)) = 1;
end

% create struct to for easier passing around
% struct = [start, target, obstacles, radius of obstacles, min, max, number
%           of handle points, MAP]
S = struct('s', start, 't', target, 'obs', obstacles', 'robs', .62*ones(1,size(obstacles,1)),  ...
    'min', [1 1], 'max', [MAX_X MAX_Y], 'n', 4, 'MAP', MAP);

%% PSO Parameters

cost_Func = @(x) costFunc(x,S); % cost function
vSize = [1 S.n];   % decision variables
iter = 400; % iterations
pop = 200; % Swarm Size
weight = 1; % inertia weight
w_Damp = 1; % damping ratio (inertia weight)
c1 = 2; % personal learning coefficient
c2 = 2; % global learning coefficient

% velocity
alpha=0.1;
max_Vel = [alpha*(S.max(1)-S.min(1)) alpha*(S.max(2)-S.min(2))]; % maximum velocity
min_Vel = [-alpha*(S.max(1)-S.min(1)) -alpha*(S.max(2)-S.min(2))]; % minimum velocity

%% Initialization

% global best
gBest.cost = inf;

% Pso struct = [position, velocity, cost, path, best = {best position, best
%                cost, best path}]
temp_struct = struct('pos', [], 'vel', [], 'cost', [], 'path', [], 'best', []);
temp_struct.best = struct('pos', [], 'cost', [], 'path', []);
pso = repmat(temp_struct,pop,1);

% Get the first values (initialize loop)
for i=1:pop
    
    % position of starting handling points
    if i > 1
        % random the points
        pso(i).pos.x = unifrnd(S.min(1),S.max(1),1,S.n);
        pso(i).pos.y = unifrnd(S.min(2),S.max(2),1,S.n);
    else
        % line from start to target
        xx = linspace(S.s(1), S.t(1), S.n+2);
        yy = linspace(S.s(2), S.t(2), S.n+2);
        pso(i).pos.x = xx(2:end-1);
        pso(i).pos.y = yy(2:end-1);
    end
    
    % velocity
    pso(i).vel.x = zeros(vSize);
    pso(i).vel.y = zeros(vSize);
    
    % cost value
    [pso(i).cost, pso(i).path] = cost_Func(pso(i).pos);
    
    % update personal best
    pso(i).best.pos = pso(i).pos;
    pso(i).best.cost = pso(i).cost;
    pso(i).best.path = pso(i).path;
    
    % update global best
    if pso(i).best.cost<gBest.cost
        gBest = pso(i).best;
    end
    
end

% best cost values
best_Cost = zeros(iter,1);

%% PSO Main Loop

% counters
temp = 0; % iteration counter if cost doesnt change
temp1 = 0; % re-random counter if PSO is lost
output = VideoWriter('PSO.mp4', 'MPEG-4');
open(output);

for it=1:iter
    for i=1:pop
        
        % update velocity
        % x axis
        pso(i).vel.x = weight*pso(i).vel.x ...
            + c1*rand(vSize).*(pso(i).best.pos.x-pso(i).pos.x) ...
            + c2*rand(vSize).*(gBest.pos.x-pso(i).pos.x);
        pso(i).vel.y = weight*pso(i).vel.y ...
            + c1*rand(vSize).*(pso(i).best.pos.y-pso(i).pos.y) ...
            + c2*rand(vSize).*(gBest.pos.y-pso(i).pos.y);
        
        % update velocity boundaries
        pso(i).vel.x = max(pso(i).vel.x,min_Vel(1));
        pso(i).vel.x = min(pso(i).vel.x,max_Vel(1));
        pso(i).vel.y = max(pso(i).vel.y,min_Vel(2));
        pso(i).vel.y = min(pso(i).vel.y,max_Vel(2));
        
        % Update Position
        pso(i).pos.x = pso(i).pos.x + pso(i).vel.x;
        pso(i).pos.y = pso(i).pos.y + pso(i).vel.y;
        
        % reflect velocity
        outside = (pso(i).pos.x<S.min(1) | pso(i).pos.x>S.max(1));
        pso(i).vel.x(outside) = -pso(i).vel.x(outside);
        outside = (pso(i).pos.y<S.min(2) | pso(i).pos.y>S.max(2));
        pso(i).vel.y(outside) = -pso(i).vel.y(outside);
        
        % update position boundaries
        pso(i).pos.x = max(pso(i).pos.x,S.min(1));
        pso(i).pos.x = min(pso(i).pos.x,S.max(1));
        pso(i).pos.y = max(pso(i).pos.y,S.min(2));
        pso(i).pos.y = min(pso(i).pos.y,S.max(2));
        
        % calculate cost
        [pso(i).cost, pso(i).path] = cost_Func(pso(i).pos);
        
        % update personal best
        if pso(i).cost<pso(i).best.cost
            pso(i).best.pos = pso(i).pos;
            pso(i).best.cost = pso(i).cost;
            pso(i).best.path = pso(i).path;
            
            % global best
            if pso(i).best.cost<gBest.cost
                gBest = pso(i).best;
            end
            
        end
    end
    
    % best cost
    best_Cost(it) = gBest.cost;
    
    % if cost function doesnt change after 25 iteration stop the loop
    % also if PSO gets stuck the handling points are re-random
    if it>1
        
        % if the cost function doesnt change per iteration
        if best_Cost(it) == best_Cost(it-1) && gBest.path.vio == 0
            temp = temp +1;
        else  % if the path violates the rules reset the early break
            temp = 0;
        end
        
        % if the PSO is lost or path goes stright through obstacles
        if best_Cost(it) < 26 || gBest.path.vio ~= 0 || best_Cost(it) > 70
            temp1 = temp1+1;
        else
            temp1 = 0;
        end
    end
    
    % if no change in cost and rule breaking end iteration
    if temp > 25
        break;
    end
    
    % if rules are broken for 20 iterations re-random the handler points
    if temp1 > 20
        temp1 = 0;
        for i=1:pop
            if i > 1
                % random the points
                pso(i).pos.x = unifrnd(S.min(1),S.max(1),1,S.n);
                pso(i).pos.y = unifrnd(S.min(2),S.max(2),1,S.n);
            else
                % line from start to target
                xx = linspace(S.s(1), S.t(1), S.n+2);
                yy = linspace(S.s(2), S.t(2), S.n+2);
                pso(i).pos.x = xx(2:end-1);
                pso(i).pos.y = yy(2:end-1);
            end
            
            % reset everything with these new random points
            % cost function record is still recorded
            pso(i).vel.x=zeros(vSize);
            pso(i).vel.y=zeros(vSize);
            [pso(i).cost, pso(i).path] = cost_Func(pso(i).pos);
            pso(i).best.pos = pso(i).pos;
            pso(i).best.cost = pso(i).cost;
            pso(i).best.path = pso(i).path;
            
            % global best
            gBest = pso(i).best;
            
        end
    end
    
    % inertia weight damping
    weight = weight*w_Damp;
    
    %% Show Iteration Information
    if gBest.path.allowed
        passed = '||';
    else
        passed = ['|| Violation = ' num2str(gBest.path.vio)];
    end
    fprintf('Iteration: %d || Current Best Cost = %f %s \n', it, best_Cost(it), passed);
    
    %%visualize the iterations
     hh = figure(1);    
     plotIt(gBest.path,S);
     legend('Start', 'Target', 'Handle Points', 'Path', 'Location', 'southeast')
     title(['PSO in Environment ', num2str(Environ)]) % title
     FF = getframe(hh);
     writeVideo(output, FF); 
     pause(0.01);
    
end
close(output); % save video

%% Final plot
clf(figure(1))
figure(1)
plotIt(gBest.path,S); % plot
legend('Start', 'Target', 'Handle Points', 'Path', 'Location', 'best') % legend
title(['PSO in Environment ', num2str(Environ)]) % title
%% Time and cost function

% %cost function plot
% figure;
% plot(best_Cost,'LineWidth',2);
% xlabel('Iteration');
% ylabel('Best Cost');
% grid on;

% time
e = cputime-t;

% cost function
d = diff([gBest.path.xx(:) gBest.path.yy(:)]);
total_length = sum(sqrt(sum(d.*d,2)));
fprintf("Processing Time: %.2f sec \t Path Length: %.2f bits \t Iteration: %d \n", e, total_length, it-1);

%% Functions

%plot functions
function plotIt(gBest,S)
hold on
imagesc((S.MAP'))
colormap(flipud(gray));
axis([1 20 1 40])
plot(S.s(1), S.s(2), 'mh','MarkerSize',10,'MarkerFaceColor','m'); %start
plot(S.t(1), S.t(2), 'kh','MarkerSize',10,'MarkerFaceColor','g'); %finish
plot(gBest.X, gBest.Y,'bo'); % handling points
plot(gBest.xx, gBest.yy,'r'); % path
hold off;
end
