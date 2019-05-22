% Astar process

function path=AStarP(X,Y,MAP,target)

% Initialize
[H,W] = size(MAP); % height and width of matrix
g_cost = zeros(H,W); % g-cost (distance from starting node)
h_cost = single(zeros(H,W)); % hue (h-cost) (distance from end node)
f_cost = single(inf(H,W)); % f-cost for open list (g-cost + h-cost)
par_x = int16(zeros(H,W)); % parent reg of x
par_y = int16(zeros(H,W)); % parent reg of y
connect_Dist = 2; % degree of connection
o_matrix = int8(zeros(H,W)); % open spots
c_matrix = int8(zeros(H,W)); % closed spots
c_matrix(MAP==1) = 1;

%% Create grid to be looked at

check = ones(2*connect_Dist+1);
temp = 2*connect_Dist+2;
mid = connect_Dist+1;

% grids (neighbours) to be investigated around current point (brute force)
for i=1:connect_Dist-1
    check(i,i) = 0;
    check(temp-i,i) = 0;
    check(i,temp-i) = 0;
    check(temp-i,temp-i) = 0;
    check(mid,i) = 0;
    check(mid,temp-i) = 0;
    check(i,mid) = 0;
    check(temp-i,mid) = 0;
end
check(mid,mid) = 0;

% nearest (around)
[row, col] = find(check==1);
near = [row col]-(connect_Dist+1);
next_Near = size(col,1);

%% H cost (heuristic) process = distance from end node (target)
[col, row] = find(target==1);
goal_Reg = [row col]; % goal
nodes_F = size(goal_Reg,1);

% calculate cost
for k=1:size(target,1)
    for j=1:size(target,2)
        if MAP(k,j)==0
            ma = goal_Reg-(repmat([j k],(nodes_F),1));
            distance = min(sqrt(sum(abs(ma).^2,2)));
            h_cost(k,j) = distance;
        end
    end
end

%% F-cost = G-cost + H-cost
f_cost(Y,X) = h_cost(Y,X);
o_matrix(Y,X) = 1;

% ends when no options are available
while 1==1
    temp_Fcost = min(min(f_cost));
    
    % if no more paths exist
    if temp_Fcost==inf
        path = inf;
        create_Path = 0;
        break
    end
    [nowY,nowX] = find(f_cost==temp_Fcost);
    nowY = nowY(1);
    nowX = nowX(1);
    
    % if it reaches the target
    if target(nowY,nowX)==1
        create_Path = 1;
        break
    end
    
    % Open space to close space
    o_matrix(nowY,nowX) = 0;
    f_cost(nowY,nowX) = inf;
    c_matrix(nowY,nowX) = 1;
    
    for p=1:next_Near
        i = near(p,1); %Y
        j = near(p,2); %X
        
        % if path is an option
        if nowY+i<1||nowY+i>H||nowX+j<1||nowX+j>W
            continue
        end
        
        flag=1;
        % if path is open
        if c_matrix(nowY+i,nowX+j)==0
            
            % check if path does not pass obstacles
            if (abs(i)>1 || abs(j)>1)
                tempC = 2*max(abs(i),abs(j))-1;
                for K=1:tempC
                    YPOS = round(K*i/tempC);
                    XPOS = round(K*j/tempC);
                    
                    if MAP(nowY+YPOS,nowX+XPOS)==1
                        flag=0; % not okay
                    end
                    
                end
            end
            
            % if okay
            if flag==1
                temp_gScore = g_cost(nowY,nowX) + sqrt(i^2+j^2);
                if o_matrix(nowY+i,nowX+j)==0
                    o_matrix(nowY+i,nowX+j) = 1;
                elseif temp_gScore >= g_cost(nowY+i,nowX+j)
                    continue
                end
                
                par_x(nowY+i,nowX+j) = nowX;
                par_y(nowY+i,nowX+j) = nowY;
                g_cost(nowY+i,nowX+j) = temp_gScore;
                f_cost(nowY+i,nowX+j) = temp_gScore+h_cost(nowY+i,nowX+j);
            end
        end
    end
end

%% add to path

k=2;
if create_Path
    path(1,:) = [nowY nowX];
    while create_Path
        x_temp = par_x(nowY,nowX);
        nowY = par_y(nowY,nowX);
        nowX = x_temp;
        path(k,:) = [nowY nowX];
        k = k+1;
        if (nowX== X && nowY==Y)
            break
        end
    end
end
end
