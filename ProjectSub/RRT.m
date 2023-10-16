clearvars
close all
x_max = 1000;
y_max = 1000;

% Define the polygon obstacles
obstacle1 = [100 100 400 400; 400 600 600 400]';
obstacle2 = [600 800 800 600; 200 200 800 800]';
% Combine all obstacles into one matrix
obstacles = {obstacle1, obstacle2};

%Step size and max nodes
EPS = 30;
numNodes = 10000;

q_start.coord = [50 50];
q_start.cost = 0;
q_start.parent = 0;

q_goal.coord = [950 950];
q_goal.cost = 0;

nodes(1) = q_start;

figure(1)
axis([0 x_max 0 y_max])
for i = 1:length(obstacles)
    obstacle = obstacles{i};
    patch(obstacle(:,1), obstacle(:,2),[0 0 0]);
end
hold on


% Plot the start and goal coordinates as green circles
scatter(q_start.coord(1), q_start.coord(2), 100, 'g', 'filled');
scatter(q_goal.coord(1), q_goal.coord(2), 100, 'g', 'filled');

for i = 1:1:numNodes
    q_rand = [floor(rand(1)*x_max) floor(rand(1)*y_max)];
    
    % Pick the closest node from existing list to branch out from
    ndist = [];
    for j = 1:1:length(nodes)
        n = nodes(j);
        tmp = dist(n.coord, q_rand);
        ndist = [ndist tmp];
    end
    [val, idx] = min(ndist);
    q_near = nodes(idx);
    
    q_new.coord = steer(q_rand, q_near.coord, val, EPS);
    if Avoid(q_near.coord, q_new.coord, obstacles)
        line([q_near.coord(1), q_new.coord(1)], [q_near.coord(2), q_new.coord(2)], 'Color', 'b', 'LineWidth', 3);
        scatter(q_new.coord(1), q_new.coord(2), 30, 'b', 'filled');
        drawnow
        hold on
        q_new.cost = dist(q_new.coord, q_near.coord) + q_near.cost;
        
        % Update parent to least cost-from node
        q_new.parent = idx;
        
        % Append to nodes
        nodes = [nodes q_new];
        
        % Check if the goal node is reached
        if dist(q_new.coord, q_goal.coord) <= EPS
            q_goal.parent = length(nodes);
            nodes = [nodes q_goal];
            break
        end
    end
end

% Plot the final path
start = length(nodes);
path = q_goal.coord;
while nodes(start).parent ~= 0
    path = [nodes(start).coord; path];
    start = nodes(start).parent;
end
path = [q_start.coord; path];

% Plot the final path in red
line(path(:,1), path(:,2), 'Color', 'r', 'LineWidth', 3);
hold off

% Stop when goal is found

function d = dist(p1, p2)
% Euclidean distance between two points
d = norm(p1 - p2);
end


function q_new = steer(q_rand, q_near, d, EPS)
    % If the distance between q_rand and q_near is less than EPS, return q_rand
    if d < EPS
        q_new = q_rand;
    % Otherwise, steer towards q_rand from q_near
    else
        theta = atan2(q_rand(2)-q_near(2), q_rand(1)-q_near(1));
        q_new = q_near + EPS*[cos(theta) sin(theta)];
    end
end