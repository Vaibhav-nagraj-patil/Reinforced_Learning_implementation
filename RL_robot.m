clear; clc;

% Define environment
gridSize = 10; % Size of the grid
maze = zeros(gridSize); % Empty grid
maze(2:3, 4) = -1; % Example obstacle
maze(6, 7:9) = -1; % Another obstacle
maze(8, 3) = -1; % Additional obstacle
actions = [0 1; 0 -1; -1 0; 1 0]; % [right, left, up, down]

% Parameters
explorationTime = 5; % Exploration time in seconds
start = [randi(gridSize), randi(gridSize)];
pos = start;
visited = zeros(gridSize);
visited(pos(1), pos(2)) = 1;
path = pos; % Store path

% Visualization setup
figure(1);
imagesc(maze);
colormap([1 1 0; 0 0 0; 0 0 0]); % Yellow = grid, Black = obstacles
hold on;
plot(pos(2), pos(1), 'ro', 'MarkerSize', 15, 'MarkerFaceColor', 'r');
hold off;
title('Exploring the Grid...');
drawnow;

% Timer
startTime = tic;

while toc(startTime) < explorationTime
    % Find valid directions
    possibleDirections = findValidDirections(pos, visited, maze, actions);

    if isempty(possibleDirections)
        % Backtrack
        path = backtrackToUnexplored(path, visited, maze, actions);
        pos = path(end, :);
    else
        % Random move
        directionIdx = randi(length(possibleDirections));
        direction = actions(possibleDirections(directionIdx), :);
        pos = pos + direction;
        visited(pos(1), pos(2)) = 1;
        path = [path; pos];
    end

    % Visualization
    figure(1);
    hold on;
    plot(path(:, 2), path(:, 1), 'b.-', 'MarkerSize', 10, 'LineWidth', 1.5);
    plot(pos(2), pos(1), 'ro', 'MarkerSize', 15, 'MarkerFaceColor', 'r');
    pause(0.2);
    hold off;
end

% Return to start
shortestPath = findShortestPath(pos, start, maze);
for i = 2:size(shortestPath, 1)
    pos = shortestPath(i, :);
    figure(1);
    hold on;
    plot(shortestPath(:, 2), shortestPath(:, 1), 'g.-', 'MarkerSize', 10, 'LineWidth', 1.5);
    plot(pos(2), pos(1), 'ro', 'MarkerSize', 15, 'MarkerFaceColor', 'r');
    pause(0.2);
    hold off;
end
title('Robot Returned to Start');

% Function definitions
function validDirs = findValidDirections(pos, visited, maze, actions)
    validDirs = [];
    for dirIdx = 1:4
        nextPos = pos + actions(dirIdx, :);
        if nextPos(1) >= 1 && nextPos(1) <= size(maze, 1) && ...
           nextPos(2) >= 1 && nextPos(2) <= size(maze, 2) && ...
           maze(nextPos(1), nextPos(2)) ~= -1 && ...
           visited(nextPos(1), nextPos(2)) == 0
            validDirs = [validDirs, dirIdx];
        end
    end
end

function path = backtrackToUnexplored(path, visited, maze, actions)
    while ~isempty(path)
        pos = path(end, :);
        possibleDirections = findValidDirections(pos, visited, maze, actions);
        if ~isempty(possibleDirections)
            return;
        end
        path(end, :) = [];
    end
end

function shortestPath = findShortestPath(start, goal, maze)
    queue = {start};
    parentMap = containers.Map();
    parentMap(mat2str(start)) = [];

    while ~isempty(queue)
        current = queue{1};
        queue(1) = [];
        if isequal(current, goal)
            shortestPath = reconstructPath(parentMap, goal);
            return;
        end

        for action = [0 1; 0 -1; -1 0; 1 0]'
            nextPos = current + action';
            if nextPos(1) >= 1 && nextPos(1) <= size(maze, 1) && ...
               nextPos(2) >= 1 && nextPos(2) <= size(maze, 2) && ...
               maze(nextPos(1), nextPos(2)) ~= -1 && ...
               ~isKey(parentMap, mat2str(nextPos))
                queue{end + 1} = nextPos;
                parentMap(mat2str(nextPos)) = current;
            end
        end
    end
    shortestPath = [];
end

function path = reconstructPath(parentMap, goal)
    path = goal;
    while ~isempty(parentMap(mat2str(path(1, :))))
        path = [parentMap(mat2str(path(1, :))); path];
    end
end
