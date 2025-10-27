%% Define external boundry
boundry = [3 4 0 1 1 0 0 0 1 1]';

%% Define objects
objects_list = {};

objects_list{1} = struct('c', [0.3 0.5], 'r', 0.15);
objects_list{2} = struct('c', [0.7 0.5], 'r', 0.15);
objects_list{3} = struct('c', [0.5 0.8], 'r', 0.05);

%% Generate h, dhx and dhy
[h, dhdx, dhdy] = GeneratePoissonSafetyFunction(boundry, objects_list);

%% Run Safe CBF-RRT
occupancy_map = h <= 0;

[Ny, Nx] = size(occupancy_map);
grid_x = 1:Nx;
grid_y = 1:Ny;

start = [10, 10];
goal = [50, 70];

% Run without CBF
params = struct( ...
    "maxIter", 5000, ...
    "v", 2, ...
    "stepSize", 1, ...
    "kappa",50, ...
    "c", 0.01, ...
    "do_cbf", false ...
);

without_cbf = figure();
hold on; grid on;
imagesc(grid_x, grid_y, ~occupancy_map);
axis xy;
axis equal;
colormap(gray);
xlim([1, 100]);
ylim([1, 100]);
plot(start(1), start(2), 'go', 'MarkerFaceColor', 'g');
plot(goal(1), goal(2), 'bo', 'MarkerFaceColor', 'b');
title('RRT* without CBF safety'); drawnow;

for i = 1:50
    path = SafeCBFRRTStar(start, goal, occupancy_map, h, dhdx, dhdy, params);
    figure(without_cbf);
    plot(path(:,1), path(:, 2), 'r');
end

% Run with CBF
params = struct( ...
    "maxIter", 5000, ...
    "v", 2, ...
    "stepSize", 1, ...
    "kappa", 20, ...
    "c", 0.1, ...
    "do_cbf", true ...
);

with_cbf = figure();
hold on; grid on;
imagesc(grid_x, grid_y, ~occupancy_map);
axis xy;
axis equal;
colormap(gray);
xlim([1, 100]);
ylim([1, 100]);
plot(start(1), start(2), 'go', 'MarkerFaceColor', 'g');
plot(goal(1), goal(2), 'bo', 'MarkerFaceColor', 'b');
title('RRT* with CBF safety'); drawnow;

for i = 1:50
    path = SafeCBFRRTStar(start, goal, occupancy_map, h, dhdx, dhdy, params);
    figure(with_cbf);
    plot(path(:,1), path(:, 2), 'r');
end