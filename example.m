%% Define external boundry
boundry = [3 4 0 1 1 0 0 0 1 1]';

%% Define objects
objects_list = {};

objects_list{1} = struct('c', [0.3 0.5], 'r', 0.1);
objects_list{2} = struct('c', [0.7 0.5], 'r', 0.1);

%% Generate h, dhx and dhy
[h, dhdx, dhdy] = GeneratePoissonSafetyFunction(boundry, objects_list);

%% Run Safe CBF-RRT
occupancy_map = h <= 0;

start = [10, 10];
goal = [90, 70];

params = struct( ...
    "maxIter", 5000, ...
    "stepSize", 1, ...
    "kappa", 5, ...
    "c", 0 ...
);

path = SafeCBFRRTStar(start, goal, occupancy_map, h, dhdx, dhdy, params);