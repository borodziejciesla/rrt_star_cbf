addpath source

%% Define external boundry
boundry = [3 4 0 1 1 0 0 0 1 1]';

%% Define objects
objects_list = {};

% objects_list{1} = struct('c', [0.3 0.5], 'r', 0.15);
% objects_list{2} = struct('c', [0.7 0.5], 'r', 0.15);
% objects_list{3} = struct('c', [0.5 0.8], 'r', 0.05);
% objects_list{4} = struct('c', [0.5 0.2], 'r', 0.05);
% objects_list{5} = struct('c', [0.2 0.2], 'r', 0.05);

rng(123);%2%123

for index = 1:15
    objects_list{index} = struct('c', rand(2, 1), 'r', 0.08);% 0.08
end

%% Generate h, dhx and dhy
[h, dhdx, dhdy] = GeneratePoissonSafetyFunction(boundry, objects_list);

%% Run Safe CBF-RRT
occupancy_map = h <= 0;

[Ny, Nx] = size(occupancy_map);
grid_x = 1:Nx;
grid_y = 1:Ny;

start = [10, 10];
goal = [90, 90];
N = 100;

% Run without CBF
params = struct( ...
    "maxIter", 10000, ...
    "v", 1, ...
    "stepSize", 2, ...
    "kappa", 50, ...
    "c", 0.01, ...
    "do_cbf", false ...
);

without_cbf = figure();
hold on; grid on;
imagesc(grid_x, grid_y, ~occupancy_map);
axis xy;
axis equal;
xlabel('x');
ylabel('y');
colormap(gray);
xlim([1, 100]);
ylim([1, 100]);
plot(start(1), start(2), 'go', 'MarkerFaceColor', 'g');
plot(goal(1), goal(2), 'bo', 'MarkerFaceColor', 'b');
title('RRT* without CBF safety'); drawnow;

lengths = zeros(1, N);
average_h = zeros(1, N);
rejection_ratios = zeros(1, N);
iterations = zeros(1, N);

for i = 1:N
    rng(i);
    [path, path_length, mean_h_path, rejection_ratio, iterations_number] ...
        = SafeCBFRRTStar(start, goal, occupancy_map, h, dhdx, dhdy, params);
    figure(without_cbf);
    plot(path(:,1), path(:, 2), 'r');
    lengths(i) = path_length;
    average_h(i) = mean_h_path;
    rejection_ratios(i) = rejection_ratio;
    iterations(i) = iterations_number;
end

average_length = mean(lengths);
average_length_std = std(lengths);
average_h_valur = mean(average_h);
average_h_std = std(average_h);
average_rejection_ratios = mean(rejection_ratios);
average_rejection_ratios_std = std(rejection_ratios);
average_iterations_number = mean(iterations);

%% Run with CBF
params = struct( ...
    "maxIter", 10000, ...
    "v", 0.5, ...
    "stepSize", 2, ...
    "kappa", 10, ...
    "c", 0.0, ...
    "do_cbf", true ...
);

with_cbf = figure();
hold on; grid on;
imagesc(grid_x, grid_y, ~occupancy_map);
axis xy;
axis equal;
xlabel('x');
ylabel('y');
colormap(gray);
xlim([1, 100]);
ylim([1, 100]);
plot(start(1), start(2), 'go', 'MarkerFaceColor', 'g');
plot(goal(1), goal(2), 'bo', 'MarkerFaceColor', 'b');
title('RRT* with CBF safety'); drawnow;

lengths_safe = zeros(1, N);
average_h_safe = zeros(1, N);
rejection_ratios_safe = zeros(1, N);
iterations_safe = zeros(1, N);

for i = 1:N
    rng(i);
    [path, path_length, mean_h_path, rejection_ratio, iterations_number] ...
        = SafeCBFRRTStar(start, goal, occupancy_map, h, dhdx, dhdy, params);
    figure(with_cbf);
    plot(path(:,1), path(:, 2), 'r');
    lengths_safe(i) = path_length;
    average_h_safe(i) = mean_h_path;
    rejection_ratios_safe(i) = rejection_ratio;
    iterations_safe(i) = iterations_number;
end

average_length_safe = mean(lengths_safe);
average_length_std_safe = std(lengths_safe);
average_h_valur_safe = mean(average_h_safe);
average_h_std_safe = std(average_h_safe);
average_rejection_ratios_safe = mean(rejection_ratios_safe);
average_rejection_ratios_std_safe = std(rejection_ratios_safe);
average_iterations_number_safe = mean(iterations_safe);