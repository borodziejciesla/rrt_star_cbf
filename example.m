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

rng(2);%(123);%2%123 -> 88007287

for index = 1:40
    objects_list{index} = struct('c', rand(2, 1), 'r', 0.06);% 0.08
end

%% Generate h, dhx and dhy
[h, dhdx, dhdy] = GeneratePoissonSafetyFunction(boundry, objects_list);

%% Prepare for running RRT*
occupancy_map = h <= 0;
distance_map = bwdist(occupancy_map, 'euclidean');

[Ny, Nx] = size(occupancy_map);
grid_x = 1:Nx;
grid_y = 1:Ny;

start = [10, 10];
goal = [90, 90];
N = 10;

%% Run without CBF
params = struct( ...
    "maxIter", 10000, ...
    "v", 1, ...
    "stepSize", 1, ...
    "kappa", 50, ...
    "c", 1.0, ...
    "do_cbf", false, ...
    "min_time", false ...
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

lengths = NaN(1, N);
average_h = zeros(1, N);
rejection_ratios = zeros(1, N);
iterations = zeros(1, N);
average_obstacle_distance_without = NaN(1, N);
minimum_obstacle_distance_without = NaN(1, N);
distance_profiles_without = NaN(100, N);
progress_percent = linspace(0, 100, 100);

for i = 1:N
    rng(i);
    [path, path_length, mean_h_path, rejection_ratio, iterations_number] ...
        = SafeCBFRRTStar(start, goal, occupancy_map, h, dhdx, dhdy, params);
    if isempty(path)
        continue;
    end
    figure(without_cbf);
    plot(path(:,1), path(:, 2), 'r');
    lengths(i) = path_length;
    average_h(i) = mean_h_path;
    rejection_ratios(i) = rejection_ratio;
    iterations(i) = iterations_number;
    [distance_profile, ~] = computeObstacleDistanceProfile(path, distance_map, grid_x, grid_y);
    average_obstacle_distance_without(i) = mean(distance_profile);
    minimum_obstacle_distance_without(i) = min(distance_profile);
    distance_profiles_without(:, i) = distance_profile;
end

average_length = mean(lengths, 'omitnan');
average_length_std = std(lengths, 'omitnan');
average_h_valur = mean(average_h);
average_h_std = std(average_h);
average_rejection_ratios = mean(rejection_ratios);
average_rejection_ratios_std = std(rejection_ratios);
average_iterations_number = mean(iterations);
average_obstacle_distance_without_val = mean(average_obstacle_distance_without, 'omitnan');
average_obstacle_distance_without_std = std(average_obstacle_distance_without, 'omitnan');

figure();
plot(progress_percent, mean(distance_profiles_without, 2, 'omitnan'), 'LineWidth', 1.5);
hold on;
grid on;
xlabel('Postęp trasy [%]');
ylabel('Odległość do najbliższej przeszkody');
title('Średnia odległość do najbliższej przeszkody wzdłuż trasy (bez CBF)');
drawnow;

%% Run with CBF
params = struct( ...
    "maxIter", 10000, ...
    "v", 1.0, ...
    "stepSize", 1, ...
    "kappa", 50, ...
    "c", 0.0, ...
    "do_cbf", true, ...
    "min_time", true ...
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

lengths_safe = NaN(1, N);
average_h_safe = zeros(1, N);
rejection_ratios_safe = zeros(1, N);
iterations_safe = zeros(1, N);
average_obstacle_distance_safe = NaN(1, N);
minimum_obstacle_distance_safe = NaN(1, N);
distance_profiles_safe = NaN(100, N);

for i = 1:N
    rng(i);
    [path, path_length, mean_h_path, rejection_ratio, iterations_number] ...
        = SafeCBFRRTStar(start, goal, occupancy_map, h, dhdx, dhdy, params);
    if isempty(path)
        continue;
    end
    figure(with_cbf);
    plot(path(:,1), path(:, 2), 'r');
    lengths_safe(i) = path_length;
    average_h_safe(i) = mean_h_path;
    rejection_ratios_safe(i) = rejection_ratio;
    iterations_safe(i) = iterations_number;
    [distance_profile, ~] = computeObstacleDistanceProfile(path, distance_map, grid_x, grid_y);
    average_obstacle_distance_safe(i) = mean(distance_profile);
    minimum_obstacle_distance_safe(i) = min(distance_profile);
    distance_profiles_safe(:, i) = distance_profile;
end

average_length_safe = mean(lengths_safe, 'omitnan');
average_length_std_safe = std(lengths_safe, 'omitnan');
average_h_valur_safe = mean(average_h_safe);
average_h_std_safe = std(average_h_safe);
average_rejection_ratios_safe = mean(rejection_ratios_safe);
average_rejection_ratios_std_safe = std(rejection_ratios_safe);
average_iterations_number_safe = mean(iterations_safe);
average_obstacle_distance_safe_val = mean(average_obstacle_distance_safe, 'omitnan');
average_obstacle_distance_safe_std = std(average_obstacle_distance_safe, 'omitnan');

%% Run with CBF without time optimization
params = struct( ...
    "maxIter", 10000, ...
    "v", 1.0, ...
    "stepSize", 1, ...
    "kappa", 50, ...
    "c", 0.0, ...
    "do_cbf", true, ...
    "min_time", false ...
);

with_cbf_without_time = figure();
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
title('RRT* with CBF safety (without time optimization)'); drawnow;

lengths_safe_without_time = NaN(1, N);
average_h_safe_without_time = zeros(1, N);
rejection_ratios_safe_without_time = zeros(1, N);
iterations_safe_without_time = zeros(1, N);
average_obstacle_distance_safe_without_time = NaN(1, N);
minimum_obstacle_distance_safe_without_time = NaN(1, N);
distance_profiles_safe_without_time = NaN(100, N);

for i = 1:N
    rng(i);
    [path, path_length, mean_h_path, rejection_ratio, iterations_number] ...
        = SafeCBFRRTStar(start, goal, occupancy_map, h, dhdx, dhdy, params);
    if isempty(path)
        continue;
    end
    figure(with_cbf_without_time);
    plot(path(:,1), path(:, 2), 'r');
    lengths_safe_without_time(i) = path_length;
    average_h_safe_without_time(i) = mean_h_path;
    rejection_ratios_safe_without_time(i) = rejection_ratio;
    iterations_safe_without_time(i) = iterations_number;
    [distance_profile, ~] = computeObstacleDistanceProfile(path, distance_map, grid_x, grid_y);
    average_obstacle_distance_safe_without_time(i) = mean(distance_profile);
    minimum_obstacle_distance_safe_without_time(i) = min(distance_profile);
    distance_profiles_safe_without_time(:, i) = distance_profile;
end

average_length_safe_without_time = mean(lengths_safe_without_time, 'omitnan');
average_length_std_safe_without_time = std(lengths_safe_without_time, 'omitnan');
average_h_value_safe_without_time = mean(average_h_safe_without_time);
average_h_std_safe_without_time = std(average_h_safe_without_time);
average_rejection_ratios_safe_without_time = mean(rejection_ratios_safe_without_time);
average_rejection_ratios_std_safe_without_time = std(rejection_ratios_safe_without_time);
average_iterations_number_safe_without_time = mean(iterations_safe_without_time);
average_obstacle_distance_safe_without_time_val = mean(average_obstacle_distance_safe_without_time, 'omitnan');
average_obstacle_distance_safe_without_time_std = std(average_obstacle_distance_safe_without_time, 'omitnan');

%% Compare results
figure();
hold on;
grid on;
plot(progress_percent, mean(distance_profiles_safe, 2, 'omitnan'), 'LineWidth', 1.5);
plot(progress_percent, mean(distance_profiles_safe_without_time, 2, 'omitnan'), 'LineWidth', 1.5);
plot(progress_percent, mean(distance_profiles_without, 2, 'omitnan'), 'LineWidth', 1.5);
xlabel('Path progress [%]');
ylabel('Distance to closest obstacle');
title('Average distance to closest obstacle');
legend('CBF with time optimization', 'CBF without time optimization', 'Without CBF', ...
    'Location', 'best');
drawnow;

all_path_lengths = [lengths(:); lengths_safe(:); lengths_safe_without_time(:)];
all_path_lengths = all_path_lengths(isfinite(all_path_lengths));
figure();
hold on;
grid on;
if isempty(all_path_lengths)
    title('Path length distribution (no successful paths)');
else
    [~, path_length_bin_edges] = histcounts(all_path_lengths, 'BinMethod', 'sqrt');
    histogram(lengths, 'BinEdges', path_length_bin_edges, 'DisplayStyle', 'bar', ...
        'LineWidth', 1.5, 'DisplayName', 'Without CBF');
    histogram(lengths_safe, 'BinEdges', path_length_bin_edges, 'DisplayStyle', 'bar', ...
        'LineWidth', 1.5, 'DisplayName', 'CBF with time optimization');
    histogram(lengths_safe_without_time, 'BinEdges', path_length_bin_edges, ...
        'DisplayStyle', 'bar', 'LineWidth', 1.5, ...
        'DisplayName', 'CBF without time optimization');
    xlabel('Path length');
    ylabel('Number of trials');
    title('Path length distribution');
    legend('Location', 'best');
end
drawnow;

minimum_distance_means = [ ...
    mean(minimum_obstacle_distance_safe, 'omitnan'), ...
    mean(minimum_obstacle_distance_safe_without_time, 'omitnan'), ...
    mean(minimum_obstacle_distance_without, 'omitnan') ...
];
minimum_distance_stds = [ ...
    std(minimum_obstacle_distance_safe, 'omitnan'), ...
    std(minimum_obstacle_distance_safe_without_time, 'omitnan'), ...
    std(minimum_obstacle_distance_without, 'omitnan') ...
];
figure();
bar(minimum_distance_means);
hold on;
grid on;
errorbar(1:3, minimum_distance_means, minimum_distance_stds, 'k.', 'LineWidth', 1.2);
xticks(1:3);
xticklabels({'CBF with time optimization', 'CBF without time optimization', 'Without CBF'});
ylabel('Minimum distance to closest obstacle');
title('Minimum obstacle distance comparison');
drawnow;

fprintf('Without CBF: Average distance to closest obstacle = %.3f +/- %.3f\n', ...
    average_obstacle_distance_without_val, average_obstacle_distance_without_std);
fprintf('With CBF: Average distance to closest obstacle = %.3f +/- %.3f\n', ...
    average_obstacle_distance_safe_val, average_obstacle_distance_safe_std);
fprintf('With CBF (without time optimization): Average distance to closest obstacle = %.3f +/- %.3f\n', ...
    average_obstacle_distance_safe_without_time_val, average_obstacle_distance_safe_without_time_std);
fprintf('Without CBF: Average path length = %.3f +/- %.3f\n', ...
    average_length, average_length_std);
fprintf('With CBF: Average path length = %.3f +/- %.3f\n', ...
    average_length_safe, average_length_std_safe);
fprintf('With CBF (without time optimization): Average path length = %.3f +/- %.3f\n', ...
    average_length_safe_without_time, average_length_std_safe_without_time);
fprintf('Minimum distance to closest obstacle (CBF with time optimization) = %.3f +/- %.3f\n', ...
    minimum_distance_means(1), minimum_distance_stds(1));
fprintf('Minimum distance to closest obstacle (CBF without time optimization) = %.3f +/- %.3f\n', ...
    minimum_distance_means(2), minimum_distance_stds(2));
fprintf('Minimum distance to closest obstacle (without CBF) = %.3f +/- %.3f\n', ...
    minimum_distance_means(3), minimum_distance_stds(3));

function [distance_profile, progress_percent] = computeObstacleDistanceProfile(path, distance_map, grid_x, grid_y)
    progress_percent = linspace(0, 100, 100);
    if isempty(path) || size(path, 1) < 2
        distance_profile = NaN(100, 1);
        return;
    end

    cumulative_length = [0; cumsum(sqrt(sum(diff(path, 1, 1).^2, 2)))];
    total_length = cumulative_length(end);

    if total_length <= 0
        distance_profile = NaN(100, 1);
        return;
    end

    target_length = total_length * linspace(0, 1, 100);
    x_profile = interp1(cumulative_length, path(:, 1), target_length, 'linear');
    y_profile = interp1(cumulative_length, path(:, 2), target_length, 'linear');

    Fdist = griddedInterpolant({grid_y, grid_x}, double(distance_map), 'linear', 'nearest');
    distance_profile = Fdist(y_profile, x_profile);
    distance_profile = max(0, distance_profile);
    distance_profile = distance_profile(:);
end