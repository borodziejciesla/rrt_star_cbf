function rrtstar_demo(do_cbf)
% RRTSTAR_DEMO  Step-by-step RRT* demo using repository MATLAB sources.
%   rrtstar_demo(do_cbf) runs an interactive demo. If do_cbf is true,
%   the safety function h and its gradient are used to filter edges and
%   an h-grid overlay is shown.

if nargin < 1, do_cbf = false; end

addpath('source');

%% Environment (reuse example.m setup)
boundry = [3 4 0 1 1 0 0 0 1 1]';
rng(2);%(88007287);
objects_list = {};
for index = 1:40
    objects_list{index} = struct('c', rand(2, 1), 'r', 0.06);
end

[h, dhdx, dhdy] = GeneratePoissonSafetyFunction(boundry, objects_list);

occupancy_map = h <= 0;
[Ny, Nx] = size(occupancy_map);
grid_x = 1:Nx; grid_y = 1:Ny;

start = [10, 10];
goal  = [90, 90];

%% Interpolants
Focc = griddedInterpolant({grid_y, grid_x}, double(occupancy_map), 'linear', 'nearest');
Fh   = griddedInterpolant({grid_y, grid_x}, double(h), 'linear', 'nearest');
Fdhx = griddedInterpolant({grid_y, grid_x}, double(dhdx), 'linear', 'nearest');
Fdhy = griddedInterpolant({grid_y, grid_x}, double(dhdy), 'linear', 'nearest');

%% Planner params
params.maxIter = 3000;
params.stepSize = 2;
params.neighborRadius = 10;
params.samplesPerEdge = 12;
params.kappa = 50;
params.c = ~do_cbf;
params.do_cbf = do_cbf;

%% Figure
fig = figure('Name','RRT* step-by-step demo');
ax = axes(fig);
imagesc(ax, grid_x, grid_y, ~occupancy_map); axis xy; axis equal; colormap(gray); hold on;
% define dark green for final path and start marker
darkGreen = [0, 0.4, 0];
% save animation as GIF frame-by-frame
gifFile = fullfile(pwd, 'rrtstar_demo.gif');
frameCount = 1;

function saveGifFrame(figHandle, gifFile, frameCount)
    frame = getframe(figHandle);
    im = frame2im(frame);
    [imind, cm] = rgb2ind(im, 256);
    if frameCount == 1
        imwrite(imind, cm, gifFile, 'gif', 'Loopcount', Inf, 'DelayTime', 0.08);
    else
        imwrite(imind, cm, gifFile, 'gif', 'WriteMode', 'append', 'DelayTime', 0.08);
    end
end
% start: square marker, filled dark green
plot(start(1), start(2), 's', 'MarkerEdgeColor', darkGreen, 'MarkerFaceColor', darkGreen, 'MarkerSize', 8);
% goal: triangle marker, filled red
plot(goal(1), goal(2), '^', 'MarkerEdgeColor', [0.8, 0, 0], 'MarkerFaceColor', [0.8, 0, 0], 'MarkerSize', 9);

% h-grid overlay
if do_cbf
    nx = 200; ny = 200;
    xs = linspace(grid_x(1), grid_x(end), nx);
    ys = linspace(grid_y(1), grid_y(end), ny);
    [Xq, Yq] = meshgrid(xs, ys);
    H = zeros(ny, nx);
    for i = 1:ny
        for j = 1:nx
            H(i,j) = Fh(ys(i), xs(j));
        end
    end
    cf = contourf(ax, xs, ys, H, 40, 'LineStyle','none');
    colormap(ax, jet);
    % set semi-transparency robustly on contour children (compat across MATLAB versions)
    try
        ch = get(cf, 'Children');
    catch
        ch = cf;
    end
    for k = 1:numel(ch)
        try
            if isprop(ch(k), 'FaceAlpha')
                set(ch(k), 'FaceAlpha', 0.45);
            end
        catch
        end
    end
    % redraw start/goal markers on top of the h-field so they remain visible
    plot(start(1), start(2), 's', 'MarkerEdgeColor', darkGreen, 'MarkerFaceColor', darkGreen, 'MarkerSize', 8);
    plot(goal(1), goal(2), '^', 'MarkerEdgeColor', [0.8, 0, 0], 'MarkerFaceColor', [0.8, 0, 0], 'MarkerSize', 9);
    % add colorbar and set label compatibly across MATLAB versions
    try
        cb = colorbar(ax);
        try
            cb.Label.String = 'h value';
        catch
            % older MATLAB versions may not allow Label assignment
            try
                ylabel(cb, 'h value');
            catch
                % silently continue if labeling fails
            end
        end
    catch
        % fallback: attempt legacy colorbar call
        colorbar('peer', ax);
    end
end

%% Initialize tree
nodes(1).pos = start(:)'; nodes(1).parent = 0; nodes(1).cost = 0;
found = false; goalIdx = -1;

drawnow;

for iter = 1:params.maxIter
    % sample
    if rand < 0.05
        p_rand = goal;
    else
        p_rand = [grid_x(1) + rand*(grid_x(end)-grid_x(1)), grid_y(1) + rand*(grid_y(end)-grid_y(1))];
    end

    % nearest
    dists = arrayfun(@(n) norm(n.pos - p_rand), nodes);
    [~, idx_near] = min(dists);
    p_near = nodes(idx_near).pos;

    % steer
    dir = p_rand - p_near; nd = norm(dir); if nd < 1e-9, continue; end
    dir = dir/nd; p_new = p_near + min(params.stepSize, nd) * dir;

    % collision check
    if ~isCollisionFreeLine(p_near, p_new, Focc, params.samplesPerEdge)
        continue;
    end
    % CBF check
    if ~isCBFSafeSegment(p_near, p_new, Fh, Fdhx, Fdhy, params.kappa, params.samplesPerEdge, params.do_cbf, 1)
        continue;
    end

    % candidate cost from nearest
    mean_h = meanSampledH(p_near, p_new, Fh, params.samplesPerEdge);
    Jedge = edgeCostWeighted(p_near, p_new, mean_h, params.c, params.do_cbf);
    cost_from_near = nodes(idx_near).cost + Jedge;

    % neighbors
    neighbor_idx = find(arrayfun(@(n) norm(n.pos - p_new), nodes) <= params.neighborRadius);

    % best parent among neighbors
    best_parent = idx_near; best_cost = cost_from_near;
    for j = neighbor_idx
        pj = nodes(j).pos;
        if ~isCollisionFreeLine(pj, p_new, Focc, params.samplesPerEdge), continue; end
        if ~isCBFSafeSegment(pj, p_new, Fh, Fdhx, Fdhy, params.kappa, params.samplesPerEdge, params.do_cbf, 1), continue; end
        mean_h_j = meanSampledH(pj, p_new, Fh, params.samplesPerEdge);
        Je = edgeCostWeighted(pj, p_new, mean_h_j, params.c, params.do_cbf);
        newCost = nodes(j).cost + Je;
        if newCost < best_cost
            best_cost = newCost; best_parent = j;
        end
    end

    % add node and draw
    newIdx = numel(nodes) + 1;
    nodes(newIdx).pos = p_new; nodes(newIdx).parent = best_parent; nodes(newIdx).cost = best_cost;
    % draw edge (tree branch)
    a = nodes(newIdx).pos; b = nodes(nodes(newIdx).parent).pos;
    plot([a(1), b(1)], [a(2), b(2)], 'r-', 'LineWidth', 0.6);
    xlim([0,100])
    ylim([0,100])
    drawnow limitrate;
    % save every few frames to GIF
    if mod(iter, 5) == 0 || found
        saveGifFrame(fig, gifFile, frameCount);
        frameCount = frameCount + 1;
    end;

    % rewire
    for j = neighbor_idx
        pj = nodes(j).pos;
        if ~isCollisionFreeLine(p_new, pj, Focc, params.samplesPerEdge), continue; end
        if ~isCBFSafeSegment(p_new, pj, Fh, Fdhx, Fdhy, params.kappa, params.samplesPerEdge, params.do_cbf, 1), continue; end
        mean_h_pj = meanSampledH(p_new, pj, Fh, params.samplesPerEdge);
        newCost = nodes(newIdx).cost + edgeCostWeighted(p_new, pj, mean_h_pj, params.c, params.do_cbf);
        if newCost < nodes(j).cost
            nodes(j).parent = newIdx; nodes(j).cost = newCost;
        end
    end

    % try connect to goal
    if norm(p_new - goal) <= params.stepSize
        if isCollisionFreeLine(p_new, goal, Focc, params.samplesPerEdge) && isCBFSafeSegment(p_new, goal, Fh, Fdhx, Fdhy, params.kappa, params.samplesPerEdge, params.do_cbf, 1)
            nodes(end+1).pos = goal; nodes(end).parent = newIdx; nodes(end).cost = nodes(newIdx).cost + edgeCostWeighted(p_new, goal, meanSampledH(p_new, goal, Fh, params.samplesPerEdge), params.c, params.do_cbf);
            goalIdx = numel(nodes); found = true; break;
        end
    end
end

%% Finalize
if ~found
    fprintf('Demo: goal not reached in %d iterations\n', params.maxIter);
else
    % reconstruct path
    path = nodes(goalIdx).pos; pidx = nodes(goalIdx).parent;
    while pidx ~= 0
        path = [nodes(pidx).pos; path]; %#ok<AGROW>
        pidx = nodes(pidx).parent;
    end
    % final path in dark green
    plot(path(:,1), path(:,2), 'Color', darkGreen, 'LineWidth', 2);
    plot(path(:,1), path(:,2), 'o', 'MarkerEdgeColor', darkGreen, 'MarkerFaceColor', darkGreen, 'MarkerSize', 6);
    title(sprintf('RRT* demo (iterations=%d)', iter));
    drawnow;
    saveGifFrame(fig, gifFile, frameCount);
    fprintf('Saved animation to %s\n', gifFile);
end

end

%% ---------------- helper functions (copied from SafeCBFRRTStar) ----------------
function ok = isCollisionFreeLine(a, b, Focc, nSamples)
    xs = linspace(a(1), b(1), nSamples);
    ys = linspace(a(2), b(2), nSamples);
    occVals = Focc(ys, xs);
    ok = all(occVals <= 0.5);
end

function ok = isCBFSafeSegment(a, b, Fh, Fdhx, Fdhy, kappa, nSamples, do_cbf, v)
    if do_cbf == false
        ok = true; return;
    end
    dvec = b - a; L = norm(dvec); if L < 1e-9, ok = true; return; end
    theta = atan2(b(2)-a(2), b(1)-a(1));
    xs = linspace(a(1), b(1), nSamples);
    ys = linspace(a(2), b(2), nSamples);
    hvals = Fh(ys, xs);
    dhx = Fdhx(ys, xs);
    dhy = Fdhy(ys, xs);
    h_prim = v*(dhx*cos(theta) + dhy*sin(theta));
    condition = (h_prim + kappa*hvals > 0);
    ok = all(condition);
end

function meanh = meanSampledH(a, b, Fh, nSamples)
    xs = linspace(a(1), b(1), nSamples);
    ys = linspace(a(2), b(2), nSamples);
    hvals = Fh(ys, xs);
    meanh = mean(hvals(:));
    meanh = max(0, min(1, meanh));
end

function J = edgeCostWeighted(a, b, mean_h, c, do_cbf)
    L = norm(b - a);
    if do_cbf
        J = c * L + (1 - c) * (1/mean_h);
    else
        J = L;
    end
end
