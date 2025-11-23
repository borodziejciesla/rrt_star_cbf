function [path, pathLength, mean_h_path, rejection_ratio, iterations_number] = SafeCBFRRTStar(start, goal, occupancy, h_grid, dhx_grid, dhy_grid, params)
% RRT_STAR_SAFETY_CBF  RRT* planning with CBF safety check and weighted cost.
%
% INPUTS:
%   start, goal        : [x, y] (grid coordinates, x in [1..Nx], y in [1..Ny])
%   occupancy          : logical (Ny x Nx) matrix (1 = occupied)
%   h_grid             : Ny x Nx safety values (higher = safer)
%   dhx_grid, dhy_grid : Ny x Nx partial derivatives (∂h/∂x, ∂h/∂y)
%   params             : struct with fields:
%       .c (0..1)               weight between length and safety (default 0.7)
%       .maxIter                (default 2000)
%       .stepSize               (default 3)
%       .neighborRadius         (default 10)
%       .samplesPerEdge         (samples for collision & CBF checks, default 20)
%       .kappa                  (CBF parameter, default 5)
%       .gridX, .gridY          optional coordinate vectors (default 1:Nx,1:Ny)
%
% OUTPUT:
%   path : Kx2 array of points from start to goal (empty if not found)
%   pathLength : scalar, total Euclidean length of the returned path
%   mean_h_path: scalar, average h along the path (length-weighted). NaN if no path.

%% --- defaults & interpolants ---
if ~exist('params','var'), params = struct(); end
if ~isfield(params,'c'), params.c = 0.5; end
if ~isfield(params,'maxIter'), params.maxIter = 10000; end
if ~isfield(params,'stepSize'), params.stepSize = 2; end
if ~isfield(params,'neighborRadius'), params.neighborRadius = 10; end
if ~isfield(params,'samplesPerEdge'), params.samplesPerEdge = 20; end
if ~isfield(params,'kappa'), params.kappa = 5; end
% optional flags used elsewhere: do_cbf, v
if ~isfield(params,'do_cbf'), params.do_cbf = true; end
if ~isfield(params,'v'), params.v = 0; end

[Ny, Nx] = size(occupancy);
if ~isfield(params,'gridX'), params.gridX = 1:Nx; end
if ~isfield(params,'gridY'), params.gridY = 1:Ny; end
gridX = params.gridX;
gridY = params.gridY;

% interpolanty (griddedInterpolant expects {y,x} order)
Focc = griddedInterpolant({gridY, gridX}, double(occupancy), 'linear', 'nearest');
Fh   = griddedInterpolant({gridY, gridX}, double(h_grid),  'linear', 'nearest');
Fdhx = griddedInterpolant({gridY, gridX}, double(dhx_grid),'linear', 'nearest');
Fdhy = griddedInterpolant({gridY, gridX}, double(dhy_grid),'linear', 'nearest');

%% --- sanity checks ---
if any(start < [gridX(1), gridY(1)]) || any(start > [gridX(end), gridY(end)])
    error('Start out of grid bounds');
end
if any(goal < [gridX(1), gridY(1)]) || any(goal > [gridX(end), gridY(end)])
    error('Goal out of grid bounds');
end

%% --- RRT* data structures ---
nodes(1).pos   = start(:)';     % row [x y]
nodes(1).parent= 0;
nodes(1).cost  = 0;
nodes(1).id    = 1;

rng('shuffle');

found = false;
goalIdx = -1;

acceptance_number = 0;

for iter = 1:params.maxIter
    % sample
    if rand < 0.05
        p_rand = goal;
    else
        p_rand = [gridX(1) + rand*(gridX(end)-gridX(1)), gridY(1) + rand*(gridY(end)-gridY(1))];
    end

    % nearest neighbor
    dists = arrayfun(@(n) norm(n.pos - p_rand), nodes);
    [~, idx_near] = min(dists);
    p_near = nodes(idx_near).pos;

    % steer
    dir = p_rand - p_near;
    nd = norm(dir);
    if nd < 1e-9, continue; end
    dir = dir/nd;
    p_new = p_near + min(params.stepSize, nd) * dir;

    % collision and CBF check from p_near -> p_new
    if ~isCollisionFreeLine(p_near, p_new, Focc, params.samplesPerEdge)
        continue;
    end
    if ~isCBFSafeSegment(p_near, p_new, Fh, Fdhx, Fdhy, params.kappa, params.samplesPerEdge, params.do_cbf, params.v)
        continue;
    end

    % candidate cost from nearest
    mean_h = meanSampledH(p_near, p_new, Fh, params.samplesPerEdge);
    Jedge = edgeCostWeighted(p_near, p_new, mean_h, params.c, params.do_cbf);
    cost_from_near = nodes(idx_near).cost + Jedge;

    % find neighbors within radius
    neighbor_idx = find(arrayfun(@(n) norm(n.pos - p_new), nodes) <= params.neighborRadius);

    % find best parent among neighbors based on total cost
    best_parent = idx_near;
    best_cost = cost_from_near;
    for j = neighbor_idx
        pj = nodes(j).pos;
        if ~isCollisionFreeLine(pj, p_new, Focc, params.samplesPerEdge)
            continue;
        end
        if ~isCBFSafeSegment(pj, p_new, Fh, Fdhx, Fdhy, params.kappa, params.samplesPerEdge, params.do_cbf, params.v)
            continue;
        end
        mean_h_j = meanSampledH(pj, p_new, Fh, params.samplesPerEdge);
        Je = edgeCostWeighted(pj, p_new, mean_h_j, params.c, params.do_cbf);
        newCost = nodes(j).cost + Je;
        if newCost < best_cost
            best_cost = newCost;
            best_parent = j;
        end
    end

    % add node
    acceptance_number = acceptance_number + 1;
    newIdx = numel(nodes) + 1;
    nodes(newIdx).pos = p_new;
    nodes(newIdx).parent = best_parent;
    nodes(newIdx).cost = best_cost;
    nodes(newIdx).id = newIdx;

    % rewire neighbors
    for j = neighbor_idx
        pj = nodes(j).pos;
        if ~isCollisionFreeLine(p_new, pj, Focc, params.samplesPerEdge)
            continue;
        end
        if ~isCBFSafeSegment(p_new, pj, Fh, Fdhx, Fdhy, params.kappa, params.samplesPerEdge, params.do_cbf, params.v)
            continue;
        end
        mean_h_pj = meanSampledH(p_new, pj, Fh, params.samplesPerEdge);
        newCost = nodes(newIdx).cost + edgeCostWeighted(p_new, pj, mean_h_pj, params.c, params.do_cbf);
        if newCost < nodes(j).cost
            nodes(j).parent = newIdx;
            nodes(j).cost = newCost;
        end
    end

    % try connect to goal
    if norm(p_new - goal) <= params.stepSize
        if isCollisionFreeLine(p_new, goal, Focc, params.samplesPerEdge) && ...
           isCBFSafeSegment(p_new, goal, Fh, Fdhx, Fdhy, params.kappa, params.samplesPerEdge, params.do_cbf, params.v)
            mean_h_goal = meanSampledH(p_new, goal, Fh, params.samplesPerEdge);
            nodes(end+1).pos = goal;
            nodes(end).parent = newIdx;
            nodes(end).cost = nodes(newIdx).cost + edgeCostWeighted(p_new, goal, mean_h_goal, params.c, params.do_cbf);
            goalIdx = numel(nodes);
            found = true;
            break;
        end
    end
end

rejection_ratio = (iter - acceptance_number) / iter;
iterations_number = iter;

% reconstruct path
if ~found
    fprintf('RRT*: goal not reached in %d iter\n', params.maxIter);
    path = [];
    pathLength = 0;
    mean_h_path = NaN;
    return;
end

path = nodes(goalIdx).pos;
pidx = nodes(goalIdx).parent;
while pidx ~= 0
    path = [nodes(pidx).pos; path]; %#ok<AGROW>
    pidx = nodes(pidx).parent;
end

% compute path length and average h along path (length-weighted)
pathLength = 0;
h_weighted_sum = 0;
nSeg = size(path,1) - 1;
if nSeg <= 0
    % single-point path (start==goal)
    pathLength = 0;
    % evaluate h at single point
    hval = Fh(path(2), path(1)); %#ok<NASGU> % note griddedInterpolant expects (y,x)
    mean_h_path = max(0, min(1, hval));
else
    for i = 1:nSeg
        a = path(i,:);
        b = path(i+1,:);
        segLen = norm(b - a);
        pathLength = pathLength + segLen;

        if segLen > 0
            % sample along segment
            xs = linspace(a(1), b(1), params.samplesPerEdge);
            ys = linspace(a(2), b(2), params.samplesPerEdge);
            hvals = Fh(ys, xs);
            segMeanH = mean(hvals(:));
            h_weighted_sum = h_weighted_sum + segMeanH * segLen;
        end
    end
    if pathLength > 0
        mean_h_path = h_weighted_sum / pathLength;
        mean_h_path = max(0, min(1, mean_h_path)); % clamp to [0,1]
    else
        mean_h_path = NaN;
    end
end

end

%% ---------------- helper functions ----------------

function ok = isCollisionFreeLine(a, b, Focc, nSamples)
    xs = linspace(a(1), b(1), nSamples);
    ys = linspace(a(2), b(2), nSamples);
    occVals = Focc(ys, xs);
    ok = all(occVals <= 0.5); % occupied ~1
end

function ok = isCBFSafeSegment(a, b, Fh, Fdhx, Fdhy, kappa, nSamples, do_cbf, v)
    if do_cbf == false
        ok = true;
        return
    end
    % discrete check of CBF: for each sample p along a->b
    %    grad_h(p) dot (b-a) >= -kappa * h(p) * ||b-a||
    theta = atan2(b(2)-a(2), b(1)-a(1));
    dvec = b - a;
    L = norm(dvec);
    if L < 1e-9, ok = true; return; end
    xs = linspace(a(1), b(1), nSamples);
    ys = linspace(a(2), b(2), nSamples);
    hvals = Fh(ys, xs);
    dhx = Fdhx(ys, xs);
    dhy = Fdhy(ys, xs);
    % for k=1:numel(xs)
    %     grad_h = [dhx(k), dhy(k)];
    %     dotp = grad_h * dvec';
    %     rhs = -kappa * hvals(k) * L;
    %     if dotp < rhs - 1e-9
    %         ok = false; return;
    %     end
    % end
    h_prim = v*(dhx*cos(theta) + dhy*sin(theta));
    condition = (h_prim+kappa*hvals > 0);
    ok = all(condition);
end

function meanh = meanSampledH(a, b, Fh, nSamples)
    xs = linspace(a(1), b(1), nSamples);
    ys = linspace(a(2), b(2), nSamples);
    hvals = Fh(ys, xs);
    meanh = mean(hvals(:));
    % optional clamp to [0,1]
    meanh = max(0, min(1, meanh));
end

function J = edgeCostWeighted(a, b, mean_h, c, do_cbf)
    L = norm(b - a);
    if do_cbf
        J = c * L + (1 - c) * (1/mean_h); %(1 - mean_h);
    else
        J = L;
    end
end
