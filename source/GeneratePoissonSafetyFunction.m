function [h, dhdx, dhdy] = GeneratePoissonSafetyFunction(boundry, objects_list)
    %% Define boundries
    gd = boundry; % external boundry
    sf = "R1";
    ns = 'R1';
    % Add circle by circle
    for index = 1:length(objects_list)
        object = objects_list{index};
        ci = [1 object.c(1) object.c(2) object.r 0 0 0 0 0 0]';
        gd = [gd ci];
        ns = char(ns, ['C' num2str(index)]);
        sf = sf + " - C" + string(index);
    end
    ns = ns';

    % Define PDE solver
    N = 2;
    model = createpde(N);

    % Load geometry
    [g, ~] = decsg(gd, sf, ns);
    geometryFromEdges(model, g);

    % Define equation
    specifyCoefficients(model, 'm', 0, 'd', 0, 'c', 1, 'a', 0, 'f', [0;0]);

    % Set boundry conditions
    applyBoundaryCondition(model, ...
        'dirichlet', ...
        'Edge', 1:model.Geometry.NumEdges, ...
        'u', @bc_location);

    % --- Generowanie siatki
    generateMesh(model, 'Hmax', 0.01);

    % --- Rozwiązanie PDE
    result = solvepde(model);


    %% Solve Poisson equation for h, dhdx and dhdy
    ux = result.NodalSolution(:,1);
    uy = result.NodalSolution(:,2);
    f_nodes = sqrt(ux.^2 + uy.^2);

    Xnodes = model.Mesh.Nodes(1,:)';
    Ynodes = model.Mesh.Nodes(2,:)';

    Finterp = scatteredInterpolant(Xnodes, Ynodes, f_nodes, 'linear', 'nearest');

    % Define equation
    model_h = createpde(1);
    p = model.Mesh.Nodes;        % 2 x Nnodes
    t = model.Mesh.Elements;     % 3 x Nelements
    geometryFromMesh(model_h, p, t);

    specifyCoefficients(model_h, ...
        'm', 0, ...
        'd', 0, ...
        'c', -1, ...
        'a', 0, ...
        'f', @(location,state) f_for_toolbox(location, Finterp));

    function fq = f_for_toolbox(location, Finterp)
        xq = location.x; yq = location.y;
        fq_phys = Finterp(xq, yq);
        fq = -fq_phys(:)';
    end

    applyBoundaryCondition(model_h, ...
        'dirichlet', ...
        'Edge', 1:model_h.Geometry.NumEdges, ...
        'u', 0);

    result_h = solvepde(model_h);

    %% Convert to grids
    h = result_h.NodalSolution; % rozwiązanie w węzłach FEM
    [p, ~, ~] = meshToPet(model_h.Mesh);
    
    % Ekstrakcja współrzędnych węzłów
    x = p(1, :)';
    y = p(2, :)';
    
    % Ustal zakres siatki i liczbę punktów
    nx = 100;
    ny = 100;
    xq = linspace(min(x), max(x), nx);
    yq = linspace(min(y), max(y), ny);
    [Xq, Yq] = meshgrid(xq, yq);
    
    % Interpolacja rozwiązania h(x,y)
    h = griddata(x, y, h, Xq, Yq, 'linear');
    
    [dhdx, dhdy] = gradient(h, xq, yq);
end


%% Boundry condition on objects
function bcval = bc_location(location, ~)
    % location.x, location.y to wektory współrzędnych punktów na brzegu
    x = location.x;
    y = location.y;
    np = numel(x);
    bcval = zeros(2, np);   % każdy punkt: [u_x; u_y]
    
    % Parametry (muszą być zgodne z tymi w skrypcie wyżej)
    c1 = [0.3, 0.5]; r1 = 0.1;
    c2 = [0.7, 0.5]; r2 = 0.1;
    tol = 1e-6;
    
    for k = 1:np
        dx1 = x(k)-c1(1); dy1 = y(k)-c1(2);
        d1 = sqrt(dx1^2 + dy1^2);
        dx2 = x(k)-c2(1); dy2 = y(k)-c2(2);
        d2 = sqrt(dx2^2 + dy2^2);
        
        if d1 <= r1 + 1e-6
            % punkt leży na brzegu pierwszego otworu
            bcval(:,k) = 1 * [x(k)-c1(1); y(k)-c1(2)];          % przykładowa wartość
        elseif d2 <= r2 + 1e-6
            % punkt na brzegu drugiego otworu
            bcval(:,k) = 1 * [x(k)-c2(1); y(k)-c2(2)];        % przykładowa wartość
        else
            % punkt na zewnętrznym brzegu (zewnętrzna ramka)
            bcval(:,k) = [0; 0];
        end
    end
end
