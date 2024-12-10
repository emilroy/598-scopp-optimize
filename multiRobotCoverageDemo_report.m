% voronoi partition feild based controller
function multiRobotCoverageDemo4()
    close all;clear all;
    % Parameters
    boundary = [30.2472, -92.151; 30.247, -92.1426; 30.2464, -92.1427; 
                30.2418, -92.1472; 30.243, -92.1501; 30.245, -92.1516];
    obstacles = {[30.244, -92.146; 30.244, -92.148; 
                  30.245, -92.148; 30.245, -92.146]};
    cell_size = 0.0001; % for discretization
    num_robots = 6;      % number of robots
    robot_speed = 20;    % speed (units per sec)
    sampling_rate = 1/10000; % sampling rate

    % starting positions (distinct)
    start_positions = [30.248, -92.147; 30.242, -92.1465; 30.245, -92.15;
                       30.2465, -92.1455; 30.246, -92.145; 30.242, -92.149];

    % Multi-robot coverage with animation
    multiRobotCoverage(num_robots, boundary, obstacles, cell_size, robot_speed, start_positions, sampling_rate);

    % generate plots 
    generate3DPlots(boundary, obstacles, cell_size);

    %print licenses used
    license('inuse')
end

function multiRobotCoverage(num_robots, boundary, obstacles, cell_size, robot_speed, start_positions, sampling_rate)
    % goal as the centroid of the boundary (optional)
    goal = [mean(boundary(:, 1)), mean(boundary(:, 2))];

    % step 1: Discretize the workspace
    grid_points = discretizeArea(boundary, obstacles, cell_size);

    % validate grid_points
    if any(~isfinite(grid_points), 'all')
        error('Grid points contain invalid values (NaN or Inf). Check discretization logic.');
    end

    % step 2: workspace partition using Voronoi diagram
    cluster_indices = voronoiPartition(grid_points, start_positions, boundary);

    % check if any cluster has been assigned
    if all(cluster_indices == 0)
        error('No grid points were assigned to any cluster. Check Voronoi partitioning.');
    end

    % check for empty clusters
    for i = 1:num_robots
        if sum(cluster_indices == i) == 0
            warning('Cluster %d has no assigned grid points. Consider adjusting cell_size or robot positions.', i);
        end
    end
    % visualize clusters (Voronoi regions)
    visualizeClusters(grid_points, cluster_indices, boundary, obstacles, start_positions);

    % step 3: Plan paths for each robot within its Voronoi region
    robot_paths = arrayfun(@(i) planPath(grid_points(cluster_indices == i, :), start_positions(i, :)), ...
                           1:num_robots, 'UniformOutput', false);

    % step 4: Animate robots following their paths
    [controller_data] = animateRobots(boundary, obstacles, robot_paths, start_positions, robot_speed, sampling_rate);

    % step 5: Visualize controller data
    % visualizeControllerData(controller_data);
end

function cluster_indices = voronoiPartition(grid_points, start_positions, boundary)
    % create polyshape for workspace boundary
    poly_boundary = polyshape(boundary(:,1), boundary(:,2));

    %  Delaunay triangulation and Voronoi diagram
    dt = delaunayTriangulation(start_positions);
    [V, R] = voronoiDiagram(dt);

    %  cluster indices
    cluster_indices = zeros(size(grid_points, 1), 1);

    % Iierate through each Voronoi region
    for j = 1:length(R)
        if all(R{j} > 0)
            vertices = V(R{j}, :);
            vertices = removeDuplicateVertices(vertices); % remove duplicates

            % validate polygon
            if isPolygonValid(vertices)
                try
                    poly_region = polyshape(vertices(:,1), vertices(:,2));
                catch ME
                    warning('Failed to create polyshape for region %d: %s', j, ME.message);
                    continue; 
                end

                % clip Voronoi region with workspace boundary
                clipped_region = intersect(poly_region, poly_boundary);

                % check clipped_region is empty
                if isempty(clipped_region)
                    continue;
                end
                % handle multiple regions after clipping
                num_subregions = clipped_region.NumRegions;
                for r = 1:num_subregions
                    sub_poly = clipped_region.Region(r);
                    % assign grid points inside the sub-region to cluster j
                    inside = isinterior(sub_poly, grid_points(:,1), grid_points(:,2));
                    cluster_indices(inside & cluster_indices == 0) = j;
                end
            else
                warning('Invalid polygon detected for region %d. Skipping.', j);
            end
        end
    end

    % handle any unassigned grid points by assigning them to the nearest cluster
    unassigned = cluster_indices == 0;
    if any(unassigned)
        fprintf('Assigning %d unassigned grid points to nearest clusters.\n', sum(unassigned));
        unassigned_points = grid_points(unassigned, :);
        for i = 1:size(unassigned_points, 1)
            distances = sqrt(sum((start_positions - unassigned_points(i, :)).^2, 2));
            [~, nearest_cluster] = min(distances);
            cluster_indices(find(unassigned, 1, 'first')) = nearest_cluster;
            unassigned = cluster_indices == 0;
        end
    end
end

% helper function: remove duplicate vertices
function vertices = removeDuplicateVertices(vertices)
    tolerance = 1e-9;
    [~, unique_idx] = unique(round(vertices / tolerance) * tolerance, 'rows');
    vertices = vertices(sort(unique_idx), :);
end

% helper function
function isValid = isPolygonValid(vertices)
    % Check polygon is valid 
    isValid = ~any(isnan(vertices), 'all') && ...
              ~any(isinf(vertices), 'all') && ...
              size(vertices, 1) >= 3; 
end

function grid_points = discretizeArea(boundary, obstacles, cell_size)
    % generate a grid of points
    [min_x, min_y, max_x, max_y] = boundingBox(boundary);
    [x, y] = meshgrid(min_x:cell_size:max_x, min_y:cell_size:max_y);
    if numel(x) < 4 || numel(y) < 4
        error('Grid size is too small. Increase the range or decrease the cell size.');
    end
    points = [x(:), y(:)];

    % filter points inside the boundary and outside obstacles
    poly_boundary = polyshape(boundary(:, 1), boundary(:, 2));
    inside = isinterior(poly_boundary, points(:, 1), points(:, 2));

    for k = 1:length(obstacles)
        poly_obstacle = polyshape(obstacles{k}(:, 1), obstacles{k}(:, 2));
        inside = inside & ~isinterior(poly_obstacle, points(:, 1), points(:, 2));
    end

    % output grid points
    grid_points = points(inside, :);
end

function path = planPath(partition, start_position)
    % plan a path using a nearest neighbor algorithm
    if isempty(partition)
        path = start_position;
        return;
    end

    path = start_position;
    remaining_points = partition;

    % include the start position
    if ~ismember(start_position, partition, 'rows')
        remaining_points = [start_position; remaining_points];
    end

    num_points = size(remaining_points, 1);
    visited = false(num_points, 1);
    visited(1) = true; %start position as visited
    current_index = 1; 

    for i = 1:(num_points - 1)
        current_point = remaining_points(current_index, :);
        %  distances to unvisited points
        distances = sqrt(sum((remaining_points(~visited, :) - current_point).^2, 2));
        [~, idx] = min(distances);
        % map idx back to the indices in remaining_points
        idx_global = find(~visited);
        next_index = idx_global(idx);
        %   next point to the path
        path = [path; remaining_points(next_index, :)];
        visited(next_index) = true;
        current_index = next_index;
    end
end

function [min_x, min_y, max_x, max_y] = boundingBox(polygon)
    % compute  bounding box of a polygon
    min_x = min(polygon(:, 1));
    min_y = min(polygon(:, 2));
    max_x = max(polygon(:, 1));
    max_y = max(polygon(:, 2));
end

function [controller_data] = animateRobots(workspaceBoundary, obstacles, robot_paths, start_positions, robot_speed, sampling_rate)
    figure; hold on;
    fill(workspaceBoundary(:, 1), workspaceBoundary(:, 2), [0.9, 0.9, 0.9], 'EdgeColor', 'k', 'LineWidth', 1.5);
    for k = 1:length(obstacles)
        fill(obstacles{k}(:, 1), obstacles{k}(:, 2), [0.7, 0.7, 0.7], 'EdgeColor', 'none');
    end
    colors = lines(length(robot_paths));

    % robot markers and paths
    robot_markers = gobjects(length(robot_paths), 1);
    robot_paths_lines = gobjects(length(robot_paths), 1);
    robot_trajectories = cell(length(robot_paths), 1);
    controller_data = repmat(struct('time', [], 'position', [], 'speed', [], 'path_deviation', []), length(robot_paths), 1);

    for i = 1:length(robot_paths)
        robot_markers(i) = plot(start_positions(i, 1), start_positions(i, 2), 'o', ...
                                'MarkerFaceColor', colors(i, :), 'MarkerEdgeColor', 'k', 'MarkerSize', 8);
        robot_paths_lines(i) = plot(NaN, NaN, '-', 'LineWidth', 1, 'Color', colors(i, :));
    end

    % time step for animation
    time = 0;
    tol = 0.001; 
    max_path_length = max(cellfun(@(x) size(x, 1), robot_paths));

    %  robot positions
    robot_positions = start_positions;

    while true
        all_robots_reached = true; 
        for i = 1:length(robot_paths)
            if ~isempty(robot_paths{i})
                all_robots_reached = false;
                % current position ad next waypoint
                current_position = robot_positions(i, :);
                next_waypoint = robot_paths{i}(1, :);

                %  direction and move towards waypoint
                direction_vector = next_waypoint - current_position;
                distance = norm(direction_vector);

                if distance < tol
                    robot_positions(i, :) = next_waypoint;
                    robot_paths{i}(1, :) = [];
                else
                    direction_unit = direction_vector / distance;
                    robot_positions(i, :) = current_position + direction_unit * robot_speed * sampling_rate;
                end
                set(robot_markers(i), 'XData', robot_positions(i, 1), 'YData', robot_positions(i, 2));
                if isempty(robot_trajectories{i})
                    robot_trajectories{i} = robot_positions(i, :);
                else
                    robot_trajectories{i} = [robot_trajectories{i}; robot_positions(i, :)];
                end
                set(robot_paths_lines(i), 'XData', robot_trajectories{i}(:, 1), 'YData', robot_trajectories{i}(:, 2));

                % controller data
                controller_data(i).time = [controller_data(i).time; time];
                controller_data(i).position = [controller_data(i).position; robot_positions(i, :)];
                controller_data(i).speed = [controller_data(i).speed; robot_speed];
                controller_data(i).path_deviation = [controller_data(i).path_deviation; norm(current_position - next_waypoint)];
            end
        end
        if all_robots_reached
            break;
        end

        %  time
        time = time + sampling_rate;
        drawnow;
        pause(sampling_rate / 10); % for visualization
    end

    %  visualization
    title('Multi-Robot Path Following with Waypoint Controller');
    xlabel('Longitude');
    ylabel('Latitude');
    axis equal;
    grid on;
    hold off;
end


function inside_obstacle = isInsideObstacle(position, obstacles)
    % check position is inside obstacles
    inside_obstacle = false;
    for k = 1:length(obstacles)
        poly_obstacle = polyshape(obstacles{k}(:,1), obstacles{k}(:,2));
        if isinterior(poly_obstacle, position(1), position(2))
            inside_obstacle = true;
            return;
        end
    end
end
function visualizeClusters(grid_points, cluster_indices, boundary, obstacles, start_positions)
    figure; hold on;

    % plot boundary and obstacles
    fill(boundary(:, 1), boundary(:, 2), [0.9, 0.9, 0.9], 'EdgeColor', 'k', 'LineWidth', 1.5);
    for k = 1:length(obstacles)
        fill(obstacles{k}(:, 1), obstacles{k}(:, 2), [0.7, 0.7, 0.7], 'EdgeColor', 'none');
    end

    colors = lines(max(cluster_indices));

    % plot each region with a unique color
    centroids = zeros(max(cluster_indices), 2); % store centroids
    for i = 1:max(cluster_indices)
        region_points = grid_points(cluster_indices == i, :);
        scatter(region_points(:, 1), region_points(:, 2), 36, colors(i, :), 'filled');
        
        % store the centroid of the Voronoi region
        if ~isempty(region_points)
            centroids(i, :) = mean(region_points, 1);
            plot(centroids(i, 1), centroids(i, 2), 'kx', 'LineWidth', 2, 'MarkerSize', 10, ...
                'DisplayName', ['Centroid ', num2str(i)]); % Add centroids to legend
        end
    end

    legend(findobj(gca, '-regexp', 'DisplayName', 'Centroid.*'), 'Location', 'Best');
    title('Workspace Partitioning and Centroids');
    xlabel('Longitude');
    ylabel('Latitude');
    axis equal;
    
    hold off;
end

function visualizeCoverage(grid_points, robot_paths, boundary, obstacles, start_positions)
    figure;
    hold on;
    fill(boundary(:, 1), boundary(:, 2), [0.9, 0.9, 0.9], 'EdgeColor', 'k', 'LineWidth', 1.5);
    for k = 1:length(obstacles)
        fill(obstacles{k}(:, 1), obstacles{k}(:, 2), [0.7, 0.7, 0.7], 'EdgeColor', 'none');
    end

    scatter(grid_points(:, 1), grid_points(:, 2), 10, [0.8, 0.8, 0.8], 'filled');

    colors = lines(length(robot_paths));

    % plot robot paths
    for i = 1:length(robot_paths)
        if ~isempty(robot_paths{i})
            plot(robot_paths{i}(:, 1), robot_paths{i}(:, 2), '-', 'LineWidth', 2, 'Color', colors(i, :));
        end
    end

    scatter(start_positions(:,1), start_positions(:,2), 100, 'r', 'p', 'filled', 'DisplayName', 'Start Positions');

    title('Robot Paths and Coverage');
    xlabel('Longitude');
    ylabel('Latitude');
    axis equal;
    legend('Boundary', 'Obstacles', 'Grid Points', 'Robot Paths', 'Start Positions', 'Location', 'bestoutside');
    hold off;
end

function generate3DPlots(boundary, obstacles, cell_size)
    % visualize goal potential, potential field, shape function, and navigation function
    if nargin < 3
        cell_size = 0.0001; % Default cell size
    end

    goal = [mean(boundary(:, 1)), mean(boundary(:, 2))];
    % Visualize Goal Potential
    visualizeGoalPotential(boundary, obstacles, goal, cell_size);
    % Visualize Potential Field
    visualizePotentialField(boundary, obstacles, goal, cell_size);
    % Visualize Shape Function
    visualizeShapeFunction(boundary, obstacles, cell_size);
    % Visualize Navigation Function
    visualizeNavigationFunction(boundary, obstacles, goal, cell_size);
end

function visualizeGoalPotential(boundary, obstacles, goal, cell_size)
    [min_x, min_y, max_x, max_y] = boundingBox(boundary);
    [x, y] = meshgrid(min_x:cell_size:max_x, min_y:cell_size:max_y);
    attractive_potential = 0.5 * ((x - goal(1)).^2 + (y - goal(2)).^2);

    %  goal potential
    figure;
    surf(x, y, attractive_potential);
    shading interp;
    colorbar;
    hold on;
    plot3(boundary(:, 1), boundary(:, 2), zeros(size(boundary, 1), 1), 'k', 'LineWidth', 2);
    for k = 1:length(obstacles)
        plot3(obstacles{k}(:, 1), obstacles{k}(:, 2), zeros(size(obstacles{k}, 1), 1), 'k', 'LineWidth', 2);
    end
    plot3(goal(1), goal(2), 0, 'rx', 'MarkerSize', 12, 'LineWidth', 3);
    title('Goal Potential Function');
    xlabel('Longitude');
    ylabel('Latitude');
    zlabel('Potential Value');
    axis tight;
    grid on;
    hold off;
end

function visualizePotentialField(boundary, obstacles, goal, cell_size)
    [min_x, min_y, max_x, max_y] = boundingBox(boundary);
    [x, y] = meshgrid(min_x:cell_size:max_x, min_y:cell_size:max_y);
    points = [x(:), y(:)];
    attractive_potential = 0.5 * ((x - goal(1)).^2 + (y - goal(2)).^2);
    repulsive_potential = computeRepulsivePotential(boundary, obstacles, points);
    total_potential = attractive_potential + reshape(repulsive_potential, size(x));

    figure;
    surf(x, y, total_potential);
    shading interp;
    colorbar;
    hold on;
    plot3(boundary(:, 1), boundary(:, 2), zeros(size(boundary, 1), 1), 'k', 'LineWidth', 2);
    for k = 1:length(obstacles)
        plot3(obstacles{k}(:, 1), obstacles{k}(:, 2), zeros(size(obstacles{k}, 1), 1), 'k', 'LineWidth', 2);
    end
    plot3(goal(1), goal(2), 0, 'rx', 'MarkerSize', 12, 'LineWidth', 3);
    title('Potential Field (Attractive + Repulsive)');
    xlabel('Longitude');
    ylabel('Latitude');
    zlabel('Potential Value');
    axis tight;
    grid on;
    hold off;
end

function repulsive_potential = computeRepulsivePotential(boundary, obstacles, points)
    eta = 10; 
    d_threshold = 0.5;
    epsilon = 1e-3; 

    repulsive_potential = zeros(size(points, 1), 1);
    for i = 1:size(points, 1)
        point = points(i, :);
        % boundary repulsive potential
        boundary_distance = minDistanceToPolygon(point, boundary) + epsilon; % Add epsilon to avoid division by zero
        if boundary_distance < d_threshold
            repulsive_potential(i) = repulsive_potential(i) + eta * (1 / boundary_distance - 1 / d_threshold)^2;
        end
        % obstacles repulsive potential
        for k = 1:length(obstacles)
            obs_distance = minDistanceToPolygon(point, obstacles{k}) + epsilon; % Add epsilon to avoid division by zero
            if obs_distance < d_threshold
                repulsive_potential(i) = repulsive_potential(i) + eta * (1 / obs_distance - 1 / d_threshold)^2;
            end
        end
    end

    % smoothing to reduce spikes
    repulsive_potential = imgaussfilt(repulsive_potential, 1); % gaussian filter for smoothness
end

function distance = minDistanceToPolygon(point, polygon)
    num_vertices = size(polygon, 1);
    distances = arrayfun(@(j) pointToSegmentDistance(point, polygon(j, :), polygon(mod(j, num_vertices) + 1, :)), 1:num_vertices);
    distance = min(distances);
end

function distance = pointToSegmentDistance(point, segment_start, segment_end)
    v = segment_end - segment_start;
    w = point - segment_start;
    c1 = dot(w, v);
    if c1 <= 0
        distance = norm(point - segment_start);
        return;
    end
    c2 = dot(v, v);
    if c2 <= c1
        distance = norm(point - segment_end);
        return;
    end
    b = c1 / c2;
    pb = segment_start + b * v;
    distance = norm(point - pb);
end

function visualizeShapeFunction(boundary, obstacles, cell_size)
    % shape function for the workspace
    [min_x, min_y, max_x, max_y] = boundingBox(boundary);
    [x, y] = meshgrid(min_x:cell_size:max_x, min_y:cell_size:max_y);
    points = [x(:), y(:)];
    shape_function = zeros(size(points, 1), 1);
    poly_boundary = polyshape(boundary(:,1), boundary(:,2));
    inside = isinterior(poly_boundary, points(:,1), points(:,2));

    for k = 1:length(obstacles)
        poly_obstacle = polyshape(obstacles{k}(:,1), obstacles{k}(:,2));
        inside = inside & ~isinterior(poly_obstacle, points(:,1), points(:,2));
    end

    shape_function(inside) = 1;
    shape_function = reshape(shape_function, size(x));

    figure;
    surf(x, y, shape_function);
    shading interp;
    colorbar;
    hold on;
    plot3(boundary(:, 1), boundary(:, 2), zeros(size(boundary, 1), 1), 'k', 'LineWidth', 2);
    for k = 1:length(obstacles)
        plot3(obstacles{k}(:, 1), obstacles{k}(:, 2), zeros(size(obstacles{k}, 1), 1), 'k', 'LineWidth', 2);
    end
    title('Shape Function of the Workspace');
    xlabel('Longitude');
    ylabel('Latitude');
    zlabel('Shape Function Value');
    axis tight;
    grid on;
    hold off;
end

function visualizeNavigationFunction(boundary, obstacles, goal, cell_size)
    [min_x, min_y, max_x, max_y] = boundingBox(boundary);
    [x, y] = meshgrid(min_x:cell_size:max_x, min_y:cell_size:max_y);
    points = [x(:), y(:)];
    attractive_potential = 0.5 * ((x - goal(1)).^2 + (y - goal(2)).^2);
    repulsive_potential = computeRepulsivePotential(boundary, obstacles, points);
    shape_function = zeros(size(points, 1), 1);
    poly_boundary = polyshape(boundary(:,1), boundary(:,2));
    inside = isinterior(poly_boundary, points(:,1), points(:,2));

    for k = 1:length(obstacles)
        poly_obstacle = polyshape(obstacles{k}(:,1), obstacles{k}(:,2));
        inside = inside & ~isinterior(poly_obstacle, points(:,1), points(:,2));
    end

    navigation_function = (attractive_potential + reshape(repulsive_potential, size(x))) .* reshape(inside, size(x));

    figure;
    surf(x, y, navigation_function);
    shading interp;
    colorbar;
    hold on;
    plot3(boundary(:, 1), boundary(:, 2), zeros(size(boundary, 1), 1), 'k', 'LineWidth', 2);
    for k = 1:length(obstacles)
        plot3(obstacles{k}(:, 1), obstacles{k}(:, 2), zeros(size(obstacles{k}, 1), 1), 'k', 'LineWidth', 2);
    end
    plot3(goal(1), goal(2), 0, 'rx', 'MarkerSize', 12, 'LineWidth', 3);
    title('Navigation Function');
    xlabel('Longitude');
    ylabel('Latitude');
    zlabel('Navigation Function Value');
    axis tight;
    grid on;
    hold off;
end


