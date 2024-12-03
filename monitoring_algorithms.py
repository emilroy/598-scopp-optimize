import numpy as np
import matplotlib.pyplot as plt
from pyDOE import lhs
from shapely.geometry import Point, Polygon
from sklearn.cluster import MiniBatchKMeans
import sklearn.neighbors as skn
from math import sqrt, tan, radians
import latlongcartconv as lc
import copy
import time
import pickle
import random
import SCoPP_settings
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class QLB:
    def __init__(self, environment, number_of_robots=None, plot="full", plot_settings=SCoPP_settings.plots(), algorithm_settings=SCoPP_settings.algorithm()):
        # Initialize environment parameters
        self.number_of_robots = number_of_robots or len(environment.starting_position)
        if len(environment.starting_position) > 1:
            geographical_starting_positions = environment.starting_position
        else:
            geographical_starting_positions = []
            for agent in range(number_of_robots):
                geographical_starting_positions.extend(environment.starting_position)
        self.geographical_boundary_points = environment.boundary_points
        self.robot_FOV = environment.robot_FOV
        self.robot_operating_height = environment.robot_operating_height
        self.robot_velocity = environment.robot_velocity
        self.geographical_geo_fencing_zones = environment.geo_fencing_holes
        self.save_path = environment.save_path

        # Initialize reference frame
        self.robot_initial_positions_in_cartesian = []
        self.boundary_points_in_cartesian = []
        self.boundary_points_in_geographical = []
        self.robot_initial_positions_in_geographical = []
        origin = [0, 0]
        origin[1] = min(np.concatenate(
            (np.transpose(geographical_starting_positions)[0], np.transpose(self.geographical_boundary_points)[0])))
        origin[0] = min(np.concatenate(
            (np.transpose(geographical_starting_positions)[1], np.transpose(self.geographical_boundary_points)[1])))
        self.origin = origin
        
        if self.geographical_geo_fencing_zones:
            self.geographical_geo_fencing_zones_in_cartesian = [[] for item in
                                                                range(len(self.geographical_geo_fencing_zones))]
            self.geographical_geo_fencing_zones_in_geographical = [[] for item in
                                                                   range(len(self.geographical_geo_fencing_zones))]
        else:
            self.geographical_geo_fencing_zones_in_cartesian = None
            self.geographical_geo_fencing_zones_in_geographical = None
        for robot, position in enumerate(geographical_starting_positions):
            self.robot_initial_positions_in_geographical.append([position[1], position[0]])
        for point in self.geographical_boundary_points:
            self.boundary_points_in_geographical.append([point[1], point[0]])
        if self.geographical_geo_fencing_zones:
            for list_id, fence_list in enumerate(self.geographical_geo_fencing_zones):
                for point in fence_list:
                    self.geographical_geo_fencing_zones_in_geographical[list_id].append([point[1], point[0]])
                    
        if number_of_robots is None:
            self.number_of_partitions = len(environment.starting_position)
        else:
            self.number_of_partitions = number_of_robots

        # Initialize method variables
        self.initialize_method_variables()

        # Initialize algorithm settings
        self.initialize_algorithm_settings(algorithm_settings)

        # Initialize computation time attributes
        self.initialize_computation_time_attributes()

        # Initialize run data attributes for statistics
        self.initialize_run_data_attributes()

        # Initialize variables for plotting
        self.initialize_plotting_variables()

        # Plot settings
        self.initialize_plot_settings(plot, plot_settings)

        # Initialize other useful information
        self.initialize_other_information()

    def initialize_method_variables(self):
        self.cell_wall_interval = 3
        self.dominated_cells_x = [[] for _ in range(self.number_of_robots)]
        self.dominated_cells_y = [[] for _ in range(self.number_of_robots)]
        self.robot_WOV = tan(radians(self.robot_FOV / 2)) * self.robot_operating_height * 2
        self.cell_size = int(self.robot_WOV)
        self.cell_size -= self.cell_size % 2
        self.cell_area = self.cell_size ** 2
        self.robot_assignment_information = None

    def initialize_algorithm_settings(self, algorithm_settings):
        self.bias_factor = algorithm_settings.bias_factor
        self.sampling_rate = algorithm_settings.sampling_rate
        self.leaf_size = algorithm_settings.leaf_size
        self.conflict_resolution_mode = algorithm_settings.conflict_resolution_mode
        self.planner = algorithm_settings.planner
        self.partition_tolerance = algorithm_settings.partition_tolerance or self.cell_size / 8
        self.partition_max_iter = algorithm_settings.partition_max_iter

    def initialize_computation_time_attributes(self):
        self.time_for_conversion = 0
        self.time_for_partitioning = 0
        self.time_for_discretization = 0
        self.time_for_conflict_resolution = 0
        self.time_for_path_planning = 0
        self.total_time = 0

    def initialize_run_data_attributes(self):
        self.data_tasks_per_robot = None
        self.data_distance_travelled_per_robot = None
        self.data_completion_time_per_robot = None
        self.data_total_mission_completion_time = None
        self.data_computation_time_without_path_planning = None
        self.data_computation_time_with_path_planning = None

    def initialize_plotting_variables(self):
        self.rough_partitioning_x = [[] for _ in range(self.number_of_robots)]
        self.rough_partitioning_y = [[] for _ in range(self.number_of_robots)]
        self.cluster_centers = None
        self.conflict_cells_x = []
        self.conflict_cells_y = []
        self.final_partitioning_x = [[] for _ in range(self.number_of_robots)]
        self.final_partitioning_y = [[] for _ in range(self.number_of_robots)]

    def initialize_plot_settings(self, plot, plot_settings):
        self.plot = plot
        self.plot_cell_boundary_size = plot_settings.cell_boundary
        self.plot_robot_path_size = plot_settings.robot_path_size
        self.plot_robot_size = plot_settings.robot_size
        self.plot_cell_size = plot_settings.cell_size
        self.partition_colors = np.round(lhs(3, samples=self.number_of_robots), decimals=1)

    def initialize_other_information(self):
        self.total_survey_area = None
        self.area_per_robot = None
        self.area_covered_over_time = []
        self.area_covered_over_time_time_vector = []

    def run(self, info=False):
        t0 = time.time()

        # Convert from geographical to cartesian coordinates
        time_stamp = time.time()
        coordinate_converter = lc.LLCCV(self.origin)
        for item in self.robot_initial_positions_in_geographical:
            self.robot_initial_positions_in_cartesian.append(coordinate_converter.get_cartesian(item))
        for item in self.boundary_points_in_geographical:
            self.boundary_points_in_cartesian.append(coordinate_converter.get_cartesian(item))
        if self.geographical_geo_fencing_zones_in_geographical:
            for list_id, fence_list in enumerate(self.geographical_geo_fencing_zones_in_geographical):
                for item in fence_list:
                    self.geographical_geo_fencing_zones_in_cartesian[list_id].append(
                        coordinate_converter.get_cartesian(item))
        self.time_for_conversion += time.time() - time_stamp

        # Commence discretization of the area
        time_stamp = time.time()
        cell_space, cells = self.discretize_massive_area()
        self.time_for_discretization += time.time() - time_stamp
        
        # Commence partitioning of the discretized area
        time_stamp = time.time()
        cells_as_dict, cell_space_as_dict, robot_assignment_information = self.partition_area(cell_space, cells)
        self.time_for_partitioning += time.time() - time_stamp
        
        # Commence conflict resolution
        time_stamp = time.time()
        robot_assignment_information, waypoints_for_robots = \
            self.resolve_conflicts(cells_as_dict, cell_space_as_dict, robot_assignment_information,
                                   self.conflict_resolution_mode)
        self.time_for_conflict_resolution += time.time() - time_stamp
        
        # Commence path planning
        timestamp = time.time()
        paths, distances = self.find_optimal_paths(self.planner, robot_assignment_information, waypoints_for_robots)
        self.time_for_path_planning = time.time() - timestamp

        self.calculate_surveillance_rate(distances)
        final_paths_in_geographical = []
        for path in paths:
            entry = []
            for item in path:
                entry.append(coordinate_converter.get_geographic(item))
            final_paths_in_geographical.append(entry)
        self.time_for_conversion += time.time() - time_stamp

        self.total_time = time.time() - t0
        self.area_per_robot = np.array(self.data_tasks_per_robot) * self.cell_area

        # Display algorithm attributes
        if info:
            self.display_info(info)
        # Plot results
        if self.plot:
            self.plot_partitioning()
            
        self.plot_potential_field_3d()

        return final_paths_in_geographical

    def plot_potential_field_3d(self):
        # Create a finer meshgrid for smoother visualization
        x = np.linspace(min(self.boundary_points_in_cartesian, key=lambda p: p[0])[0],
                        max(self.boundary_points_in_cartesian, key=lambda p: p[0])[0], 100)
        y = np.linspace(min(self.boundary_points_in_cartesian, key=lambda p: p[1])[1],
                        max(self.boundary_points_in_cartesian, key=lambda p: p[1])[1], 100)
        X, Y = np.meshgrid(x, y)
        Z = np.zeros_like(X)

        # Calculate potential field with multiple peaks
        for i in range(X.shape[0]):
            for j in range(X.shape[1]):
                point = [X[i, j], Y[i, j]]
                # Sum up potentials from all robots and obstacles
                Z[i, j] = self.calculate_multi_peak_potential(point)

        # Create the 3D plot with enhanced visualization
        fig = plt.figure(figsize=(12, 9))
        ax = fig.add_subplot(111, projection='3d')
        
        # Plot surface with enhanced appearance
        surf = ax.plot_surface(X, Y, Z, cmap='rainbow', 
                            edgecolor='none',
                            alpha=0.8,
                            antialiased=True,
                            rstride=1,
                            cstride=1)

        # Enhance the appearance
        ax.view_init(elev=30, azim=45)  # Adjust viewing angle
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Potential')
        
        # Add color bar
        fig.colorbar(surf, shrink=0.5, aspect=5)
        plt.show()

    def calculate_multi_peak_potential(self, point):
        potential = 0
        
        # Add attractive potentials (valleys) at robot goals
        k_att = 0.5  # Attractive constant
        for i, goal in enumerate(self.cluster_centers):
            dist = np.sqrt((point[0] - goal[0])**2 + (point[1] - goal[1])**2)
            potential -= k_att * np.exp(-dist/50)  # Gaussian attractive potential
        
        # Add repulsive potentials (peaks) from obstacles
        k_rep = 100  # Repulsive constant
        obstacles = self.get_obstacles()
        for obstacle in obstacles:
            for i in range(len(obstacle)):
                p1 = obstacle[i]
                p2 = obstacle[(i+1) % len(obstacle)]
                closest = self.closest_point_on_segment(point, p1, p2)
                dist = np.sqrt((point[0] - closest[0])**2 + (point[1] - closest[1])**2)
                potential += k_rep * np.exp(-dist/20)  # Gaussian repulsive potential
        
        return potential

    def discretize_massive_area(self):
        x_max = max(max(point[1] for point in self.robot_initial_positions_in_cartesian),
                    max(point[1] for point in self.boundary_points_in_cartesian))
        y_max = max(max(point[0] for point in self.robot_initial_positions_in_cartesian),
                    max(point[0] for point in self.boundary_points_in_cartesian))

        if self.geographical_geo_fencing_zones_in_cartesian:
            boundary_polygon = Polygon(self.boundary_points_in_cartesian, holes=self.geographical_geo_fencing_zones_in_cartesian)
        else:
            boundary_polygon = Polygon(self.boundary_points_in_cartesian)

        self.total_survey_area = boundary_polygon.area

        cells = np.zeros((int(x_max / self.cell_size), int(y_max / self.cell_size))) + np.NINF
        cell_squares_within_boundary_x = []
        cell_squares_within_boundary_y = []

        for row in range(len(cells)):
            for column in range(len(cells[0])):
                cell_boundary = self.get_cell_boundary(row, column, boundary_polygon)
                if cell_boundary:
                    cell_squares_within_boundary_x.extend([point[0] for point in cell_boundary])
                    cell_squares_within_boundary_y.extend([point[1] for point in cell_boundary])

        return [cell_squares_within_boundary_x, cell_squares_within_boundary_y], cells

    def get_cell_boundary(self, row, column, boundary_polygon):
        cell_boundary = []
        for i in range(0, self.cell_size + 1, self.cell_size):
            for j in range(0, self.cell_size + 1, self.cell_size):
                point = (i + column * self.cell_size, j + row * self.cell_size)
                if Point(point).within(boundary_polygon):
                    cell_boundary.append(point)
        return cell_boundary if len(cell_boundary) == 4 else None

    def partition_area(self, cell_space, cells):
        """
        Partitioning phase: divides surveyable area are into equally sized areas; one for each robot.
        """
        cell_squares_within_boundary_x = cell_space[0]
        cell_squares_within_boundary_y = cell_space[1]
        cells_within_boundary_x = []
        cells_within_boundary_y = []
        for row, _ in enumerate(cells):
            for column, _ in enumerate(cells[0]):
                cells_within_boundary_x.append(row)
                cells_within_boundary_y.append(column)
        clustering_set = np.transpose([cell_squares_within_boundary_x, cell_squares_within_boundary_y])
        kmeans = MiniBatchKMeans(n_clusters=self.number_of_partitions, max_iter=self.partition_max_iter,
                                 tol=self.partition_tolerance)
        cluster_indices = kmeans.fit_predict(clustering_set)
        self.cluster_centers = [kmeans.cluster_centers_[:, 0], kmeans.cluster_centers_[:, 1]]
        self.partition_colors = lhs(3, samples=self.number_of_partitions)
        self.partition_colors = np.round(self.partition_colors, decimals=1)
        robot_assignment_information = [[] for robot in range(self.number_of_partitions)]
        robot_initial_positions_copy = self.robot_initial_positions_in_cartesian.copy()
        self.rough_partitioning_x = [[] for robot in range(self.number_of_partitions)]
        self.rough_partitioning_y = [[] for robot in range(self.number_of_partitions)]
        minimum_distance = [[np.Inf] for robot in range(self.number_of_partitions)]
        cell_space_as_dict = dict()
        cells_as_dict = dict()
        for robot_id in range(self.number_of_partitions):
            for point_index, point in enumerate(clustering_set):
                if cluster_indices[point_index] != robot_id:
                    continue
                else:
                    row = point[0]
                    column = point[1]
                    self.rough_partitioning_x[robot_id].append(row)
                    self.rough_partitioning_y[robot_id].append(column)
                    cell_space_as_dict[column, row] = robot_id
                    if row % self.cell_size == 0 and column % self.cell_size == 0:
                        cells_as_dict[column, row] = robot_id
                        for pos in robot_initial_positions_copy:
                            if np.linalg.norm(
                                    [row - pos[0] + 0.0001,
                                     column - pos[1] + 0.0001]) < minimum_distance[robot_id]:
                                minimum_distance[robot_id] = np.linalg.norm(
                                    [row - pos[0] + 0.0001, column - pos[1] + 0.0001])
                                assignment_information = [pos[0], pos[1], row + self.cell_size / 2,
                                                          column + self.cell_size / 2,
                                                          round(minimum_distance[robot_id] / self.cell_size)]
            robot_assignment_information[robot_id] = assignment_information
            robot_initial_positions_copy.remove([assignment_information[0], assignment_information[1]])
        return cells_as_dict, cell_space_as_dict, robot_assignment_information

    def resolve_conflicts(self, cells_as_dict, cell_space_as_dict, robot_assignment_information, mode):
        self.conflict_cells_x = []
        self.conflict_cells_y = []
        self.data_tasks_per_robot = [0 for _ in range(self.number_of_robots)]
        non_dominated_cells = []

        for corner in cells_as_dict:
            cell_boundary_values = self.get_cell_boundary_values(corner, cell_space_as_dict)
            if len(cell_boundary_values) >= self.cell_size * 4 and not all(elem == cell_boundary_values[0] for elem in cell_boundary_values):
                self.add_conflict_cell(corner)
                occupied_partitions = list(set(cell_boundary_values))
                non_dominated_cells.append([[corner[0], corner[1]], occupied_partitions])
            else:
                self.add_dominated_cell(cell_boundary_values[0], corner)

        non_dominated_cells = []
        for corner in cells_as_dict:
            cell_boundary_values = []
            cell_boundary_points_y = []
            cell_boundary_points_x = []
            for i in range(self.cell_size + 1):
                for j in range(self.cell_size + 1):
                    try:
                        cell_boundary_values.append(cell_space_as_dict[i + corner[0], j + corner[1]])
                        cell_boundary_points_x.append(i + corner[1])
                        cell_boundary_points_y.append(j + corner[0])
                    except KeyError:
                        continue
            if len(cell_boundary_values) >= self.cell_size / self.cell_wall_interval * 4:
                if not all(elem == cell_boundary_values[0] for elem in cell_boundary_values):
                    self.conflict_cells_y.extend(cell_boundary_points_y)
                    self.conflict_cells_x.extend(cell_boundary_points_x)
                    occupied_partitions = []
                    for partition_number in cell_boundary_values:
                        if partition_number not in occupied_partitions:
                            occupied_partitions.append(partition_number)
                    non_dominated_cells.append([[corner[0], corner[1]], occupied_partitions])
                else:
                    self.data_tasks_per_robot[int(cell_boundary_values[0])] += 1
                    self.dominated_cells_x[int(cell_boundary_values[0])].append(int(corner[1] + self.cell_size / 2))
                    self.dominated_cells_y[int(cell_boundary_values[0])].append(int(corner[0] + self.cell_size / 2))

        # Get initial partitioning distribution information
        # self.data_tasks_per_robot, non_dominated_cells = self.sum_cells_per_robot(cell_space_as_dict)
        if mode == "bias":
            # Add bias to robots based on robot states
            distance_bias = np.transpose(robot_assignment_information.copy())[4] * self.bias_factor

            # Resolve conflicts (nondominated cells)
            self.final_partitioning_x = copy.deepcopy(self.dominated_cells_x)
            self.final_partitioning_y = copy.deepcopy(self.dominated_cells_y)
            for cell in non_dominated_cells:
                column = int(cell[0][0])
                row = int(cell[0][1])
                lower_cell_count_robot = cell[1][0]
                for region in cell[1]:
                    if self.data_tasks_per_robot[int(region)] + distance_bias[int(region)] < \
                            self.data_tasks_per_robot[int(lower_cell_count_robot)] +\
                            distance_bias[int(lower_cell_count_robot)]:
                        lower_cell_count_robot = region
                cell_space_as_dict[cell_space_as_dict[cell[0][0], cell[0][1]]] = lower_cell_count_robot
                self.final_partitioning_y[int(lower_cell_count_robot)].append(int(column + self.cell_size / 2))
                self.final_partitioning_x[int(lower_cell_count_robot)].append(int(row + self.cell_size / 2))
                self.data_tasks_per_robot[int(lower_cell_count_robot)] += 1
        else:
            # Resolve conflicts using random walk (nondominated cells)
            self.final_partitioning_x = copy.deepcopy(self.dominated_cells_x)
            self.final_partitioning_y = copy.deepcopy(self.dominated_cells_y)
            for cell in non_dominated_cells:
                row = int(cell[0][0] - self.cell_size / 2)
                column = int(cell[0][1] - self.cell_size / 2)
                robot = np.random.randint(0, self.number_of_partitions, )
                self.final_partitioning_y[robot].append(int(column + self.cell_size / 2))
                self.final_partitioning_x[robot].append(int(row + self.cell_size / 2))
                self.data_tasks_per_robot[robot] += 1

        final_partitions = [[] for _ in range(self.number_of_robots)]
        for robot_id in range(self.number_of_robots):
            final_partitions[robot_id] = [self.final_partitioning_x[robot_id], self.final_partitioning_y[robot_id]]

        return robot_assignment_information, final_partitions

    def get_cell_boundary_values(self, corner, cell_space_as_dict):
        cell_boundary_values = []
        for i in range(self.cell_size + 1):
            for j in range(self.cell_size + 1):
                try:
                    cell_boundary_values.append(cell_space_as_dict[i + corner[0], j + corner[1]])
                except KeyError:
                    continue
        return cell_boundary_values

    def add_conflict_cell(self, corner):
        self.conflict_cells_y.extend([j + corner[0] for j in range(self.cell_size + 1)])
        self.conflict_cells_x.extend([i + corner[1] for i in range(self.cell_size + 1)])

    def add_dominated_cell(self, robot_id, corner):
        self.data_tasks_per_robot[int(robot_id)] += 1
        self.dominated_cells_x[int(robot_id)].append(int(corner[1] + self.cell_size / 2))
        self.dominated_cells_y[int(robot_id)].append(int(corner[0] + self.cell_size / 2))
    
    def apply_potential_field(self, robot_positions, obstacles):
        adjusted_positions = []
        # print(robot_positions)
        for robot_pos in robot_positions:
            # Attractive force towards goal
            goal = self.cluster_centers[robot_pos[2]]
            f_att = self.attractive_force(robot_pos[:2], goal)
            
            # Repulsive force from obstacles
            f_rep = [0, 0]
            for obstacle in obstacles:
                for i in range(len(obstacle)):
                    p1 = obstacle[i]
                    p2 = obstacle[(i+1) % len(obstacle)]
                    f_rep_segment = self.repulsive_force_segment(robot_pos[:2], p1, p2)
                    f_rep = [a + b for a, b in zip(f_rep, f_rep_segment)]
            
            # Combine forces
            total_force = [a + b for a, b in zip(f_att, f_rep)]
            
            # Adjust position
            new_pos = [robot_pos[0] + total_force[0], robot_pos[1] + total_force[1], robot_pos[2]]
            adjusted_positions.append(new_pos)
        
        return adjusted_positions

    def repulsive_force_segment(self, robot_pos, p1, p2):
        self.k_rep = 100  # Repulsive force constant
        self.d0 = 5  # Influence distance of obstacle
        
        # Calculate the closest point on the segment to the robot
        closest_point = self.closest_point_on_segment(robot_pos, p1, p2)
        
        dx = robot_pos[0] - closest_point[0]
        dy = robot_pos[1] - closest_point[1]
        dist = sqrt(dx**2 + dy**2)
        
        if dist <= self.d0:
            return [self.k_rep * (1/dist - 1/self.d0) * (1/dist**2) * (dx/dist),
                    self.k_rep * (1/dist - 1/self.d0) * (1/dist**2) * (dy/dist)]
        else:
            return [0, 0]

    def closest_point_on_segment(self, p, a, b):
        ax, ay = a
        bx, by = b
        px, py = p

        abx = bx - ax
        aby = by - ay
        t = ((px - ax) * abx + (py - ay) * aby) / (abx**2 + aby**2)
        t = max(0, min(1, t))

        return [ax + t * abx, ay + t * aby]

    def attractive_force(self, robot_pos, goal_pos):
        """Calculate attractive force towards goal"""
        self.k_att = 0.5  # Attractive force constant
        dx = goal_pos[0] - robot_pos[0]
        dy = goal_pos[1] - robot_pos[1]
        dist = sqrt(dx**2 + dy**2)
        return [self.k_att * dx / dist, self.k_att * dy / dist]

    def repulsive_force(self, robot_pos, obstacle_pos):
        """Calculate repulsive force from obstacle"""
        k_rep = 100  # Repulsive force constant
        d0 = 5  # Influence distance of obstacle
        dx = robot_pos[0] - obstacle_pos[0]
        dy = robot_pos[1] - obstacle_pos[1]
        dist = sqrt(dx**2 + dy**2)
        if dist <= d0:
            return [k_rep * (1/dist - 1/d0) * (1/dist**2) * (dx/dist),
                    k_rep * (1/dist - 1/d0) * (1/dist**2) * (dy/dist)]
        else:
            return [0, 0]
    
    def get_obstacles(self):
        """
        Get obstacle positions from the environment.
        This method needs to be implemented based on how obstacles are represented in your environment.
        """
        obstacles = []
        if self.geographical_geo_fencing_zones_in_cartesian:
            for zone in self.geographical_geo_fencing_zones_in_cartesian:
                obstacles.append(zone)
        return obstacles

    def find_optimal_paths(self, planner, robot_assignment_information, waypoints_for_robots):
        self.optimal_paths = [[] for _ in range(self.number_of_partitions)]
        waypoint_distances = [[] for _ in range(self.number_of_partitions)]

        if planner == "nn":
            for robot, cell_list in enumerate(waypoints_for_robots):
                current_position = robot_assignment_information[robot][0:2]
                task_list = np.transpose(cell_list).tolist()
                task_list.append(current_position)
                while True:
                    if len(task_list) == 1:
                        waypoint_distances[robot].append(
                            sqrt((self.optimal_paths[robot][-1][0] - current_position[0]) ** 2 +
                                 (self.optimal_paths[robot][-1][1] - current_position[1]) ** 2))
                        self.optimal_paths[robot].append(current_position)
                        task_list.remove(current_position)
                        break
                    self.optimal_paths[robot].append(current_position)
                    nbrs = skn.NearestNeighbors(n_neighbors=2, algorithm='kd_tree', leaf_size=self.leaf_size).fit(
                        task_list)
                    distances, indices = nbrs.kneighbors(task_list)
                    waypoint_distances[robot].append(distances[task_list.index(current_position)][1])
                    next_position = task_list[indices[task_list.index(current_position)][1]]
                    task_list.remove(current_position)
                    current_position = next_position
                # After calculating initial paths, apply potential field adjustments
                obstacles = self.get_obstacles()
                for i in range(2):
                    adjusted_path = []
                    for point in self.optimal_paths[i]:
                        robot_pos = [point[0], point[1], i]  # Add robot index to position
                        adjusted_pos = self.apply_potential_field([robot_pos], obstacles)[0]
                        adjusted_path.append(adjusted_pos[:2])  # Remove robot index
                    self.optimal_paths[i] = adjusted_path

        elif planner == "random_walk":
            for robot, cell_list in enumerate(waypoints_for_robots):
                current_position = robot_assignment_information[robot][0:2]
                task_list = np.transpose(cell_list).tolist()
                self.optimal_paths[robot].append(current_position)
                radius = 1
                
                while task_list:
                    for task in random.sample(task_list, len(task_list)):
                        distance = sqrt((current_position[0] - task[0]) ** 2 +
                                        (current_position[1] - task[1]) ** 2)
                        if distance <= radius:
                            waypoint_distances[robot].append(distance)
                            task_list.remove(task)
                            current_position = task
                            self.optimal_paths[robot].append(task)
                            radius = 0
                            break
                    radius += 1

        elif planner == "tsp:brute":
            for robot, path in enumerate(waypoints_for_robots):
                shortest_distance = float('inf')
                start = robot_assignment_information[robot][0:2]
                path = [start] + np.transpose(path).tolist()
                
                for path_permutation in itertools.permutations(path[1:]):
                    full_path = [start] + list(path_permutation)
                    distance = 0
                    distances = []
                    for i in range(len(full_path) - 1):
                        dist = sqrt((full_path[i][0] - full_path[i+1][0]) ** 2 +
                                    (full_path[i][1] - full_path[i+1][1]) ** 2)
                        distance += dist
                        distances.append(dist)
                    
                    if distance < shortest_distance:
                        shortest_distance = distance
                        self.optimal_paths[robot] = full_path
                        waypoint_distances[robot] = distances

        elif planner == "tsp:ga":
            for robot, partition in enumerate(waypoints_for_robots):
                start = robot_assignment_information[robot][0:2]
                coords_list = np.vstack((start, np.transpose(partition)))
                
                dist_matrix = np.zeros((len(coords_list), len(coords_list)))
                for i in range(len(coords_list)):
                    for j in range(i+1, len(coords_list)):
                        dist = sqrt((coords_list[i][0] - coords_list[j][0])**2 +
                                    (coords_list[i][1] - coords_list[j][1])**2)
                        dist_matrix[i][j] = dist_matrix[j][i] = dist
                
                # Use a genetic algorithm library to solve TSP
                # This is a placeholder and should be replaced with actual GA implementation
                best_tour = self.genetic_algorithm_tsp(dist_matrix)
                
                self.optimal_paths[robot] = [tuple(coords_list[i]) for i in best_tour]
                waypoint_distances[robot] = [dist_matrix[best_tour[i]][best_tour[i+1]] 
                                            for i in range(len(best_tour)-1)]

        # Save optimal path information to specified save path
        with open(self.save_path + "waypoints", 'wb') as f:
            pickle.dump(self.optimal_paths, f)

        return self.optimal_paths, waypoint_distances
    
    def display_info(self, info):
        if info == "verbose":
                # Display computation time attributes
                print("Computation times for various operations (in seconds):")
                print("     Conversions:", self.time_for_conversion)
                print("     Initial partitioning:", self.time_for_partitioning)
                print("     Discretization:", self.time_for_discretization)
                print("     Conflict resolution:", self.time_for_conflict_resolution)
                print("     Path planning:", self.time_for_path_planning)
                print("     Entire algorithm:", self.total_time)
                # Display data attributes for statistics
                print("Statistics:")
                print("     Tasks per robot:", self.data_tasks_per_robot)
                print("     Distance for each robot to travel:", self.data_distance_travelled_per_robot, "seconds")
                print("     Mission completion time per robot:", self.data_completion_time_per_robot, "seconds")
                print("     Mission completion time of the swarm:", self.data_total_mission_completion_time, "seconds")
                # Display other environment information
                print("Area allocation information:")
                print("     Cell width:", self.cell_size, "meters")
                print("     Cell area:", self.cell_area, "square meters")
                print("     Total area to be surveyed:", self.total_survey_area, "square meters")
                print("     Area to be surveyed per robot:", self.area_per_robot, "square meters")

        else:
            if info == "time":
                # Display computation time attributes
                print("Computation times for various operations (in seconds):")
                print("     Conversions:", self.time_for_conversion)
                print("     Initial partitioning:", self.time_for_partitioning)
                print("     Discretization:", self.time_for_discretization)
                print("     Conflict resolution:", self.time_for_conflict_resolution)
                print("     Path planning:", self.time_for_path_planning)
                print("     Entire algorithm:", self.total_time)

            if info == "mission":
                # Display data attributes for statistics
                print("Statistics:")
                print("     Tasks per robot:", self.data_tasks_per_robot)
                print("     Distance for each robot to travel:", self.data_distance_travelled_per_robot, "seconds")
                print("     Mission completion time per robot:", self.data_completion_time_per_robot, "seconds")
                print("     Mission completion time of the swarm:", self.data_total_mission_completion_time,
                        "seconds")

            if info == "area":
                # Display other environment information
                print("Area allocation information:")
                print("     Cell width:", self.cell_size, "meters")
                print("     Cell area:", self.cell_area, "square meters")
                print("     Total area to be surveyed:", self.total_survey_area, "square meters")
                print("     Area to be surveyed per robot:", self.area_per_robot, "square meters")
    
    def plot_partitioning(self):
        """
        Plot algorithm run results
        """
        if "full" in self.plot:
            import pickle
            with open(self.save_path + "/rough_partitioning.txt", "wb") as fp:  # Pickling
                pickle.dump([self.rough_partitioning_x, self.rough_partitioning_y], fp)
            with open(self.save_path + "/final_partitioning.txt", "wb") as fp:  # Pickling
                pickle.dump([self.final_partitioning_x, self.final_partitioning_y], fp)
            with open(self.save_path + "/dominated_cells.txt", "wb") as fp:  # Pickling
                pickle.dump([self.dominated_cells_x, self.dominated_cells_y], fp)
            with open(self.save_path + "/conflict_cells.txt", "wb") as fp:  # Pickling
                pickle.dump([self.conflict_cells_x, self.conflict_cells_y], fp)
            with open(self.save_path + "/partition_colors.txt", "wb") as fp:  # Pickling
                pickle.dump(self.partition_colors, fp)

            with open(self.save_path + "/robot_initial_positions_in_cartesian.txt", "wb") as fp:  # Pickling
                pickle.dump(self.robot_initial_positions_in_cartesian, fp)

            with open(self.save_path + "/cluster_centers.txt", "wb") as fp:  # Pickling
                pickle.dump(self.cluster_centers, fp)

            with open(self.save_path + "/number_of_partitions.txt", "wb") as fp:  # Pickling
                pickle.dump(self.number_of_partitions, fp)

            plt.subplot(2, 2, 1)
            for robot_id in range(self.number_of_partitions):
                plt.scatter(self.rough_partitioning_x[robot_id], self.rough_partitioning_y[robot_id], marker="s",
                            s=self.plot_cell_boundary_size,
                            c=np.ones((len(self.rough_partitioning_x[robot_id]), 3)) * self.partition_colors[robot_id])
                plt.scatter(self.cluster_centers[0], self.cluster_centers[1], s=self.plot_robot_size, c='black')
            plt.axis("equal")

            plt.subplot(2, 2, 2)
            print("Number of robots: ", self.number_of_partitions)
            for robot_id in range(self.number_of_partitions):
                plt.scatter(self.rough_partitioning_x[robot_id], self.rough_partitioning_y[robot_id], marker="s",
                            s=self.plot_cell_boundary_size,
                            c=np.ones((len(self.rough_partitioning_x[robot_id]), 3)) * self.partition_colors[robot_id])
                plt.scatter(self.dominated_cells_x[robot_id], self.dominated_cells_y[robot_id], marker="s",
                            s=self.plot_cell_size,
                            c=np.ones((len(self.dominated_cells_x[robot_id]), 3)) * self.partition_colors[robot_id])
            plt.scatter(self.conflict_cells_x, self.conflict_cells_y, marker="s", s=self.plot_cell_boundary_size * 3,
                        c="black")
            plt.axis("equal")

            plt.subplot(2, 2, 3)
            for robot_id in range(self.number_of_partitions):
                plt.scatter(self.final_partitioning_x[robot_id], self.final_partitioning_y[robot_id], marker="s",
                            s=self.plot_cell_size,
                            c=np.ones((len(self.final_partitioning_x[robot_id]), 3)) * self.partition_colors[robot_id])
            plt.scatter(self.conflict_cells_x, self.conflict_cells_y, marker="s", s=self.plot_cell_boundary_size * 3,
                        c="black")
            plt.axis("equal")

            ax4 = plt.subplot(2, 2, 4)
            ax4.scatter(np.transpose(self.robot_initial_positions_in_cartesian)[0],
                        np.transpose(self.robot_initial_positions_in_cartesian)[1],
                        s=self.plot_robot_size, c="black")
            for robot_id in range(self.number_of_partitions):
                ax4.scatter(self.final_partitioning_x[robot_id], self.final_partitioning_y[robot_id], marker="s",
                            s=self.plot_cell_size,
                            c=np.ones((len(self.final_partitioning_x[robot_id]), 3)) * self.partition_colors[robot_id])
            plt.axis("equal")
        elif "partial" in self.plot:
            plt.scatter(np.transpose(self.robot_initial_positions_in_cartesian)[0],
                        np.transpose(self.robot_initial_positions_in_cartesian)[1],
                        s=self.plot_robot_size, c="black")
            for robot_id in range(self.number_of_partitions):
                plt.scatter(self.final_partitioning_x[robot_id], self.final_partitioning_y[robot_id], marker="s",
                            s=self.cell_size,
                            c=np.ones((len(self.final_partitioning_x[robot_id]), 3)) * self.partition_colors[robot_id])
        self.data_tasks_per_robot = np.round(self.data_tasks_per_robot)
        self.area_per_robot = np.array(self.data_tasks_per_robot) * (self.cell_size ** 2)

        optimal_paths_clone = []
        for robot_id in range(self.number_of_partitions):
            optimal_paths_clone.append(np.array(self.optimal_paths[robot_id]))
        if "full" in self.plot:
            for robot_id in range(self.number_of_partitions):
                ax4.plot(optimal_paths_clone[robot_id][:, 0], optimal_paths_clone[robot_id][:, 1],
                         c=self.partition_colors[robot_id])
            # with open("Lejeune/optimal_paths_clone.txt", "wb") as fp:  # Pickling
            #     pickle.dump(optimal_paths_clone, fp)

        elif "partial" in self.plot:
            for robot_id in range(self.number_of_partitions):
                plt.plot(optimal_paths_clone[robot_id][:, 0], optimal_paths_clone[robot_id][:, 1],
                         c=self.partition_colors[robot_id])
                plt.axis("equal")

        if "area" in self.plot:
            plt.figure()
            plt.plot(self.area_covered_over_time_time_vector, self.area_covered_over_time)
            plt.title("Area Surveyed Over Time")
            plt.xlabel("Time (s)")
            plt.ylabel("Area Surveyed (Percentage)")
        plt.show()

    def calculate_surveillance_rate(self, waypoint_distances):
        """
        Calculate the total area surveyed as a function of time
        """
        # Calculate area coverage per second
        trip_distances = [0 for robot in range(self.number_of_partitions)]
        for robot in range(self.number_of_partitions):
            trip_distances[robot] = sum(waypoint_distances[robot])
        # np.save("Baseline_Environment/QLB_runs/trip_distances.npy", trip_distances)
        # np.save("Baseline_Environment/QLB_runs/trip_times.npy", [trip_distances[i] / self.robot_velocity for i in range(self.number_of_partitions)])

        initial_travel_distance = [0 for robot in range(self.number_of_partitions)]
        for robot, d in enumerate(waypoint_distances):
            initial_travel_distance[robot] = d[0]
        trip_distances = np.array(trip_distances) - np.array(initial_travel_distance)
        area_covered = 0
        t = 0
        self.data_completion_time_per_robot = [0 for robot in range(self.number_of_partitions)]
        while any(waypoint_distances):
            for robot in range(self.number_of_partitions):
                if trip_distances[robot] <= 0:
                    continue
                elif initial_travel_distance[robot] > 0:
                    initial_travel_distance[robot] -= self.robot_velocity * self.sampling_rate
                elif initial_travel_distance[robot] <= 0:
                    if len(waypoint_distances[robot]) != 0:
                        waypoint_distances[robot][0] -= self.robot_velocity * self.sampling_rate
                        if waypoint_distances[robot][0] <= 0:
                            waypoint_distances[robot].pop(0)
                            area_covered += self.cell_area
                    if len(waypoint_distances[robot]) == 0 and self.data_completion_time_per_robot[robot] == 0:
                        self.data_completion_time_per_robot[robot] += t
            self.area_covered_over_time_time_vector.append(round(t, 2))
            t += self.sampling_rate
            self.area_covered_over_time.append(round(area_covered / self.total_survey_area, 2))
        self.data_total_mission_completion_time = t