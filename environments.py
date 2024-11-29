"""This code contains environment parameters to be loaded for use by the SCoPP algorithm. New classes should be created
and stored here for any new environments that the user wishes to use the algorithm on. Simply copy and paste one of the
environments below and alter the values to your liking. Each class follows the same format and require the following
attributes:

    starting_position: list of lists - Starting position (in geographical coordinates) for the robots. If only one
        position is given, all robots start at that position)
    boundary_points: list of lists - Vertices of the polygon which defines the entire survey area (in
        geographical coordinates). The vertices must be in the order which they are connected; either clockwise or
        counterclockwise.
    geo_fencing_holes: list of lists of lists - Vertices of the polygon which defines each discontnuity in the survey
        area (in geographical coordinates). The vertices must be in the order which they are connected; either clockwise
        or counterclockwise.
    robot_FOV: int, float - Downward field of view of the robots in degrees
    robot_operating_height: int, float - Height at which the robots will be flying
    robot_velocity: int, float - Velocity of the robots
    save_path: string - Directory for output data to be sent
**Optional:
    UAV: int - used to store and choose between multiple UAV parameters for a single environment
"""


class Debugger:
    """Robot parameter Class for debugging. This is a simple polygon with a low total area to reduce computation time
    substantially, to make debugging much faster
    """

    def __init__(self):
        self.starting_position = [[40.68251, -73.91134]]
        self.boundary_points = [[40.68251, -73.91134], [40.68250, -73.90935],
                                [40.68173, -73.90935], [40.68176, -73.91138]]
        self.geo_fencing_holes = None
        self.robot_FOV = 150  # degrees
        self.robot_operating_height = 2  # meters
        self.robot_velocity = 10  # meters per second
        self.save_path = "Debug/"

class SmallLafayetteFLood:
    def __init__(self):
        self.boundary_points = [[30.2472, -92.151],
                                [30.247, -92.1426],
                                [30.2464, -92.1427],
                                [30.2418, -92.1472],
                                [30.243, -92.1501],
                                [30.245, -92.1516]]

        self.starting_position = [[30.2436, -92.145]]
        self.geo_fencing_holes = []
        self.save_path = "SmallLafayetteFlood/"
        # DJI Phantom 4 Pro (max flight range: 7km)
        self.robot_FOV = 75  # degrees
        self.robot_operating_height = 10  # meters
        self.robot_velocity = 10  # meters per second

class MediumLafayetteFLood:
    def __init__(self):
        self.boundary_points = [[30.24610, -92.03380],
                                [30.24430, -92.04200],
                                [30.23530, -92.04290],
                                [30.23480, -92.03470],
                                [30.24290, -92.03210]]
        self.geo_fencing_holes = []
        # DJI Phantom 4 Pro (max flight range: 7km)
        self.robot_FOV = 75  # degrees
        self.robot_operating_height = 10  # meters
        self.robot_velocity = 10  # meters per second


class LargeLafayetteFLood:
    def __init__(self):
        self.boundary_points = [[30.27560, -92.12400],
                                [30.28350, -92.11940],
                                [30.28590, -92.12670],
                                [30.28990, -92.12330],
                                [30.29000, -92.13870],
                                [30.28180, -92.14530],
                                [30.27760, -92.13980],
                                [30.27460, -92.13650],
                                [30.27330, -92.13050]]
        self.geo_fencing_holes = []
        self.starting_position = [[30.24686, -92.03722]]
        self.save_path = "LargeLafayetteFLood/"
        # DJI Phantom 4 Pro (max flight range: 7km)
        self.robot_FOV = 75  # degrees
        self.robot_operating_height = 10  # meters
        self.robot_velocity = 10  # meters per second