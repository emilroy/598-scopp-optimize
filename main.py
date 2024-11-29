"""
This code contains examples of how to call and use the SCoPP-Monitoring module.
"""
# Import the necessary modules:
import monitoring_algorithms
import environments as envs
import optimizer
import map_generator
import os

def generate_environment_map(env, filename):
    # Create map generator
    map_gen = map_generator.MapGenerator()
    
    # Generate the map with safety check for geo_fencing_holes
    # holes = getattr(env, 'geo_fencing_holes', None)
    map_gen.generate_map(env.boundary_points, env.geo_fencing_holes)
    
    # Save files
    map_gen.save_pgm(filename)

print("Input number for desired map:")
print("1: Small Lafayette Flood Map")
print("2: Medium Lafayette Flood Map")
print("3: Large Lafayette Flood Map")
input_map = input("> ")

if input_map == "1":
    environment = envs.SmallLafayetteFLood()
    survey_area = 332195.5
elif input_map == "2":
    environment = envs.MediumLafayetteFLood()
    survey_area = 1012089.5
else:
    environment = envs.LargeLafayetteFLood()
    survey_area = 3435918.5

# make folder for environment if doesn't exist already
if not os.path.exists(environment.__class__.__name__):
    os.makedirs(environment.__class__.__name__)

generate_environment_map(environment, "maps/" + environment.__class__.__name__ +".pgm")

# Initialize monitoring algorithm instance
optimize = optimizer.OptimizerFromImage("maps/" + environment.__class__.__name__ + ".yaml", survey_area)

# Initialize monitoring algorithm instance
way_point_allocator = monitoring_algorithms.QLB(environment, number_of_robots=20, plot="full")

# Run the algorithm on the given environment and display all information
paths = way_point_allocator.run(info="verbose")