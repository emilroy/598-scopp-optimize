"""
This code contains examples of how to call and use the SCoPP-Monitoring module.
"""
# Import the necessary modules:
import monitoring_algorithms
import environments as envs
import os

# Initialize environment class
environment = envs.SmallLafayetteFLood(1, ["disc","path"]) # change environment here  =========================================
print("Environment:", environment.__class__.__name__)

if not os.path.exists(environment.save_path):
    os.makedirs(environment.save_path)  # Create the folder

# Initialize monitoring algorithm instance
way_point_allocator = monitoring_algorithms.QLB(environment, number_of_robots=15,plot="full")
print("Algorithm:", way_point_allocator.__class__.__name__)

# Run the algorithm on the given environment and display all information
paths = way_point_allocator.run(info="verbose")