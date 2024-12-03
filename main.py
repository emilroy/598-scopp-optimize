"""
This code contains examples of how to call and use the SCoPP-Monitoring module.
"""
# Import the necessary modules:
import monitoring_algorithms
import environments as envs
import os

# Initialize environment class
environment = envs.Benning(1)

if not os.path.exists(environment.__class__.__name__):
    os.makedirs(environment.__class__.__name__)  # Create the folder

# Initialize monitoring algorithm instance
way_point_allocator = monitoring_algorithms.QLB(environment, number_of_robots=30,plot="full")

# Run the algorithm on the given environment and display all information
paths = way_point_allocator.run(info="verbose")