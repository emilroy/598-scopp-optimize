"""
This code contains examples of how to call and use the SCoPP-Monitoring module.
"""
# Import the necessary modules:
import monitoring_algorithms
import environments as envs

# Initialize environment class
environment = envs.MediumLafayetteFLood()

# Initialize monitoring algorithm instance
way_point_allocator = monitoring_algorithms.QLB(environment, number_of_robots=30,plot="full")

# Run the algorithm on the given environment and display all information
paths = way_point_allocator.run(info="verbose")