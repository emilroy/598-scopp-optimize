import numpy as np
import pickle
import matplotlib.pyplot as plt
from matplotlib.offsetbox import OffsetImage, AnnotationBbox
import imageio
from math import sqrt
import environments as tap  # Ensure the 'environments' module is correctly configured

# Load parameters
parameters = tap.MediumLafayetteFLood(1) # change environment here =================
save_path = parameters.save_path
sampling_rate = 1 / 10

# Load data
with open(save_path + "waypoints", "rb") as f:
    paths = pickle.load(f)
coverage = np.load(save_path + "area_coverage.npy").tolist()
velocity = parameters.robot_velocity
t = np.linspace(0, (len(coverage) - 1) * sampling_rate, int(len(coverage)))

# Truncate coverage to the maximum point
for index, data_point in enumerate(coverage):
    if data_point == max(coverage):
        del coverage[index + 1:]

# Initialize path variables
tol = sampling_rate * 8
rt_path_x = [[] for _ in range(len(paths))]
rt_path_y = [[] for _ in range(len(paths))]
rt_path_cells = [[] for _ in range(len(paths))]
max_x, max_y = np.NINF, np.NINF

# Find map bounds
for path in paths:
    for position in path:
        max_x = max(max_x, position[0])
        max_y = max(max_y, position[1])

# Load map image
map_png = plt.imread(save_path + "map.png")

# Simulation loop
for iteration, time in enumerate(t):
    for agent, path in enumerate(paths):
        if len(path) > 1:
            rt_path_x[agent].append(path[0][0])
            rt_path_y[agent].append(path[0][1])
            direction_x = (path[1][0] - path[0][0]) / sqrt((path[1][0] - path[0][0]) ** 2 +
                                                           (path[1][1] - path[0][1]) ** 2)
            direction_y = (path[1][1] - path[0][1]) / sqrt((path[1][0] - path[0][0]) ** 2 +
                                                           (path[1][1] - path[0][1]) ** 2)
            path[0][0] += direction_x * velocity * sampling_rate
            path[0][1] += direction_y * velocity * sampling_rate
            if sqrt((path[1][0] - path[0][0]) ** 2 + (path[1][1] - path[0][1]) ** 2) < tol:
                paths[agent].pop(0)
                rt_path_cells[agent].append(1)
            else:
                rt_path_cells[agent].append(0)
        else:
            rt_path_x[agent].append(path[0][0])
            rt_path_y[agent].append(path[0][1])
            rt_path_cells[agent].append(0)

# Visualization and GIF creation
scrubbing_speed = 50
fig, ax = plt.subplots(figsize=(10, 8))
# fig.canvas.manager.full_screen_toggle() # toggle fullscreen mode
# fig.set_size_inches(15, 5)
plt.axis([0, max_x + 5, 0, max_y + 5])
plt.axis("off")
# ax.imshow(map_png, extent=[0, max_x + 5, 0, max_y + 5])

uav_image = OffsetImage(plt.imread("uav_icon.png"), zoom=0.05)
image_list = []

# Generate animation frames
for iteration in range(int(len(coverage) / scrubbing_speed)):
    ax.clear()
    # ax.imshow(map_png, extent=[0, max_x + 5, 0, max_y + 5])
    for agent in range(len(paths)):
        x = rt_path_x[agent][iteration * scrubbing_speed]
        y = rt_path_y[agent][iteration * scrubbing_speed]
        ax.plot(rt_path_x[agent][:iteration * scrubbing_speed],
                rt_path_y[agent][:iteration * scrubbing_speed], c="blue")
        ab = AnnotationBbox(uav_image, (x, y), frameon=False)
        ax.add_artist(ab)

    # Render and capture the frame
    fig.canvas.draw()
    width, height = fig.canvas.get_width_height()
    image = np.frombuffer(fig.canvas.tostring_rgb(), dtype='uint8')
    image_list.append(image.reshape((height, width, 3)))

# Save as GIF
imageio.mimsave(save_path + "coverage2.gif", image_list, fps=10)
print("Simulation complete. GIF saved to:", save_path + "coverage2.gif")
