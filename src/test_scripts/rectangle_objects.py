from __future__ import print_function  # Enable Python 3 print behavior in Python 2
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
import yaml

# Load YAML metadata (your map's metadata)
with open('/home/humanoidrobots/test_scripts/maps/room.yaml', 'r') as file:
    map_metadata = yaml.load(file)  # Python 2 style loading

# Path to the PGM image file from YAML data
image_path = map_metadata['image']

# Load the PGM image using PIL
img = Image.open(image_path)

# Get the map resolution and size from YAML metadata
resolution = map_metadata['resolution']  # e.g., 0.05 meters per pixel
origin = map_metadata['origin']  # [x, y, z] coordinates of the origin

# Set custom threshold values for occupied and free space
occupied_thresh = 0.2  # Occupied space is from 0 to 0.2 (lower values)

# Convert the full map to a NumPy array
img_array = np.array(img)

# Normalize the image array to values between 0 and 1
img_array_normalized = img_array / 255.0

# Find the occupied areas (those below occupied_thresh)
occupied_map = img_array_normalized <= occupied_thresh

# Initialize visited array to track visited pixels
visited = np.zeros_like(occupied_map)

# Function to perform flood fill to find connected components
def flood_fill(occupied_map, visited, start_row, start_col):
    stack = [(start_row, start_col)]
    component = []
    while stack:
        row, col = stack.pop()
        if visited[row, col] == 0:  # If the pixel is not visited
            visited[row, col] = 1
            component.append((row, col))
            # Add all 4-neighbor pixels to the stack
            for r, c in [(row-1, col), (row+1, col), (row, col-1), (row, col+1)]:
                if 0 <= r < occupied_map.shape[0] and 0 <= c < occupied_map.shape[1]:
                    if occupied_map[r, c] and visited[r, c] == 0:
                        stack.append((r, c))
    return component

# Find all the objects (connected components)
objects = []
for row in range(occupied_map.shape[0]):
    for col in range(occupied_map.shape[1]):
        if occupied_map[row, col] and visited[row, col] == 0:
            # Start flood fill from unvisited occupied pixel
            component = flood_fill(occupied_map, visited, row, col)
            objects.append(component)

# Plot the map and show the border and object rectangles
fig, ax = plt.subplots()

# Display the map
ax.imshow(img_array_normalized, cmap='gray')

# Add border of the map (black border around the full map)
rect = plt.Rectangle((0, 0), img_array.shape[1], img_array.shape[0], linewidth=2, edgecolor='blue', facecolor='none')
ax.add_patch(rect)

# Loop through each detected object and draw a rectangle around it
for object_id, component in enumerate(objects, 1):
    # Get the min and max row and column indices for the bounding box
    rows, cols = zip(*component)
    min_row, max_row = min(rows), max(rows)
    min_col, max_col = min(cols), max(cols)

    # Convert the pixel coordinates of the bounding box to real-world coordinates
    x_min = min_col * resolution + origin[0]
    y_min = min_row * resolution + origin[1]
    x_max = max_col * resolution + origin[0]
    y_max = max_row * resolution + origin[1]

    # Draw a rectangle around the object
    rect = plt.Rectangle((min_col, min_row), max_col - min_col, max_row - min_row, linewidth=2, edgecolor='red', facecolor='none')
    ax.add_patch(rect)

    # Optionally, print out the bounding box in real-world coordinates
    print("Object {}: Xmin = {:.2f}, Ymin = {:.2f}, Xmax = {:.2f}, Ymax = {:.2f}".format(
        object_id, x_min, y_min, x_max, y_max))

    # Add the object number inside the bounding box
    ax.text(min_col + (max_col - min_col) / 2, min_row + (max_row - min_row) / 2, str(object_id),
            color='white', fontsize=10, ha='center', va='center', fontweight='bold')

# Show the plot with the map and rectangles
plt.title("Map with Border, Object Rectangles (Red) and Numbers")
plt.show()
