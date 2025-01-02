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

# Convert the entire image to real-world coordinates
# Get the indices of occupied space (those that are <= occupied_thresh)
occupied_indices = np.where(img_array_normalized <= occupied_thresh)

# Convert the indices to real-world coordinates
occupied_positions = []
for row, col in zip(occupied_indices[0], occupied_indices[1]):
    # Convert pixel (row, col) to real-world coordinates (x, y)
    x = (col * resolution) + origin[0]
    y = (row * resolution) + origin[1]
    occupied_positions.append((x, y))

# Show the positions of the occupied objects
print("Occupied positions (real-world coordinates):")
for position in occupied_positions:
    print("X: {:.2f} meters, Y: {:.2f} meters".format(position[0], position[1]))

# Optionally, you can visualize the occupied positions on the map
plt.imshow(img_array_normalized, cmap='gray')
plt.scatter([pos[0] for pos in occupied_positions], [pos[1] for pos in occupied_positions], color='red', label="Occupied")
plt.title("Occupied Positions (Red) on Full Map")
plt.legend()
plt.show()
