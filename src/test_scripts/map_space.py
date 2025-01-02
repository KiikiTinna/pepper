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
free_thresh = 0.9  # Free space is from 0.8 to 1 (higher values)

# Convert the full map to a NumPy array
img_array = np.array(img)

# Normalize the image array to values between 0 and 1
img_array_normalized = img_array / 255.0

# Find the non-zero pixels to detect the area of the map (cropping)
non_zero_pixels = np.where(img_array_normalized > 0)  # Get indices of non-zero pixels

# Get the bounding box (min and max row/column of non-zero pixels)
min_row, max_row = min(non_zero_pixels[0]), max(non_zero_pixels[0])
min_col, max_col = min(non_zero_pixels[1]), max(non_zero_pixels[1])

# Crop the image to the bounding box (only the region with map data)
cropped_img_array = img_array[min_row:max_row + 1, min_col:max_col + 1]

# Normalize the cropped image
cropped_img_array_normalized = cropped_img_array / 255.0

# Create a color map for free and occupied spaces (RGB format)
# Initialize a black image (for RGB channels)
cropped_map_color = np.zeros((cropped_img_array.shape[0], cropped_img_array.shape[1], 3))

# Apply thresholding:
# Mark free space (between 0.8 and 1) with green (RGB: [0, 1, 0])
cropped_map_color[cropped_img_array_normalized >= free_thresh] = [0, 1, 0]  # Free space in green

# Mark occupied space (between 0 and 0.2) with red (RGB: [1, 0, 0])
cropped_map_color[cropped_img_array_normalized <= occupied_thresh] = [1, 0, 0]  # Occupied space in red

# Show the color-coded cropped map
plt.imshow(cropped_map_color)
plt.title("Cropped Map (Free = Green, Occupied = Red)")
plt.show()

# Optionally print out the cropped region details
print("Cropped region coordinates: ")
print("Min Row:", min_row, "Max Row:", max_row)
print("Min Col:", min_col, "Max Col:", max_col)

# Print the resolution and origin
print("Map Resolution: ", resolution, "meters per pixel")
print("Map Origin: ", origin)
