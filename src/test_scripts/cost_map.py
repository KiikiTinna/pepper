import numpy as np
import yaml
from PIL import Image

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
free_thresh = 0.9  # Free space is from 0.8 and above

# Convert the full map to a NumPy array
img_array = np.array(img)

# Normalize the image array to values between 0 and 1
img_array_normalized = img_array / 255.0

# Create the cost map based on occupancy grid
cost_map = np.ones_like(img_array_normalized) * 100  # Initialize with high costs

# Assign lower cost values for free spaces and higher cost for obstacles
cost_map[img_array_normalized <= occupied_thresh] = 100  # High cost for occupied spaces
cost_map[img_array_normalized >= free_thresh] = 1  # Low cost for free spaces
cost_map[(img_array_normalized > occupied_thresh) & (img_array_normalized < free_thresh)] = 50  # Medium cost for unknown areas

# Displaying the cost map (optional: for visualization)
import matplotlib.pyplot as plt

plt.imshow(cost_map, cmap='hot', interpolation='nearest')  # Use 'hot' color map for better visualization
plt.title("Cost Map")
plt.colorbar(label='Cost')
plt.show()

# For debugging: print part of the cost map
print(cost_map[:10, :10])  # Print a small p