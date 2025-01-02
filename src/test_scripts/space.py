from PIL import Image
import numpy as np

# Path to the PGM image file from YAML data
image_path = '/home/humanoidrobots/test_scripts/maps/room.pgm'

# Load the PGM image using PIL
img = Image.open(image_path)

# Convert the image to a NumPy array
img_array = np.array(img)

# Set thresholds based on free and occupied thresholds in the YAML file
free_thresh = 0.7 # As per your YAML data
occupied_thresh = 0  # As per your YAML data

# Normalize the image array to values between 0 and 1
img_array_normalized = img_array / 255.0

# Identify free and occupied spaces based on the threshold
free_spaces = []
occupied_spaces = []

# Iterate through each pixel in the array
height, width = img_array.shape
for y in xrange(height):  # Use xrange for Python 2
    for x in xrange(width):  # Use xrange for Python 2
        if img_array_normalized[y, x] <= free_thresh:
            free_spaces.append((x, y))  # Free space
        elif img_array_normalized[y, x] >= occupied_thresh:
            occupied_spaces.append((x, y))  # Occupied space

# Print results (or use them further)
print ("Free spaces:", len(free_spaces))
print ("Occupied spaces:", len(occupied_spaces))
