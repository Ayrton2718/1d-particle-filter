#!/usr/bin/env python3
import numpy as np

# Map dimensions (200x200 pixels)
width, height = 500, 500
maxval = 255  # Maximum grayscale value (white)

# Create an image filled with white (free space)
img = 0 * np.ones((height, width), dtype=np.uint8)

# --- Obstacle 1: A Circle in the Center ---
center_x, center_y = width // 2, height // 2
radius = 150  # Radius of the circle in pixels
for y in range(height):
    for x in range(width):
        if (x - center_x) ** 2 + (y - center_y) ** 2 <= radius ** 2:
            img[y, x] = 255

# Write the image to a PGM file in binary format (P5)
pgm_filename = 'map.pgm'
with open(pgm_filename, 'wb') as f:
    header = f"P5\n# Big map with a circle and a square obstacle\n{width} {height}\n{maxval}\n"
    f.write(header.encode('ascii'))
    f.write(img.tobytes())

print(f"{pgm_filename} generated successfully.")

# --- Generate the YAML File ---
# This YAML file provides the necessary parameters for applications (e.g., ROS map server)
yaml_filename = 'map.yaml'
yaml_content = f"""\
image: {pgm_filename}         # Path to the PGM image file
resolution: 0.1                # Resolution [meters/pixel]
origin: [-25.0, -25.0, 0.0]       # The map's origin [x, y, theta]
negate: 0                   # Do not invert pixel values (0 = false)
occupied_thresh: 0.65        # Pixels with normalized values above this are considered occupied
free_thresh: 0.196           # Pixels with normalized values below this are considered free
"""

with open(yaml_filename, 'w') as f:
    f.write(yaml_content)

print(f"{yaml_filename} generated successfully.")
