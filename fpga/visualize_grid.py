import matplotlib.pyplot as plt
import numpy as np
import re
import sys
import os

if not os.path.exists("output.txt"):
    print("Error: output.txt not found. Run your Verilator testbench first!")
    sys.exit(1)

# Create a blank 100x100 grid (0 to 99)
grid = np.zeros((100, 100))

# Parse the output file
with open("output.txt", "r") as f:
    for line in f:
        # Match format: Grid: [ 50, 75] | Weight: 20
        match = re.search(r"Grid:\s*\[\s*(\d+),\s*(\d+)\]\s*\|\s*Weight:\s*(\d+)", line)
        if match:
            x = int(match.group(1))
            y = int(match.group(2))
            weight = int(match.group(3))
            
            # Accumulate weight, clamping at 99
            if 0 <= x < 100 and 0 <= y < 100:
                grid[y][x] = min(grid[y][x] + weight, 99)

# Render the Heatmap
plt.figure(figsize=(8, 8))
# 'hot' colormap uses black for 0, mapping up to white for high confidence
plt.imshow(grid, cmap='hot', origin='lower', extent=[0, 100, 0, 100], vmin=0, vmax=99)
plt.colorbar(label='Confidence Weight (0-99)')
plt.title('LiDAR Occupancy Grid Simulation')
plt.xlabel('Memory X Index')
plt.ylabel('Memory Y Index')

# Mark the LiDAR center point
plt.scatter(50, 50, color='cyan', marker='^', s=100, label='UMARV Robot')
plt.legend()

plt.show()
