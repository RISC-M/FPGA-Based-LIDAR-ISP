import matplotlib.pyplot as plt
import numpy as np

# Create an empty 100x100 grid
grid = np.zeros((100, 100))

# Load the hardware data
with open("output_map.txt", "r") as f:
    for line in f:
        parts = line.strip().split()
        if len(parts) == 3:
            x, y, weight = map(int, parts)
            grid[y, x] = weight # Note: numpy arrays are [row(y), col(x)]

# Plot the heatmap
plt.figure(figsize=(8, 8))
plt.imshow(grid, cmap='hot', origin='lower', vmin=0, vmax=100)
plt.colorbar(label='Occupancy Confidence (%)')
plt.title("FPGA Output: 2000mm Circle")
plt.xlabel("Grid X")
plt.ylabel("Grid Y")
plt.grid(color='gray', linestyle='--', linewidth=0.5, alpha=0.3)
plt.show()
