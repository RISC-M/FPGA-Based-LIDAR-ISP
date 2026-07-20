# FPGA LiDAR Occupancy-Grid Accelerator

This project uses a pipelined LiDAR data processor in FPGA fabric (DE10-NANO) to turn VLP-16 returns into a 100 x 100 occupancy grid. The HPS receives the LiDAR stream and the FPGA performs the per-point processing.

## Demo

[Watch the LiDAR demo](notes/LiDAR%20iSP%20Demo.mp4)


## FPGA pipeline

The processor handles two LiDAR returns at a time in three stages:

1. **Transform:** looks up trigonometric coefficients and converts range, azimuth, and laser ID into fixed-point `x`, `y`, and `z` coordinates.
2. **Filter and map:** removes points below the configured height threshold or outside the grid, then maps accepted points to occupancy-grid cells.
3. **Read-modify-write:** updates the grid confidence value for each cell, using banked memory and hazard handling to support two point lanes.

The occupancy grid is double-buffered: the FPGA writes the next frame while the HPS reads, displays, and clears the completed frame.

## HPS software

[`hps/lidar_script.c`](hps/lidar_script.c) is the runtime program. It receives VLP-16 UDP packets, packs pairs of returns into three 32-bit writes for the FPGA, detects each full LiDAR rotation, swaps the grid buffer, and renders the completed grid in the terminal.
