# generate_points.py
import sys

filename = "stimulus.txt"

with open(filename, "w") as f:
    azimuth = 0
    # Generate 90 clock cycles of data (2 points per cycle = 180 points, covering 360 degrees)
    for i in range(90):
        valid = 3 # 2'b11 (both lanes valid)
        
        # Lane 0
        dist0 = 2000        # 2000 mm distance
        az0 = azimuth * 100 # Q1.15 format (e.g., 2.00 degrees = 200)
        lid0 = 0            # Laser 0 (-15 degrees elevation)
        
        azimuth += 2 # Increment azimuth by 2 degrees
        
        # Lane 1
        dist1 = 2000
        az1 = azimuth * 100
        lid1 = 0
        
        azimuth += 2
        
        # Write as hex: VALID DIST0 AZ0 LID0 DIST1 AZ1 LID1
        # Example output: 3 07d0 0000 0 07d0 00c8 0
        f.write(f"{valid:1x} {dist0:04x} {az0:04x} {lid0:1x} {dist1:04x} {az1:04x} {lid1:1x}\n")

print(f"Generated test points in {filename}")
