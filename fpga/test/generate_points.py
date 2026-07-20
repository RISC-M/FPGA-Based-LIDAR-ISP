import sys

# We generate a 360 degree sweep, distance 2000mm, laser ID 0 (flat).
# We need to output two points per line to feed Lane 1 and Lane 0 simultaneously.

filename = "stimulus.txt"
with open(filename, "w") as f:
    # 360 degrees, step by 2 so we have 180 points total (90 clock cycles)
    for i in range(0, 360, 2):
        # Point 1 (Lane 1)
        az1 = (i + 1) * 100 # Multiply by 100 for your LUT logic
        d1 = 2000
        l1 = 0
        v1 = 1
        
        # Point 0 (Lane 0)
        az0 = i * 100
        d0 = 2000
        l0 = 0
        v0 = 1
        
        # Format: valid1 distance1 azimuth1 laser_id1 valid0 distance0 azimuth0 laser_id0
        f.write(f"{v1} {d1} {az1} {l1} {v0} {d0} {az0} {l0}\n")

print(f"Generated {filename} with 90 cycles of dual-lane stimulus.")