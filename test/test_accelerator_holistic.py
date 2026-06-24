import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ReadOnly, Timer
import os
import random

# Pack elements into a 74-bit packed structure mapping INGRESS_PACKET
def pack_ingress_packet(v1, d1, az1, l1, v0, d0, az0, l0):
    val = 0
    # valid[1] is bit 73, valid[0] is bit 72
    val |= (v1 & 1) << 73
    val |= (v0 & 1) << 72
    # distance[1] is bits 71:56, distance[0] is bits 55:40
    val |= (d1 & 0xFFFF) << 56
    val |= (d0 & 0xFFFF) << 40
    # azimuth[1] is bits 39:24, azimuth[0] is bits 23:8
    val |= (az1 & 0xFFFF) << 24
    val |= (az0 & 0xFFFF) << 8
    # laser_id[1] is bits 7:4, laser_id[0] is bits 3:0
    val |= (l1 & 0xF) << 4
    val |= (l0 & 0xF) << 0
    return val

# Driver simulating memory-mapped AXI-Avalon bridge accesses from HPS
class HPSBridgeDriver:
    def __init__(self, dut, clk):
        self.dut = dut
        self.clk = clk
        
    async def write(self, addr, val):
        """Simulate a single MM CPU write to the memory block."""
        self.dut.hps_we.value = 1
        self.dut.hps_write_addr.value = addr
        self.dut.hps_data_in_flat.value = val
        await RisingEdge(self.clk)
        
        self.dut.hps_we.value = 0
        self.dut.hps_write_addr.value = 0
        self.dut.hps_data_in_flat.value = 0
        # Simulating standard MM bridge spacing/idle cycles (CPU bus delay)
        for _ in range(3):
            await RisingEdge(self.clk)
            
    async def read(self, addr):
        """Simulate a single MM CPU read from the memory block."""
        self.dut.hps_read_addr.value = addr
        await RisingEdge(self.clk)
        
        # Read has a 1-cycle latency (synchronous memory output)
        await ReadOnly()
        val = int(self.dut.hps_data_out_flat.value)
        
        # We must exit ReadOnly phase before setting values. Wait for the next edge.
        await RisingEdge(self.clk)
        self.dut.hps_read_addr.value = 0
        
        for _ in range(2):
            await RisingEdge(self.clk)
        return val


# Software Reference Model (Golden Model) of the hardware accelerator
class GoldModel:
    def __init__(self, rom_path):
        self.rom = {}
        self.mem0 = [0] * 10000  # Ping Buffer
        self.mem1 = [0] * 10000  # Pong Buffer
        self.ctr = 0
        
        with open(rom_path, "r") as f:
            for idx, line in enumerate(f):
                line = line.strip()
                if line:
                    val = int(line, 16)
                    x = self._to_signed_16((val >> 32) & 0xFFFF)
                    y = self._to_signed_16((val >> 16) & 0xFFFF)
                    z = self._to_signed_16(val & 0xFFFF)
                    self.rom[idx] = (x, y, z)
                    
    def _to_signed_16(self, val):
        return val - 65536 if val >= 32768 else val
        
    def process_point(self, v1, d1, az1, l1, v0, d0, az0, l0, gnd_thresh=-2000, hit_weight=20):
        results = []
        
        # Process Lane 1
        l1_hit = None
        if v1:
            az1_deg = (az1 * 655) >> 16
            rom_addr1 = (l1 << 9) | az1_deg
            x_coeff1, y_coeff1, z_coeff1 = self.rom.get(rom_addr1, (0, 0, 0))
            
            x1 = (d1 * x_coeff1) >> 15
            y1 = (d1 * y_coeff1) >> 15
            z1 = (d1 * z_coeff1) >> 15
            
            scaled_x1 = (x1 * 1311) >> 16
            scaled_y1 = (y1 * 1311) >> 16
            
            mem_x1 = scaled_x1 + 50
            mem_y1 = scaled_y1 + 50
            
            if z1 >= gnd_thresh and 0 <= mem_x1 < 100 and 0 <= mem_y1 < 100:
                l1_hit = (mem_y1 * 100 + mem_x1, hit_weight)
                
        # Process Lane 0
        l0_hit = None
        if v0:
            az0_deg = (az0 * 655) >> 16
            rom_addr0 = (l0 << 9) | az0_deg
            x_coeff0, y_coeff0, z_coeff0 = self.rom.get(rom_addr0, (0, 0, 0))
            
            x0 = (d0 * x_coeff0) >> 15
            y0 = (d0 * y_coeff0) >> 15
            z0 = (d0 * z_coeff0) >> 15
            
            scaled_x0 = (x0 * 1311) >> 16
            scaled_y0 = (y0 * 1311) >> 16
            
            mem_x0 = scaled_x0 + 50
            mem_y0 = scaled_y0 + 50
            
            if z0 >= gnd_thresh and 0 <= mem_x0 < 100 and 0 <= mem_y0 < 100:
                l0_hit = (mem_y0 * 100 + mem_x0, hit_weight)
                
        # Reduction Tree (consolidate if they land in the exact same cell)
        if l1_hit and l0_hit and l1_hit[0] == l0_hit[0]:
            results.append((l0_hit[0], hit_weight * 2))
        else:
            if l0_hit:
                results.append(l0_hit)
            if l1_hit:
                results.append(l1_hit)
                
        return results

    def write_acc(self, addr, increment):
        target_mem = self.mem1 if self.ctr == 1 else self.mem0
        target_mem[addr] = min(target_mem[addr] + increment, 99)

    def write_hps(self, addr, val):
        target_mem = self.mem0 if self.ctr == 1 else self.mem1
        target_mem[addr] = val

    def read_hps(self, addr):
        target_mem = self.mem0 if self.ctr == 1 else self.mem1
        return target_mem[addr]

    def toggle_switch(self):
        self.ctr = 1 - self.ctr


async def reset_dut(dut):
    dut.rst_n.value = 0
    dut.buffer_switch.value = 0
    dut.data_in_flat.value = 0
    dut.hps_we.value = 0
    dut.hps_write_addr.value = 0
    dut.hps_data_in_flat.value = 0
    dut.hps_read_addr.value = 0
    await Timer(20, units="ns")
    await RisingEdge(dut.clk)
    dut.rst_n.value = 1
    await RisingEdge(dut.clk)


async def clear_acc_buffer(dut, hps, addresses):
    # To clear the active accelerator buffer, we swap it to HPS, write 0, and swap back.
    dut.buffer_switch.value = 1
    await RisingEdge(dut.clk)
    dut.buffer_switch.value = 0
    await RisingEdge(dut.clk)
    
    for addr in addresses:
        await hps.write(addr, 0)
        
    dut.buffer_switch.value = 1
    await RisingEdge(dut.clk)
    dut.buffer_switch.value = 0
    await RisingEdge(dut.clk)


@cocotb.test()
async def test_reset_and_defaults(dut):
    """Test Case 1: Verification of reset state and default memory values."""
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    hps = HPSBridgeDriver(dut, dut.clk)
    await reset_dut(dut)
    
    # Read random memory addresses immediately after reset. They should be 0.
    for _ in range(20):
        addr = random.randint(0, 9999)
        val = await hps.read(addr)
        assert val == 0, f"Memory address {addr} is not 0 after reset: got {val}"
    dut._log.info("Reset and defaults verification passed successfully.")


@cocotb.test()
async def test_out_of_bounds_and_filtering(dut):
    """Test Case 2: Verification of out-of-bounds coordinate filtering and ground thresholds."""
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    hps = HPSBridgeDriver(dut, dut.clk)
    await reset_dut(dut)
    
    # Send a packet that is valid but has coordinates landing far outside the 100x100 grid
    # E.g., distance 8000mm, which projects to coordinates > 100.
    # Format: v1 d1 az1 l1 v0 d0 az0 l0
    packed_oob = pack_ingress_packet(
        v1=1, d1=8000, az1=4500, l1=0,  # X, Y projects out of bounds
        v0=1, d0=8000, az0=13500, l0=0  # X, Y projects out of bounds
    )
    
    dut.data_in_flat.value = packed_oob
    await RisingEdge(dut.clk)
    dut.data_in_flat.value = 0
    
    for _ in range(10):
        await RisingEdge(dut.clk) # Wait to drain
        
    # Toggle switch to allow HPS to read
    dut.buffer_switch.value = 1
    await RisingEdge(dut.clk)
    dut.buffer_switch.value = 0
    await RisingEdge(dut.clk)
    
    # Read entire memory to ensure no stray writes happened
    for i in range(10000):
        val = await hps.read(i)
        assert val == 0, f"Write occurred at {i} for out-of-bounds packet: got {val}"
        
    dut._log.info("Out-of-bounds filtering verified successfully.")


@cocotb.test()
async def test_read_after_write_forwarding(dut):
    """Test Case 3: Verification of Read-After-Write forwarding logic inside stage_rmw."""
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    hps = HPSBridgeDriver(dut, dut.clk)
    await reset_dut(dut)
    await clear_acc_buffer(dut, hps, [5050])
    
    # We will send multiple packets hitting the exact same cell (50, 50) consecutively.
    # Address = 50 * 100 + 50 = 5050.
    # Since distance is 0, coordinate is scaled_x = 0, scaled_y = 0.
    # shifted coordinates: 0 + 50 = 50, 0 + 50 = 50.
    # Thus, distance = 0 will always project to (50, 50).
    packed_raw = pack_ingress_packet(
        v1=1, d1=0, az1=0, l1=0,
        v0=1, d0=0, az0=0, l0=0
    )
    
    # Inject this packet 3 times consecutively
    # First write: increments to 40 (reduction tree consolidates)
    # Second write: increments to 40 + 40 = 80 (relies on RAW forwarding)
    # Third write: increments to 80 + 40 = 120 -> saturates at 99 (RAW forwarding)
    for _ in range(3):
        await ReadOnly()
        assert dut.stall.value == 0, "Spurious stall encountered."
        await RisingEdge(dut.clk)
        dut.data_in_flat.value = packed_raw
        
    await RisingEdge(dut.clk)
    dut.data_in_flat.value = 0
    
    # Drain
    for _ in range(15):
        await RisingEdge(dut.clk)
        
    # Toggle switch
    dut.buffer_switch.value = 1
    await RisingEdge(dut.clk)
    dut.buffer_switch.value = 0
    await RisingEdge(dut.clk)
    
    val = await hps.read(5050)
    assert val == 99, f"RAW accumulation failed to saturate. Expected 99, got {val}"
    
    # Clean up the RAM cell before completing the test
    await hps.write(5050, 0)
    dut._log.info("RAW forwarding and saturation verified successfully.")


@cocotb.test()
async def test_bank_conflicts_and_stalls(dut):
    """Test Case 4: Verification of bank conflict stalling and resolving behavior."""
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    hps = HPSBridgeDriver(dut, dut.clk)
    await reset_dut(dut)
    await clear_acc_buffer(dut, hps, [5050, 5150])
    
    # Send a packet where:
    # Lane 0 projects to address 5050 (Bank 0)
    # Lane 1 projects to address 5150 (Bank 0)
    # Both lanes are valid, and both map to Bank 0 (even address).
    # This must cause a bank conflict stall!
    
    # To get address 5150: X=50, Y=51.
    # From ROM: 0 degrees, laser 0, x_coeff = 0, y_coeff = cos(-15) = 0.9659, z_coeff = sin(-15).
    # So projects to:
    # Lane 0: dist = 0 -> maps to (50, 50) -> address 5050 (Bank 0)
    # Lane 1: dist = 100, az = 0 -> projects to x = 0, y = 96 -> mem_x = 50, mem_y = 51 -> address 5150 (Bank 0)
    
    packed_conflict = pack_ingress_packet(
        v1=1, d1=100, az1=0, l1=0, # (50, 51) -> 5150
        v0=1, d0=0, az0=0, l0=0    # (50, 50) -> 5050
    )
    
    await RisingEdge(dut.clk)
    dut.data_in_flat.value = packed_conflict
    
    # Wait for the data to propagate to stage_rmw (takes 3 clock cycles)
    await RisingEdge(dut.clk) # Edge 1 (DUT samples packed_conflict)
    dut.data_in_flat.value = 0 # Clear immediately so we do not feed new conflicts!
    
    await RisingEdge(dut.clk) # Edge 2 (DUT registers t2f_pipe)
    await RisingEdge(dut.clk) # Edge 3 (DUT registers f2r_pipe, triggers stall)
    
    # Check that stall goes high
    await ReadOnly()
    assert dut.stall.value == 1, "Bank conflict failed to trigger pipeline stall."
    
    # The stall should resolve in the next cycle (resolving_stall goes high, stall goes low)
    await RisingEdge(dut.clk) # Edge 4
    await ReadOnly()
    assert dut.stall.value == 0, "Bank conflict stall failed to resolve after 1 cycle."
    
    # Drain
    for _ in range(15):
        await RisingEdge(dut.clk)
        
    # Toggle switch
    dut.buffer_switch.value = 1
    await RisingEdge(dut.clk)
    dut.buffer_switch.value = 0
    await RisingEdge(dut.clk)
    
    val0 = await hps.read(5050)
    val1 = await hps.read(5150)
    
    assert val0 == 20, f"Expected weight 20 at 5050, got {val0}"
    assert val1 == 20, f"Expected weight 20 at 5150, got {val1}"
    
    # Clean up the RAM cells before completing the test
    await hps.write(5050, 0)
    await hps.write(5150, 0)
    dut._log.info("Bank conflict stalling and resolution verified successfully.")


@cocotb.test()
async def test_ping_pong_buffer_operations(dut):
    """Test Case 5: Verification of memory-mapped double-buffer routing and HPS clearance."""
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    hps = HPSBridgeDriver(dut, dut.clk)
    await reset_dut(dut)
    await clear_acc_buffer(dut, hps, [5050])
    
    # Write a point to the grid
    packed_pt = pack_ingress_packet(
        v1=0, d1=0, az1=0, l1=0,
        v0=1, d0=0, az0=0, l0=0 # Maps to (50, 50) -> address 5050
    )
    
    dut.data_in_flat.value = packed_pt
    await RisingEdge(dut.clk)
    dut.data_in_flat.value = 0
    
    for _ in range(15):
        await RisingEdge(dut.clk)
        
    # Before buffer switch: HPS reads should see 0 (reads inactive buffer)
    val_pre = await hps.read(5050)
    assert val_pre == 0, f"HPS read active buffer prematurely: got {val_pre}"
    
    # Switch ping-pong buffers
    dut.buffer_switch.value = 1
    await RisingEdge(dut.clk)
    dut.buffer_switch.value = 0
    await RisingEdge(dut.clk)
    
    # After buffer switch: HPS should see the point
    val_post = await hps.read(5050)
    assert val_post == 20, f"HPS failed to read switched point: got {val_post}"
    
    # Clear the point over the HPS virtual bridge (simulating clearing memory map for next frame)
    await hps.write(5050, 0)
    
    # Verify the cell is cleared
    val_cleared = await hps.read(5050)
    assert val_cleared == 0, f"HPS failed to write/clear memory: got {val_cleared}"
    
    # Toggle switch back
    dut.buffer_switch.value = 1
    await RisingEdge(dut.clk)
    dut.buffer_switch.value = 0
    await RisingEdge(dut.clk)
    
    # The cell should be 0 on the other buffer (since it has not been written to by the acc in this test)
    val_swapped_back = await hps.read(5050)
    assert val_swapped_back == 0, f"Expected swapped buffer to be clean: got {val_swapped_back}"
    
    dut._log.info("Double-buffer ping-pong operations verified successfully.")


@cocotb.test()
async def test_random_lidar_sweep_vs_golden_model(dut):
    """Test Case 6: Holistic verification of randomized sensor input against software golden model."""
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    hps = HPSBridgeDriver(dut, dut.clk)
    await reset_dut(dut)
    
    rom_path = os.path.join(os.path.dirname(__file__), "azimuth_sincos.mem")
    gold = GoldModel(rom_path)
    
    dut._log.info("Running randomized LiDAR sweep test vs Golden Model...")
    
    # Generate 200 random dual-lane points
    # To test RAW forwarding and bank conflicts, we intentionally allow some repeated cells
    num_cycles = 200
    stimulus = []
    
    for _ in range(num_cycles):
        v1 = random.choice([0, 1])
        v0 = random.choice([0, 1])
        
        # Keep distance in range [0, 2000]
        d1 = random.randint(0, 2000)
        d0 = random.randint(0, 2000)
        
        # Keep azimuth in range [0, 35900] in centi-degrees
        # Let's keep it close to 0 to increase probability of conflicts/RAW hazards
        az1 = (random.choice([0, 9000, 18000, 27000]) + random.randint(-500, 500)) % 36000
        az0 = (random.choice([0, 9000, 18000, 27000]) + random.randint(-500, 500)) % 36000
        
        l1 = random.randint(0, 15)
        l0 = random.randint(0, 15)
        
        stimulus.append((v1, d1, az1, l1, v0, d0, az0, l0))

    # Extract all addresses that will be touched by the stimulus and clear them beforehand
    addresses_to_clear = set()
    for item in stimulus:
        hits = gold.process_point(*item)
        for addr, inc in hits:
            addresses_to_clear.add(addr)
    await clear_acc_buffer(dut, hps, list(addresses_to_clear))


    # Apply stimulus
    line_idx = 0
    while line_idx < num_cycles:
        await ReadOnly()
        stalled = dut.stall.value
        
        await RisingEdge(dut.clk)
        
        if not stalled:
            v1, d1, az1, l1, v0, d0, az0, l0 = stimulus[line_idx]
            
            # Apply to DUT
            packed_val = pack_ingress_packet(v1, d1, az1, l1, v0, d0, az0, l0)
            dut.data_in_flat.value = packed_val
            
            # Apply to Gold Model
            hits = gold.process_point(v1, d1, az1, l1, v0, d0, az0, l0)
            for addr, inc in hits:
                gold.write_acc(addr, inc)
                
            line_idx += 1
        else:
            # Stalled, keep input constant (simulate correct stall driver behavior)
            pass

    # Clear inputs
    await RisingEdge(dut.clk)
    dut.data_in_flat.value = 0
    
    # Wait for pipeline to drain
    for _ in range(25):
        await RisingEdge(dut.clk)
        
    # Trigger buffer switch
    dut.buffer_switch.value = 1
    await RisingEdge(dut.clk)
    dut.buffer_switch.value = 0
    await RisingEdge(dut.clk)
    
    # Swap Gold Model buffer
    gold.toggle_switch()
    
    # Compare entire grid
    mismatches = 0
    for i in range(10000):
        dut_val = await hps.read(i)
        gold_val = gold.read_hps(i)
        
        if dut_val != gold_val:
            dut._log.error(f"Mismatch at address {i} (X={i%100}, Y={i//100}): DUT={dut_val}, Gold={gold_val}")
            mismatches += 1
            
    assert mismatches == 0, f"LiDAR Sweep verification failed. Found {mismatches} mismatching cell values."
    dut._log.info("Holistic LiDAR sweep comparison against Golden Model passed with zero mismatches!")


@cocotb.test()
async def test_live_stream_lidar_feed(dut):
    """Test Case 7: Verification of real-time continuous streaming of LiDAR data with concurrent HPS read-and-clear."""
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    hps = HPSBridgeDriver(dut, dut.clk)
    await reset_dut(dut)
    
    # Step 1: Clear the specific addresses we will write to
    await clear_acc_buffer(dut, hps, [7050, 7250, 7450, 5070, 5072])
    
    # Pre-calculated stimulus packages representing target hits for 3 frames:
    # Frame 1: Point at X=50, Y=70 (Addr 7050) -> dist=1036, az=0, laser=0
    # Frame 2: Point at X=50, Y=72 (Addr 7250) -> dist=1140, az=0, laser=0
    #            Point at X=70, Y=50 (Addr 5070) -> dist=1036, az=9000, laser=0
    # Frame 3: Point at X=50, Y=74 (Addr 7450) -> dist=1243, az=0, laser=0
    #            Point at X=72, Y=50 (Addr 5072) -> dist=1140, az=9000, laser=0
    
    frame_stimulus = {
        1: [
            pack_ingress_packet(v1=0, d1=0, az1=0, l1=0, v0=1, d0=1036, az0=0, l0=0)
        ],
        2: [
            pack_ingress_packet(v1=1, d1=1036, az1=9000, l1=0, v0=1, d0=1140, az0=0, l0=0)
        ],
        3: [
            pack_ingress_packet(v1=1, d1=1140, az1=9000, l1=0, v0=1, d0=1243, az0=0, l0=0)
        ]
    }
    
    # Frame 1 Execution
    dut._log.info("--- Live Stream: Frame 1 ---")
    dut.data_in_flat.value = frame_stimulus[1][0]
    await RisingEdge(dut.clk)
    dut.data_in_flat.value = 0
    for _ in range(15): await RisingEdge(dut.clk) # Drain
    
    # Swap frames (Switch to Frame 2)
    dut.buffer_switch.value = 1
    await RisingEdge(dut.clk)
    dut.buffer_switch.value = 0
    await RisingEdge(dut.clk)
    
    # Frame 2 Execution (Concurrently Accelerator runs Frame 2, HPS reads and clears Frame 1 data)
    dut._log.info("--- Live Stream: Frame 2 (Concurrently processing Frame 2 and HPS clearing Frame 1) ---")
    
    # Start HPS read and clear task (asynchronous coroutine)
    async def hps_read_and_clear_f1():
        # Verify Frame 1 data at address 7050
        val = await hps.read(7050)
        assert val == 20, f"Frame 1: Expected hit at 7050, got {val}"
        dut._log.info("Frame 1: Verified hit at 7050.")
        # Clear Frame 1 data
        await hps.write(7050, 0)
        # Verify it's cleared
        val_c = await hps.read(7050)
        assert val_c == 0, f"Frame 1: Failed to clear 7050, got {val_c}"
        dut._log.info("Frame 1: Successfully cleared 7050.")
        
    hps_task = cocotb.start_soon(hps_read_and_clear_f1())
    
    # Drive Frame 2 inputs to the Accelerator
    dut.data_in_flat.value = frame_stimulus[2][0]
    await RisingEdge(dut.clk)
    dut.data_in_flat.value = 0
    for _ in range(15): await RisingEdge(dut.clk) # Drain
    
    # Wait for HPS tasks to finish
    await hps_task
    
    # Swap frames (Switch to Frame 3)
    dut.buffer_switch.value = 1
    await RisingEdge(dut.clk)
    dut.buffer_switch.value = 0
    await RisingEdge(dut.clk)
    
    # Frame 3 Execution (Concurrently Accelerator runs Frame 3, HPS reads and clears Frame 2 data)
    dut._log.info("--- Live Stream: Frame 3 (Concurrently processing Frame 3 and HPS clearing Frame 2) ---")
    
    async def hps_read_and_clear_f2():
        # Verify Frame 2 data at 7250 and 5070
        val_a = await hps.read(7250)
        val_b = await hps.read(5070)
        assert val_a == 20, f"Frame 2: Expected hit at 7250, got {val_a}"
        assert val_b == 20, f"Frame 2: Expected hit at 5070, got {val_b}"
        dut._log.info("Frame 2: Verified hits at 7250 and 5070.")
        # Clear Frame 2 data
        await hps.write(7250, 0)
        await hps.write(5070, 0)
        
    hps_task = cocotb.start_soon(hps_read_and_clear_f2())
    
    # Drive Frame 3 inputs to the Accelerator
    dut.data_in_flat.value = frame_stimulus[3][0]
    await RisingEdge(dut.clk)
    dut.data_in_flat.value = 0
    for _ in range(15): await RisingEdge(dut.clk) # Drain
    
    await hps_task
    
    # Swap frames (Switch HPS to Frame 3)
    dut.buffer_switch.value = 1
    await RisingEdge(dut.clk)
    dut.buffer_switch.value = 0
    await RisingEdge(dut.clk)
    
    # HPS verification of Frame 3
    dut._log.info("--- Live Stream: Verifying Frame 3 ---")
    val_a = await hps.read(7450)
    val_b = await hps.read(5072)
    assert val_a == 20, f"Frame 3: Expected hit at 7450, got {val_a}"
    assert val_b == 20, f"Frame 3: Expected hit at 5072, got {val_b}"
    dut._log.info("Frame 3: Verified hits at 7450 and 5072.")
    
    # Clear Frame 3
    await hps.write(7450, 0)
    await hps.write(5072, 0)
    
    # Swap frames back to double-check that Frame 1 buffer was indeed kept clean
    # (since HPS cleared it during Frame 2)
    dut.buffer_switch.value = 1
    await RisingEdge(dut.clk)
    dut.buffer_switch.value = 0
    await RisingEdge(dut.clk)
    
    dut._log.info("--- Live Stream: Double checking Frame 1 buffer is clean ---")
    val_check = await hps.read(7050)
    assert val_check == 0, f"Double check: Frame 1 buffer was not kept clean, got {val_check}"
    
    dut._log.info("Live stream LiDAR verification passed with zero lag and clean memory clearance!")

