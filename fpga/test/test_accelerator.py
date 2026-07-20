import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ReadOnly
import os

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

@cocotb.test()
async def test_accelerator(dut):
    """Test the LIDAR accelerator IP core using cocotb and Verilator."""

    # 1. Start Clock (100 MHz, 10ns period)
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())

    # 2. Reset Sequence
    dut.rst_n.value = 0
    dut.buffer_switch.value = 0
    dut.data_in_flat.value = 0
    dut.hps_we.value = 0
    dut.hps_write_addr.value = 0
    dut.hps_data_in_flat.value = 0
    dut.hps_read_addr.value = 0
    
    # Hold reset for 3 clock cycles
    for _ in range(3):
        await RisingEdge(dut.clk)
    dut.rst_n.value = 1
    await RisingEdge(dut.clk)

    # 3. Read stimulus file
    stimulus_path = os.path.join(os.path.dirname(__file__), "stimulus.txt")
    if not os.path.exists(stimulus_path):
        dut._log.error(f"Stimulus file not found at: {stimulus_path}")
        raise FileNotFoundError(f"Missing {stimulus_path}")
        
    dut._log.info("Loading stimulus.txt and starting data injection...")
    
    with open(stimulus_path, "r") as f:
        lines = f.readlines()
        
    line_idx = 0
    while line_idx < len(lines):
        # Sample 'stall' at the beginning of the clock cycle (using ReadOnly or simple query)
        await ReadOnly()
        stalled = dut.stall.value
        
        # We drive inputs on the rising edge
        await RisingEdge(dut.clk)
        
        if not stalled:
            line = lines[line_idx].strip()
            if not line:
                line_idx += 1
                continue
            parts = [int(x) for x in line.split()]
            if len(parts) == 8:
                v1, d1, az1, l1, v0, d0, az0, l0 = parts
                packed_val = pack_ingress_packet(v1, d1, az1, l1, v0, d0, az0, l0)
                dut.data_in_flat.value = packed_val
                line_idx += 1
        else:
            # If stalled, we keep the previous data_in_flat value and wait
            dut._log.info(f"Pipeline stalled at stimulus line {line_idx}. Waiting...")
            
    # Zero out the input after completion
    await RisingEdge(dut.clk)
    dut.data_in_flat.value = 0

    # 4. Wait for Pipeline to Drain (20 clock cycles)
    dut._log.info("Sweep complete. Waiting for pipeline to drain...")
    for _ in range(20):
        await RisingEdge(dut.clk)

    # 5. Switch Ping-Pong Buffers
    dut._log.info("Triggering frame switch!")
    dut.buffer_switch.value = 1
    await RisingEdge(dut.clk)
    dut.buffer_switch.value = 0
    await RisingEdge(dut.clk)

    # 6. Read back the Grid Map via HPS Bridge interface
    dut._log.info("Reading HPS memory map over the virtual Quartus bridge...")
    output_path = os.path.join(os.path.dirname(__file__), "output_map_cocotb.txt")
    
    with open(output_path, "w") as out_f:
        for i in range(10000):
            dut.hps_read_addr.value = i
            
            # Wait 1 clock cycle for synchronous memory read latency
            await RisingEdge(dut.clk)
            
            # Read back data
            val = int(dut.hps_data_out_flat.value)
            if val > 0:
                # Addr i-1 is the address corresponding to the output due to 1-cycle latency
                addr_latched = i - 1
                if addr_latched >= 0:
                    x = addr_latched % 100
                    y = addr_latched // 100
                    out_f.write(f"{x} {y} {val}\n")
                    dut._log.info(f"Hit at Addr {addr_latched} (X={x}, Y={y}): Weight {val}")
                    
    dut._log.info("Simulation complete. Results written to output_map_cocotb.txt")
