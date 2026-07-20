`timescale 1ns/1ps
`include "sys_defs.svh"

module tb_accelerator;

    // --- Clocks & Resets ---
    logic clk;
    logic rst_n;
    
    always #5 clk = ~clk; // 100MHz Clock

    // --- Accelerator Ports ---
    INGRESS_PACKET data_in;
    logic switch;
    
    // --- HPS Ports ---
    logic                        hps_we;
    logic [$clog2(`MEM_DEPTH):0] hps_write_addr;
    OCC_ENTRY                    hps_data_in;
    logic [$clog2(`MEM_DEPTH):0] hps_read_addr;
    OCC_ENTRY                    hps_data_out;

    // --- Top Level Instantiation ---
    accelerator dut (
        .clk(clk),
        .rst_n(rst_n),
        .data_in_flat(data_in),
        .buffer_switch(switch),
        
        .hps_we(hps_we),
        .hps_write_addr(hps_write_addr),
        .hps_data_in(hps_data_in),
        .hps_read_addr(hps_read_addr),
        .hps_data_out(hps_data_out)
    );

    // --- File IO Variables ---
    int fd_in, fd_out, scan_result;
    logic [31:0] v1, d1, a1, l1, v0, d0, a0, l0;

    initial begin
        // 1. Initialize
        $dumpfile("waveform.vcd");
        $dumpvars(0, tb_accelerator);
        
        clk = 0;
        rst_n = 0;
        switch = 0;
        data_in = '0;
        hps_we = 0;
        hps_write_addr = '0;
        hps_data_in = '0;
        hps_read_addr = '0;

        // Open files
        fd_in = $fopen("stimulus.txt", "r");
        if (fd_in == 0) begin
            $display("ERROR: Could not open stimulus.txt");
            $finish;
        end
        
        fd_out = $fopen("output_map.txt", "w");

        // 2. Reset Sequence
        #20;
        rst_n = 1;
        #10;

        // 3. Inject Data
        $display("Starting data injection...");
        while (!$feof(fd_in)) begin
            // ONLY read the next line if the pipeline is NOT stalling
            if (!dut.stall) begin
                scan_result = $fscanf(fd_in, "%d %d %d %d %d %d %d %d\n", v1, d1, a1, l1, v0, d0, a0, l0);
                
                data_in.valid[1]    = v1;
                data_in.distance[1] = d1;
                data_in.azimuth[1]  = a1;
                data_in.laser_id[1] = l1;
                
                data_in.valid[0]    = v0;
                data_in.distance[0] = d0;
                data_in.azimuth[0]  = a0;
                data_in.laser_id[0] = l0;
            end
            
            // Wait 1 clock cycle
            @(posedge clk);
        end
        
        // Zero out the input to prevent stale data processing
        data_in = '0;

        // 4. Wait for Pipeline to Drain
        // Pipeline is ~5 cycles deep. Wait 20 just to be absolutely safe.
        $display("Sweep complete. Waiting for pipeline to drain...");
        repeat(20) @(posedge clk);

        // 5. Switch Ping-Pong Buffers
        $display("Triggering frame switch!");
        switch = 1;
        @(posedge clk);
        switch = 0;
        @(posedge clk);

        // 6. Read out the HPS Data
        $display("Reading HPS memory map...");
        for (int i = 0; i < 10000; i++) begin
            hps_read_addr = i;
            
            // Wait 1 clock cycle for memory to output the data
            @(posedge clk);
            
            // Because the read has 1 cycle latency, data_out is valid on the NEXT edge.
            // But to keep the loop simple, we sample right after the edge.
            // We actually need to evaluate on the negedge, or just wait one more #1.
            #1; 
            if (hps_data_out > 0) begin
                $display("Hit at Addr %0d: Weight %0d", i-1, hps_data_out);
                // Calculate X and Y from 1D Address
                $fwrite(fd_out, "%0d %0d %0d\n", (i-1)%100, (i-1)/100, hps_data_out);
            end
        end

        // 7. Finish
        $display("Simulation complete.");
        $fclose(fd_in);
        $fclose(fd_out);
        $finish;
    end

endmodule
