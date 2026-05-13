`timescale 1ns/1ps
`include "sys_defs.svh"

module tb_stage_t;

    // --- Signals ---
    logic clk;
    logic rst_n;
    INGRESS_PACKET   data_in;
    TRANSFORM_PACKET t2f_pipe;

    // --- Instantiate the Device Under Test (DUT) ---
    stage_t dut (
        .clk(clk),
        .rst_n(rst_n),
        .data_in(data_in),
        .t2f_pipe(t2f_pipe)
    );

    // --- Clock Generation (100 MHz) ---
    initial clk = 0;
    // Using '<=' instead of '=' to satisfy Verilator's strict BLKSEQ linter rule
    always #5 clk <= ~clk; 

    // --- Test Sequence ---
    initial begin
        // 1. Setup Waveform Dumping for GTKWave
        $dumpfile("waveform.vcd");
        $dumpvars(0, tb_stage_t); 

        // 2. Initialize and Reset
        rst_n = 0;
        data_in = '0;
        
        // Wait a few cycles, then release reset
        @(negedge clk);
        @(negedge clk);
        rst_n = 1;

        $display("========================================");
        $display("   LIDAR STAGE_T PIPELINE TESTBENCH");
        $display("========================================");

        // --- CYCLE 1: INJECT VECTOR BATCH A ---
        @(negedge clk);
        data_in.valid       = 2'b11; 
        // Lane 0: "Forward Max" (15 deg Elev, 0 deg Azim, 5000mm)
        data_in.laser_id[0] = 4'd15;   
        data_in.azimuth[0]  = 16'd0;      
        data_in.distance[0] = 16'd5000; 
        
        // Lane 1: "Right Flank" (7 deg Elev, 270 deg Azim, 2500mm)
        data_in.laser_id[1] = 4'd7;    
        data_in.azimuth[1]  = 16'd27000;  
        data_in.distance[1] = 16'd2500;
        $display("[Cycle 1] Injected Vectors A (Forward Max & Right Flank)");


        // --- CYCLE 2: INJECT VECTOR BATCH B ---
        // As Batch B enters, Batch A is moving to the 2nd pipeline stage
        @(negedge clk);
        data_in.valid       = 2'b11; 
        // Lane 0: "Rear Left Diagonal" (-15 deg Elev, 135 deg Azim, 1000mm)
        data_in.laser_id[0] = 4'd0;    
        data_in.azimuth[0]  = 16'd13500;  
        data_in.distance[0] = 16'd1000; 
        
        // Lane 1: "Ghost Point" (-7 deg Elev, 45 deg Azim, 0mm)
        data_in.laser_id[1] = 4'd8;    
        data_in.azimuth[1]  = 16'd4500;   
        data_in.distance[1] = 16'd0;
        $display("[Cycle 2] Injected Vectors B (Rear Left & Ghost Point)");


        // --- CYCLE 3: READ OUTPUTS FOR BATCH A ---
        // Batch A has completed its 2-cycle journey and is ready on t2f_pipe
        @(negedge clk);
        data_in.valid = 2'b00; // Stop sending new data
        $display("\n--- OUTPUTS FOR VECTORS A ---");
        $display("Forward Max | X: %5d mm, Y: %5d mm, Z: %5d mm", 
                 t2f_pipe.x[0], t2f_pipe.y[0], t2f_pipe.z[0]);
        $display("Right Flank | X: %5d mm, Y: %5d mm, Z: %5d mm", 
                 t2f_pipe.x[1], t2f_pipe.y[1], t2f_pipe.z[1]);


        // --- CYCLE 4: READ OUTPUTS FOR BATCH B ---
        // Batch B has now completed its 2-cycle journey
        @(negedge clk);
        $display("\n--- OUTPUTS FOR VECTORS B ---");
        $display("Rear Left   | X: %5d mm, Y: %5d mm, Z: %5d mm", 
                 t2f_pipe.x[0], t2f_pipe.y[0], t2f_pipe.z[0]);
        $display("Ghost Point | X: %5d mm, Y: %5d mm, Z: %5d mm", 
                 t2f_pipe.x[1], t2f_pipe.y[1], t2f_pipe.z[1]);

        $display("========================================");

        // Wait one final cycle to let the waveforms flush, then end
        @(negedge clk);
        $finish;
    end

endmodule