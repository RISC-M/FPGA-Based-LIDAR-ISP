`timescale 1ns/1ps
`include "sys_defs.svh"

module tb_pipeline;

    // --- Signals ---
    logic clk;
    logic rst_n;
    INGRESS_PACKET   data_in;
    TRANSFORM_PACKET t2f_pipe;
    RMW_PACKET       f2r_pipe;

    // --- File I/O Handles ---
    integer fd_in;
    integer fd_out;
    integer scan_result;

    // Variables to hold raw hex reads
    logic [3:0]  raw_valid;
    logic [15:0] raw_d0, raw_a0, raw_d1, raw_a1;
    logic [3:0]  raw_l0, raw_l1;

    // --- Instantiate Pipeline Stages ---
    stage_t dut_transform (
        .clk(clk),
        .rst_n(rst_n),
        .data_in(data_in),
        .t2f_pipe(t2f_pipe)
    );

    stage_f #(
        .GND_THRESH(-16'sd1000), // Lower the threshold so -517mm passes
        .HIT_WEIGHT(8'd20)
    ) dut_filter_quantize (
        .clk(clk),
        .rst_n(rst_n),
        .t2f_pipe(t2f_pipe),
        .f2r_pipe(f2r_pipe)
    );

    // --- Clock Generation ---
    initial clk = 0;
    always #5 clk <= ~clk; 

    // --- Main Verification Sequence ---
    initial begin
        $dumpfile("waveform.vcd");
        $dumpvars(0, tb_pipeline);

        // Open Files
        fd_in  = $fopen("test/stimulus.txt", "r");
        fd_out = $fopen("output.txt", "w");
        
        if (fd_in == 0) begin
            $display("ERROR: Could not open stimulus.txt. Did you run the python script?");
            $finish;
        end

        // Initialize
        rst_n = 0;
        data_in = '0;
        
        @(negedge clk);
        @(negedge clk);
        rst_n = 1;
        $display("Pipeline Reset Complete. Starting File Stream...");

        // Read file line by line
        while (!$feof(fd_in)) begin
            @(negedge clk);
            
            scan_result = $fscanf(fd_in, "%h %h %h %h %h %h %h\n", 
                                  raw_valid, raw_d0, raw_a0, raw_l0, raw_d1, raw_a1, raw_l1);
            
            if (scan_result == 7) begin
                data_in.valid       = raw_valid[1:0];
                data_in.distance[0] = raw_d0;
                data_in.azimuth[0]  = raw_a0;
                data_in.laser_id[0] = raw_l0;
                data_in.distance[1] = raw_d1;
                data_in.azimuth[1]  = raw_a1;
                data_in.laser_id[1] = raw_l1;
            end else begin
                data_in.valid = 2'b00;
            end
        end

        // Wait for pipeline to flush out the last data (3 cycles)
        @(negedge clk); data_in.valid = 2'b00;
        @(negedge clk);
        @(negedge clk);
        @(negedge clk);

        $display("File Stream Complete. Check output.txt");
        $fclose(fd_in);
        $fclose(fd_out);
        $finish;
    end

    // --- Output Monitor ---
    // Watches the end of the pipeline and logs valid memory hits
    // --- Output Monitor ---
    always @(negedge clk) begin
        if (rst_n === 1'b1) begin
            for (int i = 0; i < 2; i++) begin
                // DEBUG: Print every point to the terminal, even if invalid
                if (data_in.valid[i]) begin
                    $display("Cycle Check: L%0d | Valid_In: %b | Z_Coord: %d | Mem_Valid: %b", 
                             i, data_in.valid[i], t2f_pipe.z[i], f2r_pipe.valid[i]);
                end

                if (f2r_pipe.valid[i]) begin
                    $fdisplay(fd_out, "Grid: [%3d, %3d] | Weight: %2d", 
                              f2r_pipe.mem_x[i], f2r_pipe.mem_y[i], f2r_pipe.increment[i]);
                end
            end
        end
    end

endmodule

