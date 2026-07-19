`include "sys_defs.svh"

module stage_f #(
    parameter signed [15:0] GND_THRESH = 16'sd10,
    parameter logic  [7:0]  HIT_WEIGHT = 8'd20
) (
    input  logic clk,
    input  logic rst_n,
    input  TRANSFORM_PACKET t2f_pipe,
    input  logic stall, // backpressure stall from RAW hazards
    output RMW_PACKET f2r_pipe
);
    RMW_PACKET next_f2r_pipe;

    // Intermediate math arrays declared outside the procedural loop scope
    logic signed [31:0] math_x0, math_y0;
    logic signed [31:0] math_x1, math_y1;
    logic signed [15:0] mem_x_signed0, mem_y_signed0;
    logic signed [15:0] mem_x_signed1, mem_y_signed1;

    always_comb begin
        next_f2r_pipe = '0;

        // Lane 0 math
        math_x0 = t2f_pipe.x[0] * 32'sd1311;
        math_y0 = t2f_pipe.y[0] * 32'sd1311;
        mem_x_signed0 = 16'(math_x0 >>> 16) + 16'sd50;
        mem_y_signed0 = 16'(math_y0 >>> 16) + 16'sd50;

        if (t2f_pipe.valid[0] && 
            t2f_pipe.z[0] >= GND_THRESH && 
            mem_x_signed0 >= 0 && mem_x_signed0 < 100 && 
            mem_y_signed0 >= 0 && mem_y_signed0 < 100) begin
            
            next_f2r_pipe.valid[0]     = 1'b1;
            next_f2r_pipe.mem_x[0]     = MEM_X'(mem_x_signed0);
            next_f2r_pipe.mem_y[0]     = MEM_Y'(mem_y_signed0);
            next_f2r_pipe.increment[0] = HIT_WEIGHT;
        end

        // Lane 1 math
        math_x1 = t2f_pipe.x[1] * 32'sd1311;
        math_y1 = t2f_pipe.y[1] * 32'sd1311;
        mem_x_signed1 = 16'(math_x1 >>> 16) + 16'sd50;
        mem_y_signed1 = 16'(math_y1 >>> 16) + 16'sd50;

        if (t2f_pipe.valid[1] && 
            t2f_pipe.z[1] >= GND_THRESH && 
            mem_x_signed1 >= 0 && mem_x_signed1 < 100 && 
            mem_y_signed1 >= 0 && mem_y_signed1 < 100) begin
            
            next_f2r_pipe.valid[1]     = 1'b1;
            next_f2r_pipe.mem_x[1]     = MEM_X'(mem_x_signed1);
            next_f2r_pipe.mem_y[1]     = MEM_Y'(mem_y_signed1);
            next_f2r_pipe.increment[1] = HIT_WEIGHT;
        end

        // REDUCTION TREE
        // If both points are valid and land in the exact same memory cell
        if (next_f2r_pipe.valid[0] && next_f2r_pipe.valid[1] &&
           (next_f2r_pipe.mem_x[0] == next_f2r_pipe.mem_x[1]) &&
           (next_f2r_pipe.mem_y[0] == next_f2r_pipe.mem_y[1])) begin
            
            // Consolidate both hits into Lane 0
            next_f2r_pipe.increment[0] = HIT_WEIGHT << 1; // Double the hit rate
            
            // Kill Lane 1 so the memory controller ignores it
            next_f2r_pipe.valid[1]     = 1'b0;
            next_f2r_pipe.increment[1] = 8'd0;
        end
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            f2r_pipe <= '0;
        end else if (!stall) begin
            f2r_pipe <= next_f2r_pipe;
        end
    end

endmodule
