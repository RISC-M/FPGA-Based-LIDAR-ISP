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

    always_comb begin
        next_f2r_pipe = '0; 
        for(int i = 0; i < 2; i++) begin
            
            // Divide by 50 hardware friendly
            logic signed [31:0] scaled_x = (signed'(32'(t2f_pipe.x[i])) * 32'sd1311) >>> 16;
            logic signed [31:0] scaled_y = (signed'(32'(t2f_pipe.y[i])) * 32'sd1311) >>> 16;

            // Shift Origin to 50,50
            logic signed [15:0] mem_x_signed = 16'(scaled_x) + 16'sd50;
            logic signed [15:0] mem_y_signed = 16'(scaled_y) + 16'sd50;

            // Boundary checking (filter out points outside of occupancy range or a ground point)
            if (t2f_pipe.valid[i] && 
                t2f_pipe.z[i] >= GND_THRESH && 
                mem_x_signed >= 0 && mem_x_signed < 100 && 
                mem_y_signed >= 0 && mem_y_signed < 100) begin
                
                next_f2r_pipe.valid[i]     = 1'b1;
                next_f2r_pipe.mem_x[i]     = MEM_X'(mem_x_signed);
                next_f2r_pipe.mem_y[i]     = MEM_Y'(mem_y_signed);
                next_f2r_pipe.increment[i] = HIT_WEIGHT;
                
            end else begin
                next_f2r_pipe.valid[i]     = 1'b0;
                next_f2r_pipe.increment[i] = 8'd0;
            end
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
