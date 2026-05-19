`include "sys_defs.svh"
module mem (
    input  logic clk,
    
    // Write Ports
    input  logic we,                                    // Write Enable
    input  logic [$clog2(`MEM_DEPTH)-1:0] write_addr,    // 14-bit Write Address
    input  OCC_ENTRY data_in,                           // Data to write
    
    // Read Ports
    input  logic [$clog2(`MEM_DEPTH)-1:0] read_addr,     // 14-bit Read Address
    output OCC_ENTRY data_out                           // Data read out
);
    // Initialize the entire RAM to Zero natively at declaration
    OCC_ENTRY ram [0:`MEM_DEPTH-1] = '{default: '0};

    // Synchronous Write
    always_ff @(posedge clk) begin
        if (we) begin
            ram[write_addr] <= data_in;
        end
    end

    // Synchronous Read
    always_ff @(posedge clk) begin
        data_out <= ram[read_addr];
    end

endmodule
