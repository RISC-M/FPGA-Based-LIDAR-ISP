`include "sys_defs.svh"

module double_buffer #(
    parameter `MEM_DEPTH = 5000
)(
    input logic clk,
    input logic rst_n,
    input logic switch,

    // Accelerator ports
    input  logic                         acc_we,
    input  logic [$clog2(`MEM_DEPTH)-1:0] acc_write_addr,
    input  OCC_ENTRY                     acc_data_in,
    input  logic [$clog2(`MEM_DEPTH)-1:0] acc_read_addr,
    output OCC_ENTRY                     acc_data_out,

    // HPS ports
    input  logic                         hps_we,         // Write 0s, so still need write
    input  logic [$clog2(`MEM_DEPTH)-1:0] hps_write_addr,
    input  OCC_ENTRY                     hps_data_in,
    input  logic [$clog2(`MEM_DEPTH)-1:0] hps_read_addr,
    output OCC_ENTRY                     hps_data_out
);

    // Internal wires for the two mem modules
    logic                         we         [1:0];
    logic [$clog2(`MEM_DEPTH)-1:0] write_addr [1:0];
    OCC_ENTRY                     data_in    [1:0];
    logic [$clog2(`MEM_DEPTH)-1:0] read_addr  [1:0];
    OCC_ENTRY                     data_out   [1:0];

    mem #(.`MEM_DEPTH(`MEM_DEPTH)) mem0 (
        .clk(clk),
        .we(we[0]), .write_addr(write_addr[0]), .data_in(data_in[0]),
        .read_addr(read_addr[0]), .data_out(data_out[0])
    );

    mem #(.`MEM_DEPTH(`MEM_DEPTH)) mem1 (
        .clk(clk),
        .we(we[1]), .write_addr(write_addr[1]), .data_in(data_in[1]),
        .read_addr(read_addr[1]), .data_out(data_out[1])
    );

    // 2 state ping pong
    logic ctr;
    always_ff @(posedge clk or negedge rst_n) begin
        if(!rst_n)
            ctr <= 1'b0;
        else if (switch)
            ctr <= ~ctr;
    end

    always_comb begin
        if (ctr == 1'b0) begin
            // CTR = 0: Accelerator gets Mem0, HPS gets Mem1
            we[0]         = acc_we;
            write_addr[0] = acc_write_addr;
            data_in[0]    = acc_data_in;
            read_addr[0]  = acc_read_addr;
            acc_data_out  = data_out[0];

            we[1]         = hps_we;
            write_addr[1] = hps_write_addr;
            data_in[1]    = hps_data_in;
            read_addr[1]  = hps_read_addr;
            hps_data_out  = data_out[1];
        end else begin
            // CTR = 1: Accelerator gets Mem1, HPS gets Mem0
            we[1]         = acc_we;
            write_addr[1] = acc_write_addr;
            data_in[1]    = acc_data_in;
            read_addr[1]  = acc_read_addr;
            acc_data_out  = data_out[1];

            we[0]         = hps_we;
            write_addr[0] = hps_write_addr;
            data_in[0]    = hps_data_in;
            read_addr[0]  = hps_read_addr;
            hps_data_out  = data_out[0];
        end
    end

endmodule