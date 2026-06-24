`include "sys_defs.svh"

module accelerator (
    input clk,
    input rst_n,
    input [73:0] data_in_flat, 
    input buffer_switch,

    // HPS Processor Ports (Continuous 14-bit Address Space)
    input  logic                            hps_we,         
    input  logic [$clog2(`MEM_DEPTH):0]     hps_write_addr, // 14-bit (10,000 cells)
    input  OCC_ENTRY                        hps_data_in_flat,  
    input  logic [$clog2(`MEM_DEPTH):0]     hps_read_addr,  // 14-bit (10,000 cells)
    output OCC_ENTRY                        hps_data_out_flat  
);

    // Pipeline wires
    INGRESS_PACKET   data_in;
    assign data_in = INGRESS_PACKET'(data_in_flat);

    TRANSFORM_PACKET t2f_pipe;
    RMW_PACKET       f2r_pipe;
    logic            stall;

    // Memory wires
    logic        [1:0]                         proc2mem_we;
    logic        [1:0][$clog2(`MEM_DEPTH)-1:0] proc2mem_wraddr; // 13-bit Bank Address
    OCC_ENTRY    proc2mem_data [1:0];
    logic        [1:0][$clog2(`MEM_DEPTH)-1:0] proc2mem_rdaddr; // 13-bit Bank Address
    OCC_ENTRY    mem2proc_data [1:0];

    // Pipeline instantiations
    stage_t t (
        .clk(clk), .rst_n(rst_n), .stall(stall),
        .data_in(data_in), .t2f_pipe(t2f_pipe)
    );

    stage_f #(.GND_THRESH(-16'sd2000)) f (
        .clk(clk), .rst_n(rst_n), .stall(stall),
        .t2f_pipe(t2f_pipe), .f2r_pipe(f2r_pipe)
    );

    stage_rmw rmw (
        .clk(clk), .rst_n(rst_n), .stall(stall),
        .f2r_pipe(f2r_pipe), // Fixed: Added the missing dot here!
        .proc2mem_we(proc2mem_we),
        .proc2mem_wraddr(proc2mem_wraddr),
        .proc2mem_data(proc2mem_data),
        .proc2mem_rdaddr(proc2mem_rdaddr),
        .mem2proc_data(mem2proc_data)
    );

    // Memory bank routing
    logic hps_we_b0, hps_we_b1;
    OCC_ENTRY hps_data_out_b0, hps_data_out_b1;

    always_comb begin
        // Route HPS Writes based on LSB
        hps_we_b0 = hps_we & ~hps_write_addr[0];
        hps_we_b1 = hps_we &  hps_write_addr[0];

        // Mux HPS Reads based on LSB
        if (hps_read_addr[0] == 1'b0)
            hps_data_out_flat = hps_data_out_b0;
        else
            hps_data_out_flat = hps_data_out_b1;
    end

    // Memory Instantiations
    // BANK 0 (Even Addresses)
    double_buffer occ_b0 (
        .clk(clk), .rst_n(rst_n), .buffer_switch(buffer_switch),
        
        .acc_we(proc2mem_we[0]),
        .acc_write_addr(proc2mem_wraddr[0]),
        .acc_data_in(proc2mem_data[0]),
        .acc_read_addr(proc2mem_rdaddr[0]),
        .acc_data_out(mem2proc_data[0]),

        .hps_we(hps_we_b0),
        .hps_write_addr(hps_write_addr[$clog2(`MEM_DEPTH):1]), // Shift away LSB
        .hps_data_in(hps_data_in_flat),
        .hps_read_addr(hps_read_addr[$clog2(`MEM_DEPTH):1]),   // Shift away LSB
        .hps_data_out(hps_data_out_b0)
    );

    // BANK 1 (Odd Addresses)
    double_buffer occ_b1 (
        .clk(clk), .rst_n(rst_n), .buffer_switch(buffer_switch),
        
        .acc_we(proc2mem_we[1]),
        .acc_write_addr(proc2mem_wraddr[1]),
        .acc_data_in(proc2mem_data[1]),
        .acc_read_addr(proc2mem_rdaddr[1]),
        .acc_data_out(mem2proc_data[1]),

        .hps_we(hps_we_b1),
        .hps_write_addr(hps_write_addr[$clog2(`MEM_DEPTH):1]),
        .hps_data_in(hps_data_in_flat),
        .hps_read_addr(hps_read_addr[$clog2(`MEM_DEPTH):1]),
        .hps_data_out(hps_data_out_b1)
    );

endmodule
