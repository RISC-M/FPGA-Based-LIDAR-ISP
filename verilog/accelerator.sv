`include "sys_defs.svh"

module accelerator (
    input clk,
    input rst_n,

    input  logic        stream_we,
    input  logic [1:0]  stream_addr,  // 2-bit address (0, 1, 2)
    input  logic [31:0] stream_wdata, // 32-bit writes from C code

    // HPS Processor Ports (Continuous 14-bit Address Space)
    input  logic                            hps_we,         
    input  logic [$clog2(`MEM_DEPTH):0]     hps_write_addr, // 14-bit (10,000 cells)
    input  OCC_ENTRY                        hps_data_in_flat,  
    input  logic [$clog2(`MEM_DEPTH):0]     hps_read_addr,  // 14-bit (10,000 cells)
    output OCC_ENTRY                        hps_data_out_flat  
);

    // ====================================================
    // DESERIALIZER: Software 32-bit -> Hardware 74-bit
    // ====================================================
    logic [31:0] stream_reg0;
    logic [31:0] stream_reg1;
    logic [9:0]  stream_reg2;
    logic        stream_valid;
    logic        switch;

    TRANSFORM_PACKET t2f_pipe;
    RMW_PACKET       f2r_pipe;
    INGRESS_PACKET   hw_stream_packet;

    logic [31:0] debug_last_wdata;
    logic [1:0]  debug_last_addr;
    logic        debug_we_seen;

    logic signed [15:0] debug_t2f_x;
    logic signed [15:0] debug_t2f_y;
    logic signed [15:0] debug_t2f_z;
    logic [1:0]         debug_t2f_valid;
    logic [7:0]         debug_f2r_x;
    logic [7:0]         debug_f2r_y;
    logic               debug_f2r_valid;

    logic signed [15:0] debug_x_coeff;
    logic signed [15:0] debug_y_coeff;
    logic signed [15:0] debug_z_coeff;
    logic signed [15:0] debug_distance;
    logic [8:0]         debug_az_degree;

    DATA [1:0] t_debug_x_coeff;
    DATA [1:0] t_debug_y_coeff;
    DATA [1:0] t_debug_z_coeff;
    DATA [1:0] t_debug_distance_q;
    logic [1:0][8:0] t_debug_az_degree;

    logic [1:0] data_in_valid_q;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            debug_t2f_x     <= 16'h0;
            debug_t2f_y     <= 16'h0;
            debug_t2f_z     <= 16'h0;
            debug_t2f_valid <= 2'b0;
            debug_f2r_x     <= 8'h0;
            debug_f2r_y     <= 8'h0;
            debug_f2r_valid <= 1'b0;

            debug_x_coeff   <= 16'h0;
            debug_y_coeff   <= 16'h0;
            debug_z_coeff   <= 16'h0;
            debug_distance  <= 16'h0;
            debug_az_degree <= 9'h0;
            data_in_valid_q <= 2'b0;
        end else begin
            if (t2f_pipe.valid != 2'b00) begin
                debug_t2f_x     <= t2f_pipe.x[1];
                debug_t2f_y     <= t2f_pipe.y[1];
                debug_t2f_z     <= t2f_pipe.z[1];
                debug_t2f_valid <= t2f_pipe.valid;
            end
            if (f2r_pipe.valid != 2'b00) begin
                debug_f2r_x     <= f2r_pipe.mem_x[1];
                debug_f2r_y     <= f2r_pipe.mem_y[1];
                debug_f2r_valid <= f2r_pipe.valid[1];
            end

            data_in_valid_q <= hw_stream_packet.valid;
            if (data_in_valid_q != 2'b00) begin
                debug_x_coeff   <= t_debug_x_coeff[1];
                debug_y_coeff   <= t_debug_y_coeff[1];
                debug_z_coeff   <= t_debug_z_coeff[1];
                debug_distance  <= t_debug_distance_q[1];
                debug_az_degree <= t_debug_az_degree[1];
            end
        end
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            stream_reg0  <= '0;
            stream_reg1  <= '0;
            stream_reg2  <= '0;
            stream_valid <= 1'b0;
            switch       <= 1'b0;
            debug_last_wdata <= 32'h0;
            debug_last_addr  <= 2'h0;
            debug_we_seen    <= 1'b0;
        end else begin
            stream_valid <= 1'b0; // Default to 0, pulses high on completion
            if (stream_we) begin
                stream_reg0  <= (stream_addr == 2'd0) ? stream_wdata : stream_reg0;
                stream_reg1  <= (stream_addr == 2'd1) ? stream_wdata : stream_reg1;
                stream_reg2  <= (stream_addr == 2'd2) ? stream_wdata[9:0] : stream_reg2;
                stream_valid <= (stream_addr == 2'd2);
                switch       <= (stream_addr == 2'd3) ? stream_wdata[0] : switch;

                debug_last_wdata <= stream_wdata;
                debug_last_addr  <= stream_addr;
                debug_we_seen    <= 1'b1;
            end
        end
    end

    always_comb begin
        hw_stream_packet.laser_id[0] = ELEVATION'(stream_reg0[3:0]);
        hw_stream_packet.laser_id[1] = ELEVATION'(stream_reg0[7:4]);
        
        hw_stream_packet.azimuth[0]  = DATA'(stream_reg0[23:8]);
        hw_stream_packet.azimuth[1]  = DATA'({stream_reg1[7:0], stream_reg0[31:24]});
        
        hw_stream_packet.distance[0] = DATA'(stream_reg1[23:8]);
        hw_stream_packet.distance[1] = DATA'({stream_reg2[7:0], stream_reg1[31:24]});
        
        // Only assert valid bit into the pipeline if the 3rd write just occurred
        hw_stream_packet.valid       = stream_valid ? stream_reg2[9:8] : 2'b00;
    end

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
        .data_in(hw_stream_packet), .t2f_pipe(t2f_pipe),
        .debug_x_coeff(t_debug_x_coeff),
        .debug_y_coeff(t_debug_y_coeff),
        .debug_z_coeff(t_debug_z_coeff),
        .debug_distance_q(t_debug_distance_q),
        .debug_az_degree(t_debug_az_degree)
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

        // Mux HPS Reads based on LSB and address range
        if (hps_read_addr >= 14'd9000) begin
            case (hps_read_addr - 14'd9000)
                14'd0: hps_data_out_flat = debug_t2f_x[7:0];
                14'd1: hps_data_out_flat = debug_t2f_x[15:8];
                14'd2: hps_data_out_flat = debug_t2f_y[7:0];
                14'd3: hps_data_out_flat = debug_t2f_y[15:8];
                14'd4: hps_data_out_flat = debug_t2f_z[7:0];
                14'd5: hps_data_out_flat = debug_t2f_z[15:8];
                14'd6: hps_data_out_flat = debug_f2r_x;
                14'd7: hps_data_out_flat = debug_f2r_y;
                14'd8: hps_data_out_flat = debug_y_coeff[7:0];
                14'd9: hps_data_out_flat = debug_y_coeff[15:8];
                14'd10: hps_data_out_flat = debug_z_coeff[7:0];
                14'd11: hps_data_out_flat = debug_z_coeff[15:8];
                14'd12: hps_data_out_flat = debug_distance[7:0];
                14'd13: hps_data_out_flat = debug_distance[15:8];
                14'd14: hps_data_out_flat = debug_az_degree[7:0];
                14'd15: hps_data_out_flat = {7'b0, debug_az_degree[8]};
                14'd16: hps_data_out_flat = {7'b0, debug_f2r_valid};
                14'd17: hps_data_out_flat = {6'b0, debug_t2f_valid};
                default: hps_data_out_flat = 8'h00;
            endcase
        end else begin
            if (hps_read_addr[0] == 1'b0)
                hps_data_out_flat = hps_data_out_b0;
            else
                hps_data_out_flat = hps_data_out_b1;
        end
    end

    // Memory Instantiations
    // BANK 0 (Even Addresses)
    double_buffer occ_b0 (
        .clk(clk), .rst_n(rst_n), .switch(switch),
        
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
        .clk(clk), .rst_n(rst_n), .switch(switch),
        
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
