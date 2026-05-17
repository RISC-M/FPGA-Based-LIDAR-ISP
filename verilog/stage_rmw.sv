`include "sys_defs.svh"
module stage_rmw (
    input RMW_PACKET f2r_pipe,
    input clk,
    input rst_n,

    // Write
    output logic        [1:0] proc2mem_we,                                    // Write Enable
    output logic        [1:0][$clog2(`MEM_DEPTH)-1:0] proc2mem_wraddr,    // 14-bit Write Address
    output OCC_ENTRY    proc2mem_data [1:0],                              // Data to write
    
    // Read
    output logic        [1:0][$clog2(`MEM_DEPTH)-1:0] proc2mem_rdaddr,     // 14-bit Read Address
    input  OCC_ENTRY    mem2proc_data [1:0],

    // Stall
    output logic stall
);
    // READ STAGE //
      
    // GENERATE MEM ADDRESSES
    logic [$clog2(`MEM_DEPTH)-1:0] mem_addrs [1:0];
    always_comb begin
        for(int i=0; i<2; i++) begin
            // Computes Y * 100 + X efficiently
            mem_addrs[i] = (f2r_pipe.mem_y[i] << 6) + (f2r_pipe.mem_y[i] << 5) + 
                           (f2r_pipe.mem_y[i] << 2) + f2r_pipe.mem_x[i]; 
        end
    end

    // ASSIGN MEM ADDRESSES TO CORRECT Bank
    // if both bits have the correct bank, send it through
    // if both bits do not have the correct bank, swap the entries
    // if one but does not have the correct bank, stall

    logic conflict;
    // Conflict  happens when they are both valid and they have the same bank
    assign conflict = f2r_pipe.valid[0] && f2r_pipe.valid[1] && 
                      (mem_addrs[0][0] == mem_addrs[1][0]);

    logic resolving_stall; // 1 bit counter to count if we are processing a stall
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) 
            resolving_stall <= 1'b0;
        else if (conflict && !resolving_stall) 
            resolving_stall <= 1'b1; // Enter resolution state
        else 
            resolving_stall <= 1'b0; // Exit resolution state
    end

    assign stall = conflict & !resolving_stall; // only send stall signal when its first cycle of conflict

    R2W_PACKET r2w_packet;
    R2W_PACKET next_r2w_packet;

    // ROUTING LOGIC
    always_comb begin
        proc2mem_rdaddr[0] = '0;
        proc2mem_rdaddr[1] = '0;
        next_r2w_packet = '0;
        if (conflict) begin
            if (!resolving_stall) begin
                // Cycle 1 of conflict: only process lane 1
                if (mem_addrs[0][0] == 1'b0) begin 
                    proc2mem_rdaddr[0] = mem_addrs[0][13:1]; // Route to Bank 0
                    next_r2w_packet.addr[0]   = mem_addrs[0][13:1];
                    next_r2w_packet.inc[0]    = f2r_pipe.increment[0];
                    next_r2w_packet.valid[0]  = 1'b1;
                end else begin
                    proc2mem_rdaddr[1] = mem_addrs[0][13:1]; // Route to Bank 1
                    next_r2w_packet.addr[1]   = mem_addrs[0][13:1];
                    next_r2w_packet.inc[1]    = f2r_pipe.increment[0];
                    next_r2w_packet.valid[1]  = 1'b1;
                end
            end else begin
                // Cycle 2 of conflict: only process lane 2
                if (mem_addrs[1][0] == 1'b0) begin 
                    proc2mem_rdaddr[0] = mem_addrs[1][13:1]; // Route to Bank 0
                    next_r2w_packet.addr[0]   = mem_addrs[1][13:1];
                    next_r2w_packet.inc[0]    = f2r_pipe.increment[1];
                    next_r2w_packet.valid[0]  = 1'b1;
                end else begin
                    proc2mem_rdaddr[1] = mem_addrs[1][13:1]; // Route to Bank 1
                    next_r2w_packet.addr[1]   = mem_addrs[1][13:1];
                    next_r2w_packet.inc[1]    = f2r_pipe.increment[1];
                    next_r2w_packet.valid[1]  = 1'b1;
                end
            end
        end else begin
            // Normal operation: Route if valid
            if (f2r_pipe.valid[0]) begin
                if (mem_addrs[0][0] == 1'b0) begin
                    proc2mem_rdaddr[0] = mem_addrs[0][13:1];
                    next_r2w_packet.addr[0]   = mem_addrs[0][13:1];
                    next_r2w_packet.inc[0]    = f2r_pipe.increment[0];
                    next_r2w_packet.valid[0]  = 1'b1;
                end else begin
                    proc2mem_rdaddr[1] = mem_addrs[0][13:1];
                    next_r2w_packet.addr[1]   = mem_addrs[0][13:1];
                    next_r2w_packet.inc[1]    = f2r_pipe.increment[0];
                    next_r2w_packet.valid[1]  = 1'b1;
                end
            end 
            if (f2r_pipe.valid[1]) begin
                if (mem_addrs[1][0] == 1'b0) begin
                    proc2mem_rdaddr[0] = mem_addrs[1][13:1];
                    next_r2w_packet.addr[0]   = mem_addrs[1][13:1];
                    next_r2w_packet.inc[0]    = f2r_pipe.increment[1];
                    next_r2w_packet.valid[0]  = 1'b1;
                end else begin
                    proc2mem_rdaddr[1] = mem_addrs[1][13:1];
                    next_r2w_packet.addr[1]   = mem_addrs[1][13:1];
                    next_r2w_packet.inc[1]    = f2r_pipe.increment[1];
                    next_r2w_packet.valid[1]  = 1'b1;
                end
            end
        end
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if(!rst_n) 
            r2w_packet <= '0;
        else 
            r2w_packet <= next_r2w_packet;
    end

    // MODIFY + WRITE STAGE//

    // Forwarding History Regs
    logic [1:0]       last_we;
    logic [1:0][12:0] last_wraddr; 
    OCC_ENTRY         last_wdata [1:0];

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            last_we     <= '0;
            last_wraddr <= '0;
            last_wdata[0] <= '0; 
            last_wdata[1] <= '0;
        end else begin
            // Remember what we are writing right now for the NEXT cycle
            last_we     <= proc2mem_we;
            last_wraddr <= proc2mem_wraddr;
            last_wdata  <= proc2mem_data;
        end
    end
    
    OCC_ENTRY   fwd_data [1:0];
    logic [8:0] alu_res [1:0]; // 9-bit to safely catch overflow > 255

    always_comb begin
        for(int i=0; i<2; i++) begin
            // Forwarding mux
            // If we wrote to this bank last cycle AND the address we wrote 
            // matches the address we are reading right now:
            if (last_we[i] && (last_wraddr[i] == r2w_packet.addr[i])) begin
                fwd_data[i] = last_wdata[i];     // Forward and use the history register
            end else begin
                fwd_data[i] = mem2proc_data[i];  // Otherwise, use fetched data
            end

            // Result accumulation
            alu_res[i] = fwd_data[i] + r2w_packet.increment[i];
            // Proc2mem assignments
            proc2mem_we[i]     = r2w_packet.valid[i];
            proc2mem_wraddr[i] = r2w_packet.addr[i];
            // Confidence saturates at max value of 99%
            proc2mem_data[i]   = (alu_res[i] > 9'd99) ? 8'd99 : alu_res[i][7:0];
        end
    end
    
endmodule