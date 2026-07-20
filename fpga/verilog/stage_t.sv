`include "sys_defs.svh"
module stage_t (
    input clk,
    input rst_n,
    input stall,
    input INGRESS_PACKET data_in,
    output TRANSFORM_PACKET t2f_pipe,

    output DATA [1:0] debug_x_coeff,
    output DATA [1:0] debug_y_coeff,
    output DATA [1:0] debug_z_coeff,
    output DATA [1:0] debug_distance_q,
    output logic [1:0][8:0] debug_az_degree
);

TRANSFORM_PACKET next_t2f_pipe;

DATA [1:0] x_coeff;
DATA [1:0] y_coeff;
DATA [1:0] z_coeff;

trig_lut lut(
    .clk,
    .stall,
    .azimuth(data_in.azimuth),
    .laser_id(data_in.laser_id),
    .x_coeff,
    .y_coeff,
    .z_coeff,
    .debug_az_degree(debug_az_degree)
);

logic   [1:0] valid_q;
DATA    [1:0] distance_q;

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        valid_q       <= '0;
        distance_q[0] <= '0;
        distance_q[1] <= '0;
    end else if (!stall) begin
        valid_q       <= data_in.valid;
        distance_q[0] <= data_in.distance[0];
        distance_q[1] <= data_in.distance[1];
    end
end

logic signed [31:0] math_x0, math_y0, math_z0;
logic signed [31:0] math_x1, math_y1, math_z1;

always_comb begin
    math_x0 = distance_q[0] * x_coeff[0];
    math_y0 = distance_q[0] * y_coeff[0];
    math_z0 = distance_q[0] * z_coeff[0];

    math_x1 = distance_q[1] * x_coeff[1];
    math_y1 = distance_q[1] * y_coeff[1];
    math_z1 = distance_q[1] * z_coeff[1];

    next_t2f_pipe.valid = valid_q;
    
    next_t2f_pipe.x[0] = DATA'(math_x0 >>> 15);
    next_t2f_pipe.y[0] = DATA'(math_y0 >>> 15);
    next_t2f_pipe.z[0] = DATA'(math_z0 >>> 15);

    next_t2f_pipe.x[1] = DATA'(math_x1 >>> 15);
    next_t2f_pipe.y[1] = DATA'(math_y1 >>> 15);
    next_t2f_pipe.z[1] = DATA'(math_z1 >>> 15);
end

always_ff @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        t2f_pipe <= '0;
    end else if (!stall) begin
        t2f_pipe <= next_t2f_pipe;
    end
end

assign debug_x_coeff = x_coeff;
assign debug_y_coeff = y_coeff;
assign debug_z_coeff = z_coeff;
assign debug_distance_q = distance_q;

endmodule
