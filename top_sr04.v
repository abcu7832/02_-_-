`timescale 1ns / 1ps

module top_sr04(
    input clk,
    input rst,
    input start,
    input echo,
    output trig,
    output [7:0] fnd_data,
    output [3:0] fnd_com
);
    wire [13:0] dist;
    wire o_btn;

    btn_device U_BTN (
        .clk(clk),
        .rst(rst),
        .i_btn(start),
        .o_btn(o_btn)
    );
    sr04_controller U_sr04_ctrl(
        .clk(clk),
        .rst(rst),
        .start(o_btn),
        .echo(echo),
        .trig(trig),
        .dist(dist),
        .dist_done()
    );

    fnd_controllr U_FND(
        .clk(clk),
        .reset(rst),
        .count_data(dist),
        .fnd_data(fnd_data),
        .fnd_com(fnd_com)
    );
endmodule
