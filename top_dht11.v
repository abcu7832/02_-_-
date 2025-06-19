`timescale 1ns / 1ps

module top_dht11(
    input clk,
    input rst,
    input start,
    output [7:0] fnd_data,
    output [3:0] fnd_com,
    output [3:0] state_led,
    output LED,
    inout dht11_io
);

    wire [7:0] w_t_data, w_rh_data;
    wire w_btn;
    wire w_dht11_valid;

    assign LED = w_dht11_valid;

    btn_device U_BTN (
        .clk(clk),
        .rst(rst),
        .i_btn(start),
        .o_btn(w_btn)
    );

    dht11_controller U_dht11 (
        .clk(clk),
        .rst(rst),
        .start(w_btn),
        .rh_data(w_rh_data),
        .t_data(w_t_data),
        .dht11_done(),
        .dht11_valid(w_dht11_valid), // checksum에 대한 신호
        .state_led(state_led),
        .dht11_io(dht11_io)
    );

    fnd_controllr U_FND(
        .clk(clk),
        .reset(rst),
        .rh_data(w_rh_data),
        .t_data(w_t_data),
        .fnd_data(fnd_data),
        .fnd_com(fnd_com)
    );
endmodule
