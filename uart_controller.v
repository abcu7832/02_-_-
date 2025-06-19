`timescale 1ns / 1ps

module uart_controller (
    input        clk,
    input        rst,
    input        btn_start,  //up button에 할당
    input  [7:0] tx_din,
    input        rx,
    //output [7:0] rx_data,
    //output       rx_done,
    //output       tx_done,
    output       tx
);
    wire w_baud_tick, w_btn_start;
    wire w_tx_busy, w_tx_done;
    wire [7:0] w_dout;
    wire w_rx_done;

    assign rx_done = w_rx_done;

    btn_device U_BTN_U (
        .clk(clk),
        .rst(rst),
        .i_btn(btn_start),
        .o_btn(w_btn_start)
    );
    
    uart_tx U_TX (
        .clk(clk),
        .rst(rst),
        .baud_tick(w_baud_tick),
        .start(w_btn_start|w_rx_done),
        .din(w_dout),
        .o_tx_done(w_tx_done),
        .o_tx_busy(w_tx_busy),
        .o_tx(tx)
    );

    uart_rx U_RX (
        .clk(clk),
        .rst(rst),
        .b_tick(w_baud_tick),
        .rx(rx),
        .dout(w_dout),
        .rx_done(w_rx_done)
    );

    baudrate U_BR (
        .clk(clk),
        .rst(rst),
        .baud_tick(w_baud_tick)
    );

endmodule
