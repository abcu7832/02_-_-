`timescale 1ns / 1ps

module uart_controller (
    input        clk,
    input        rst,
    input        rx,
    input        rx_pop,
    input  [7:0] tx_push_data,
    input        tx_push,
    output [7:0] rx_pop_data,    
    output       rx_empty,
    output       rx_done,
    output       tx_full,
    output       tx_done,
    output       tx_busy,
    output       tx
);
    wire w_baud_tick;
    wire [7:0] w_tx_data, w_rx_data, rx2tx_data;
    wire w_rx_done, w_tx_busy, w_tx_done;
    wire tx2rx, tx2uart, rx2tx;

    assign pop_data = w_rx_data;
    assign rx_done = w_rx_done;
    assign tx_done = w_tx_done;
    assign tx_busy = w_tx_busy;
    
    fifo U_TX_FIFO(
        .clk(clk),
        .rst(rst),
        .wr(tx_push),
        .rd(~w_tx_busy),
        .w_Data(tx_push_data),
        .full(tx_full), 
        .empty(tx2uart),
        .r_Data(w_tx_data)
    );

    fifo U_RX_FIFO(
        .clk(clk),
        .rst(rst),
        .wr(w_rx_done),
        .rd(rx_pop),
        .w_Data(w_rx_data),
        .full(),
        .empty(rx_empty), 
        .r_Data(rx_pop_data)
    );

    uart_tx U_TX (
        .clk(clk),
        .rst(rst),
        .baud_tick(w_baud_tick),
        .start(w_rx_done),
        .din(w_tx_data),
        .o_tx_done(w_tx_done),
        .o_tx_busy(w_tx_busy),
        .o_tx(tx)
    );

    uart_rx U_RX (
        .clk(clk),
        .rst(rst),
        .b_tick(w_baud_tick),
        .rx(rx),
        .dout(w_rx_data),
        .rx_done(w_rx_done)
    );

    baudrate U_BR (
        .clk(clk),
        .rst(rst),
        .baud_tick(w_baud_tick)
    );
endmodule

