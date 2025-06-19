`timescale 1ns / 1ps

module TOP(
    input clk,
    input rst,
    input rx,
    output tx
);
    wire [7:0] w_tx_data, w_rx_data, rx2tx_data;
    wire w_rx_done, w_tx_busy;
    wire tx2rx, tx2uart, rx2tx;

    uart_controller UART(
        .clk(clk),
        .rst(rst),
        .btn_start(~tx2uart),  // up button에 할당
        .tx_din(w_tx_data),
        .rx(rx),
        .rx_data(w_rx_data),
        .rx_done(w_rx_done),
        .tx_busy(w_tx_busy),
        .tx(tx)
    );

    fifo U_RX_FIFO(
        .clk(clk),
        .rst(rst),
        .wr(w_rx_done),
        .rd(~tx2rx),
        .w_Data(w_rx_data),
        .full(),//빈 상태
        .empty(rx2tx),
        .r_Data(rx2tx_data)
    );

    fifo U_TX_FIFO(
        .clk(clk),
        .rst(rst),
        .wr(~rx2tx),
        .rd(~w_tx_busy),
        .w_Data(rx2tx_data),
        .full(tx2rx),
        .empty(tx2uart),
        .r_Data(w_tx_data)
    );
endmodule
