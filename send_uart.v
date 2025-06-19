`timescale 1ns / 1ps

module send_uart(
    input clk,
    input rst,
    input rx,
    input [9:0] i_send_data,
    input btn_start,
    output tx
);  
    wire w_start, w_tx_full;
    reg [1:0] c_state, n_state;
    reg [7:0] send_data_reg, send_data_next;
    reg send_reg, send_next;.
    reg [1:0] send_cnt_reg, send_cnt_next;

    btn_device U_START_BD(
        .clk(clk),
        .rst(rst),
        .i_btn(btn_start),
        .o_btn(w_start)
    );

    uart_controller U__UART_CNTL(
        .clk(clk),
        .rst(rst),
        .rx(rx),
        .rx_pop(),
        .tx_push_data(send_data_reg),
        .tx_push(send_reg),
        .rx_pop_data(), 
        .rx_empty(),
        .rx_done(),
        .tx_full(w_tx_full),
        .tx_done(),
        .tx_busy(),
        .tx(tx)
    );

    datatoascii U_DtoA(
        .i_data(i_send_data),
        .o_data(w_send_data)
    );

    always @(posedge clk, posedge rst) begin
        if (rst) begin
            c_state       <= 0;
            send_data_reg <= 0;
            send_reg      <= 0;
            send_cnt_reg<=0;
        end else begin
            c_state       <= n_state;
            send_data_reg <= send_data_next;
            send_reg      <= send_next;
            send_cnt_reg<=send_cnt_next;
        end
    end

    always @(*) begin
        n_state        = c_state;
        send_data_next = send_data_reg;
        send_next      = send_reg;
        send_cnt_next=send_cnt_reg;
        case (c_state)
            2'b00: begin
                if(w_start) begin
                    n_state = 1;

                end
            end
            2'b01: begin
                if (~w_tx_full) begin
                    send_next = 1; // send tick 생성    
                    if(send_cnt_reg<3)begin
                        send_data_next = w_send_data >> (send_cnt_reg * 8);
                    end else begin
                        n_state = 0; 
                    end
                    //send_data_next = "0";
                end else begin
                    n_state = c_state;
                end
                
            end
        endcase
    end
endmodule

// decoder, LUT
module datatoascii (
    input  [ 9:0] i_data,
    output [23:0] o_data
);
    assign o_data[7:0]   = i_data % 10 + 8'h30;
    assign o_data[15:8]  = (i_data / 10) % 10 + 8'h30;
    assign o_data[23:16] = (i_data / 100) % 10 + 8'h30;
endmodule
