`timescale 1ns / 1ps

module tb_uart_cntl ();
    reg clk, rst, start, rx;
    wire tx;

    uart_controller U_UC (
        .clk(clk),
        .rst(rst),
        .btn_start(start),  //up button에 할당
        .tx(tx),
        .rx(rx)
    );

    always #5 clk = ~clk;

    initial begin
        #0; clk = 0; rst = 1; start = 0; rx = 1;
        #20; rst = 0;
        #10000; start = 1;//BUTTON PUSH
        #1000000; start = 0;
        #2000000; rx = 0;  //start 
        #104160; rx = 1;  //d0
        #104160; rx = 0;
        #104160; rx = 0;
        #104160; rx = 0;
        #104160; rx = 1;
        #104160; rx = 1;
        #104160; rx = 0;
        #104160; rx = 0;  //d7
        #104160; rx = 1;  //stop
        #200000000;
        $stop;
    end
endmodule
