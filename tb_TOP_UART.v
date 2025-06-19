`timescale 1ns / 1ps

module tb_TOP_UART();

    reg clk;
    reg rst;
    reg rx;
    wire tx;

    TOP U_UART(
        .clk(clk),
        .rst(rst),
        .rx(rx),
        .tx(tx)
    );

    always #5 clk = ~clk;

    initial begin
        #0; clk = 0; rst = 1; rx = 1;
        #20 rst = 0;
        #2000000; rx = 0;  //start 
         #104160; rx = 0;  //d0
        #104160; rx = 1;
        #104160; rx = 0;
        #104160; rx = 0; // R
        #104160; rx = 1;
        #104160; rx = 0;
        #104160; rx = 1;
        #104160; rx = 0;  //d7
        #104160; rx = 1;  //stop
        #2000000; rx = 0;  //start 
        #104160; rx = 1;  //d0
        #104160; rx = 0;
        #104160; rx = 1;
        #104160; rx = 0;
        #104160; rx = 1; // U
        #104160; rx = 0;
        #104160; rx = 1;
        #104160; rx = 0;  //d7
        #104160; rx = 1;  //stop
        #2000000; rx = 0;  //start 
        #104160; rx = 0;  //d0
        #104160; rx = 1;
        #104160; rx = 1;
        #104160; rx = 1; // N
        #104160; rx = 0;
        #104160; rx = 0;
        #104160; rx = 1;
        #104160; rx = 0;  //d7
        #104160; rx = 1;  //stop
        #2000000;
        $stop;
    end
endmodule
