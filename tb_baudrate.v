`timescale 1ns / 1ps

module tb_baudrate();
    reg  clk;
    reg  rst;
    wire baud_tick;

    baudrate U_BAUDRATE (
        .clk(clk),
        .rst(rst),
        .baud_tick(baud_tick)
    );

    always #5 clk = ~clk; 
    
    initial begin
        #0; clk = 0; rst = 1;
        #20; rst = 0;
        #1000000000;
        $finish;
    end
endmodule
