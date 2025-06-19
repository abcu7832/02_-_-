`timescale 1ns / 1ps

module tb_sr04();
    reg clk;
    reg rst;
    reg start;
    reg echo;
    wire trig;
    wire [13:0] dist;
    wire dist_done;

    sr04_controller dut(
        .clk(clk),
        .rst(rst),
        .start(start),
        .echo(echo),
        .trig(trig),
        .dist(dist),
        .dist_done(dist_done)
    );

    always #5 clk = ~clk;

    initial begin
        #0; clk = 0; rst = 1; start = 0; echo = 0;
        #20; rst = 0;
        #20; start = 1;
        #10; start = 0;// button
        #10000; echo = 1;
        #(1000*1000); echo = 0;
        #100;
        $stop;
    end
endmodule
