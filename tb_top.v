`timescale 1ns / 1ps

module tb_top();
    reg clk, reset;
    wire [7:0] fnd_data;
    wire [3:0] fnd_com;

    top U_TOP(
        .clk(clk),
        .reset(reset),
        .fnd_data(fnd_data),
        .fnd_com(fnd_com)
    );

    always #5 clk = ~clk;//5ns마다 반전 => 주기 10ns

    initial begin
        #0; clk = 1'b0; reset = 1'b1;
        #20; reset = 1'b0;
        #10000
        $finish;
    end
endmodule
