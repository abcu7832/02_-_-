`timescale 1ns / 1ps

module tb_calculator();
    reg clk, reset;
    reg [7:0] a, b;
    wire [7:0] fnd_data;
    wire [3:0] fnd_com;

    calculator dut (
        .clk(clk),
        .reset(reset),
        .a(a[7:0]),
        .b(b[7:0]),
        .fnd_data(fnd_data[7:0]),
        .fnd_com(fnd_com[3:0])
    );

    integer i, j;

    always #5 clk = ~clk;//5ns마다 반전 => 주기 10ns

    initial begin
        #0; a = 8'h00; b = 8'h00; clk = 1'b0; reset = 1'b1;
        #20; reset = 1'b0;
        for (i = 0; i <= 110; i = i + 1) begin
            for (j = 0; j <= 110; j = j + 1) begin
                a = j;
                b = i;
                #1;
            end
        end
        $finish;
        //$stop;
    end
endmodule
