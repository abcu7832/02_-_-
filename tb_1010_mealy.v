module TB;
    reg clk, rst_n, x;
    wire z;

    seq_detector_1010 dut (
        .clk(clk),
        .reset(rst_n),
        .din_bit(x),
        .dout_bit(z)
    );
    always #5 clk = ~clk;

    initial begin
        clk = 0;
        x   = 0;
        #10 rst_n = 1;
        #10 rst_n = 0;

        #10 x = 1;
        #10 x = 1;
        #10 x = 0;
        #10 x = 1;
        #10 x = 0;
        #10 x = 1;
        #10 x = 0;
        #10 x = 1;
        #10 x = 1;
        #10 x = 1;
        #10 x = 0;
        #10 x = 1;
        #10 x = 0;
        #10 x = 1;
        #10 x = 0;
        #10;
        $finish;
    end

endmodule
