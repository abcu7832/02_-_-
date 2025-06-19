`timescale 1ns / 1ps

module tb_stopwatch();
    reg clk, reset, btnR_Clear, btnL_RunStop, sw0;
    wire [7:0] fnd_data;
    wire [3:0] fnd_com;

    stopwatch U_StopWatch(//top module
        .clk(clk),
        .reset(reset),
        .btnR_Clear(btnR_Clear),
        .btnL_RunStop(btnL_RunStop),
        .sw0(sw0),
        .fnd_data(fnd_data),
        .fnd_com(fnd_com)
    );

    always #5 clk = ~clk;

    initial begin
        #0 clk = 0; reset = 1; btnR_Clear = 0; sw0 = 0; btnL_RunStop=0;
        #20 reset = 0; 
        #1000000 btnL_RunStop=1;
        #10 btnL_RunStop=0;//RUN
        #1000000 btnL_RunStop = 1;
        #10 btnL_RunStop = 0;//STOP
        #10000000 btnR_Clear = 1;
        #10 btnR_Clear = 0;//CLEAR
        #1000000 btnR_Clear = 1;
        #10 btnR_Clear = 0;//STOP
        #1000000 btnL_RunStop = 1;
        #10 btnL_RunStop = 0;//RUN
        #1000000 sw0 = 1;
        #10000000 sw0 = 0;
        #1000000
        $finish;
    end

endmodule
