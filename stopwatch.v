`timescale 1ns / 1ps

module stopwatch (//top module
    input        clk,
    input        reset,
    input        btnR_Clear,
    input        btnL_RunStop,
    input        sw0,
    output [7:0] fnd_data,
    output [3:0] fnd_com
);
    wire [$clog2(100)-1:0] w_low_fnd;
    wire [$clog2(60)-1:0] w_high_fnd;
    wire w_clear, w_runstop, w_mode;
    wire w_btnR, w_btnL;

    stopwatch_cu U_StopWatch_CU(
        .clk(clk),
        .reset(reset),
        .i_clear(w_btnR),
        .i_runstop(w_btnL),
        .sw(sw0),
        .o_clear(w_clear),
        .o_runstop(w_runstop),
        .o_mode(w_mode)
    );

    btn_device U_btnR(
        .clk(clk),
        .rst(reset),
        .i_btn(btnR_Clear),
        .o_btn(w_btnR)
    );

    btn_device U_btnL(
        .clk(clk),
        .rst(reset),
        .i_btn(btnL_RunStop),
        .o_btn(w_btnL)
    );

    stopwatch_dp U_Stopwatch_DP(
        .clk(clk),
        .reset(reset),
        .run_stop(w_runstop),
        .clear(w_clear),
        .mode(w_mode),
        .low_fnd(w_low_fnd),
        .high_fnd(w_high_fnd)
    );

    fnd_controller U_FND_CONTROLLER (
        .clk(clk),
        .reset(reset),
        .msec(w_low_fnd),
        .sec(w_high_fnd),
        .fnd_data(fnd_data),
        .fnd_com(fnd_com)
    );
endmodule