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
    wire [$clog2(100)-1:0] w_msec;
    wire [$clog2(60)-1:0] w_sec, w_min;
    wire [$clog2(24)-1:0] w_hour;
    wire w_clear, w_runstop, w_mode;
    wire w_btnR, w_btnL;

    stopwatch_cu U_StopWatch_CU(
        .clk(clk),
        .reset(reset),
        .i_clear(w_btnR),
        .i_runstop(w_btnL),
        .o_clear(w_clear),
        .o_runstop(w_runstop)
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
        .msec(w_msec),
        .sec(w_sec),
        .min(w_min),
        .hour(w_hour)
    );

    fnd_controller U_FND_CONTROLLER (
        .clk(clk),
        .rst(reset),
        .sw0(sw0),
        .msec(w_msec),
        .sec(w_sec),
        .min(w_min),
        .hour(w_hour),
        .fnd_data(fnd_data),
        .fnd_com(fnd_com)
    );
endmodule