`timescale 1ns / 1ps

module stopwatch_dp (
    input        clk,
    input        reset,
    input        run_stop,
    input        clear,
    output  [6:0] msec,
    output  [5:0] sec,
    output  [5:0] min,
    output  [4:0] hour
);
    wire w_tick_100hz, w_msec_tick, w_sec_tick, w_min_tick, w_hour_tick;

    tick_gen U_tick_gen_10ms( // 10ms 생성
        .clk(clk & run_stop),
        .reset(reset|clear),
        .o_tick(w_msec_tick)
    );

    time_counter #(.TICK_COUNT(100)) U_MSEC (
        .clk(clk),
        .rst(reset|clear),
        .i_tick(w_msec_tick),
        .o_time(msec),
        .o_tick(w_sec_tick)
    );

    time_counter #(.TICK_COUNT(60)) U_SEC (
        .clk(clk),
        .rst(reset|clear),
        .i_tick(w_sec_tick),
        .o_time(sec),
        .o_tick(w_min_tick)
    );

    time_counter #(.TICK_COUNT(60)) U_MIN (
        .clk(clk),
        .rst(reset|clear),
        .i_tick(w_min_tick),
        .o_time(min),
        .o_tick(w_hour_tick)
    );

    time_counter #(.TICK_COUNT(24)) U_HOUR (
        .clk(clk),
        .rst(reset|clear),
        .i_tick(w_hour_tick),
        .o_time(hour),
        .o_tick()
    );
endmodule

// 10ms 생성 => FCOUNT = 1_000_000
// 1s 생성 => msec tick 받고 FCOUNT = 100
module tick_gen #(parameter FCOUNT = 1_000_000)( // 10ms 생성
    input  clk,
    input  reset,
    output o_tick
);
    reg r_o_tick;
    reg [$clog2(FCOUNT)-1:0] r_counter;

    assign o_tick = r_o_tick;

    always @(posedge clk, posedge reset) begin
        if(reset) begin
            r_counter <= 0;
            r_o_tick <= 0;
        end else if(reset==0)begin   
            if(r_counter == FCOUNT - 1) begin
                r_counter <= 0;
                r_o_tick <= 1;
            end else begin
                r_counter <= r_counter + 1;
                r_o_tick <= 0;
            end      
        end
    end
endmodule

// 카운트 후 값 전달
module time_counter #(
    parameter TICK_COUNT = 100
) (
    input                               clk,
    input                               rst,
    input                               i_tick,
    output     [$clog2(TICK_COUNT)-1:0] o_time,
    output                              o_tick
);

    reg [$clog2(TICK_COUNT)-1:0] count_reg, count_next;
    reg o_tick_reg, o_tick_next;

    assign o_time = count_reg;
    assign o_tick = o_tick_reg;
    
    // state register
    always @(posedge clk, posedge rst) begin
        if(rst) begin
            count_reg <= 0;// nonblock => f/f 사용할때 일반적으로 
            o_tick_reg <= 1'b0;
        end else begin
            count_reg <= count_next; 
            o_tick_reg <= o_tick_next;
        end
    end

    // next state
    always @(*) begin // Combinational logic으로 구현
        count_next = count_reg; 
        o_tick_next = 1'b0;
        if(i_tick == 1'b1) begin
            if (count_reg == (TICK_COUNT - 1)) begin
                count_next = 1'b0;
                o_tick_next = 1'b1;
            end else begin
               count_next = count_reg + 1; 
               o_tick_next = 1'b0;
            end
        end
    end
endmodule

module tick_gen_100hz (
    input      clk,
    input      rst,
    output reg o_tick_100
);
    parameter FCOUNT = 1_000_000;
    reg [$clog2(FCOUNT)-1:0] r_counter;

    //state register
    always @(posedge clk, posedge rst) begin
        if(rst) begin 
            r_counter <= 0;
            o_tick_100 <= 0;
        end else begin
            if(r_counter == FCOUNT - 1) begin
                r_counter <= 0; 
                o_tick_100 <= 1'b1;
            end else begin
                r_counter <= r_counter + 1; 
                o_tick_100 <= 1'b0;
            end
        end
    end
endmodule
