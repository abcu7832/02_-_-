`timescale 1ns / 1ps

module sr04_controller(
    input           clk,
    input rst,
    input start,
    input echo,
    output trig,
    output [13:0] dist,
    output dist_done
);
    wire w_tick;

    start_trigger U_Start_trigg (
        .clk(clk),
        .rst(rst),
        .i_tick(w_tick),
        .start(start),
        .o_sr04_trigger(trig)
    );

    distance U_Distance (
        .clk(clk),
        .rst(rst),
        .i_tick(w_tick),
        .echo(echo),
        .distance(dist),
        .dist_done(dist_done)
    );

    tick_gen U_tick_gen (
        .clk(clk),
        .rst(rst),
        .o_tick_1mhz(w_tick)//1MHz
    );
endmodule

module distance (
    input             clk,
    input             rst,
    input             i_tick,
    input             echo,
    output reg [13:0] distance,
    output reg        dist_done // tick 발생
);
    // distance(cm) = count/58
    reg [14:0] count; // 58 * 400 $clog2(58*400)
    reg echo_d;

    always @(posedge clk, posedge rst) begin
        if(rst) begin
            count       <= 0;
            echo_d      <= 0;
            distance    <= 0;
            dist_done   <= 0;
        end else begin
            echo_d <= echo;
            if(~echo_d & echo) begin // positive edge 감지
                count <= 0;
                distance <= 0;
            end
            if(echo & i_tick) begin
                count <= count + 1;
                if(count == 58) begin
                    distance <= distance + 1;
                end
            end
            if(dist_done == 1) begin
                dist_done <= 0; // 틱 발생
            end
            if(echo_d & ~echo) begin // falling edge 감지
                dist_done <= 1; // 틱 발생
            end
        end
    end
endmodule

module start_trigger (
    input clk,
    input rst,
    input i_tick,
    input start,
    output reg o_sr04_trigger
);
    reg start_d, start_edge;
    reg [3:0] count;
    reg trig_active;

    // 버튼의 상승 에지 감지
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            start_d <= 0;
            start_edge <= 0;
            o_sr04_trigger <= 0;
            count <= 0;
            trig_active <= 0;
        end else begin
            start_edge <= start & ~start_d; // rising edge
            start_d <= start;
            if (start_edge) begin
                trig_active <= 1;
                count <= 0;
            end
            if (trig_active && i_tick) begin
                if (count < 10) begin
                    o_sr04_trigger <= 1;
                    count <= count + 1;
                end else begin
                    o_sr04_trigger <= 0;
                    trig_active <= 0;
                end
            end else if (!trig_active) begin
                o_sr04_trigger <= 0;
            end
        end
    end
endmodule

module tick_gen (
    input clk,
    input rst,
    output o_tick_1mhz
);
    parameter F_COUNT = 100;

    reg [$clog2(F_COUNT)-1:0] count;
    reg tick;

    assign o_tick_1mhz = tick;

    always @(posedge clk, posedge rst) begin
        if(rst) begin
            count <= 0;
        end else begin
            if(count == F_COUNT - 1) begin
                count <= 0;
                tick <= 1'b1;
            end else begin
                count <= count + 1;
                tick <= 1'b0;
            end
        end
    end
endmodule
