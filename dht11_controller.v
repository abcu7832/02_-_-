`timescale 1ns / 1ps

module dht11_controller (
    input clk,
    input rst,
    input start,
    output [7:0] rh_data,
    output [7:0] t_data,
    output dht11_done,
    output dht11_valid, // checksum에 대한 신호
    output [3:0] state_led,
    inout dht11_io
);
    wire w_tick;

    tick_gen_10us U_Tick (
        .clk(clk),
        .rst(rst),
        .o_tick(w_tick)
    );

    parameter IDLE = 0, START = 1, WAIT = 2, SYNC_L = 3, SYNC_H = 4, DATA_SYNC = 5, DATA_DETECT = 6, VALID = 7, DATA_DETECT_f = 8, STOP = 9;

    reg [3:0] c_state, n_state;
    reg [$clog2(1900)-1:0] tick_cnt_reg, tick_cnt_next;
    reg dht11_reg, dht11_next;
    reg io_en_reg, io_en_next;
    reg [39:0] data_reg, data_next;
    reg [$clog2(40)-1:0] data_cnt_reg, data_cnt_next;

    reg dht11_done_reg, dht11_done_next;
    reg dht11_valid_reg, dht11_valid_next;
    reg w_tick_d, dht11_io_d, tick_edge_r, dht11_io_r, dht11_io_f;// rising edge

    assign dht11_io = (io_en_reg) ? dht11_reg : 1'bz;  //출력인 경우
    assign state_led = c_state;
    assign dht11_valid = dht11_valid_reg;
    assign dht11_done = dht11_done_reg;
    
    assign rh_data = data_reg[23:16];//습도 => dht11_done 신호 나올때 받으면 됨.
    assign t_data = data_reg[39:32];//온도 => dht11_done 신호 나올때 받으면 됨.

    always @(posedge clk, posedge rst) begin
        if (rst) begin
            c_state         <= IDLE;
            tick_cnt_reg    <= 0;
            dht11_reg       <= 1;  // 초기값 항상 high로
            io_en_reg       <= 1;  // idle에서 항상 출력 모드
            data_reg        <= 0;
            data_cnt_reg    <= 39;
            dht11_valid_reg <= 0;
            w_tick_d        <= 0;
            dht11_io_d      <= 0;
            dht11_done_reg  <= 0;
            dht11_valid_reg <= 0;

        end else begin
            c_state         <= n_state;
            tick_cnt_reg    <= tick_cnt_next;
            dht11_reg       <= dht11_next;
            io_en_reg       <= io_en_next;
            data_reg        <= data_next;
            data_cnt_reg    <= data_cnt_next;

            dht11_valid_reg <= dht11_valid_next;
            dht11_done_reg  <= (dht11_done_reg)?0:dht11_done_next;

            w_tick_d        <= w_tick;
            tick_edge_r     <= (tick_edge_r)? 0: (~w_tick_d & w_tick);// tick rising edge

            dht11_io_d      <= dht11_io;
            dht11_io_r      <= (dht11_io_r)? 0: (~dht11_io_d & dht11_io);// io rising edge
            dht11_io_f      <= (dht11_io_f)? 0: (dht11_io_d & ~dht11_io);// io falling edge
        end
    end

    always @(*) begin
        n_state          = c_state;
        tick_cnt_next    = tick_cnt_reg;
        dht11_next       = dht11_reg;
        io_en_next       = io_en_reg;
        data_next        = data_reg;
        dht11_valid_next = dht11_valid_reg;
        dht11_done_next  = dht11_done_reg;
        data_cnt_next    = data_cnt_reg;

        case (c_state)
            IDLE: begin // 0
                dht11_next = 1;
                io_en_next = 1;//출력모드
                if (start) begin
                    n_state = START;
                    dht11_valid_next = 0;//valid
                end
            end
            START: begin // 1
                if (w_tick) begin
                    // 카운트
                    dht11_next = 0;
                    if (tick_cnt_reg == 1900) begin
                        n_state       = WAIT;
                        tick_cnt_next = 0;
                    end else begin
                        tick_cnt_next = tick_cnt_reg + 1;
                    end
                end
            end
            WAIT: begin // 2
                dht11_next = 1;
                if (w_tick) begin
                    if (tick_cnt_reg == 2) begin
                        n_state       = SYNC_L;
                        tick_cnt_next = 0;
                        io_en_next    = 0; // input 모드
                    end else begin
                        tick_cnt_next = tick_cnt_reg + 1;
                    end
                end
            end
            SYNC_L: begin // 3
                if (w_tick & dht11_io) begin
                    n_state = SYNC_H;
                end
            end
            SYNC_H: begin // 4
                if (w_tick & !dht11_io) begin
                    n_state = DATA_SYNC;
                    tick_cnt_next = 0;
                end
            end
            DATA_SYNC: begin // 5 
                if (w_tick) begin
                    if (dht11_io) begin
                        n_state       = DATA_DETECT;
                        dht11_next    = 0;
                        tick_cnt_next = 0;
                    end
                end
            end
            DATA_DETECT: begin  // 6
                if (dht11_io & tick_edge_r) begin  // tick이 rising edge일 때
                    tick_cnt_next = tick_cnt_reg + 1;  // 증가
                end
                if (dht11_io_f) begin  // input: falling edge일 때
                    n_state = DATA_DETECT_f;
                end
            end
            VALID: begin
                if(data_reg[39:32] + data_reg[31:24] + data_reg[23:16] + data_reg[15:8] == data_reg[7:0]) begin
                    dht11_valid_next = 1;
                end else begin
                    dht11_valid_next = 0;
                end
                n_state = STOP;
            end
            DATA_DETECT_f: begin
                if (tick_cnt_reg >= 5) begin  // 1인 경우
                    data_next[data_cnt_reg] = 1;
                end else begin  // 0인 경우
                    data_next[data_cnt_reg] = 0;
                end
                
                if (data_cnt_reg == 0) begin
                    tick_cnt_next = 0;
                    data_cnt_next = 39;
                    dht11_done_next = 1;  //done
                    n_state = VALID;
                end else begin
                    n_state = DATA_SYNC; // data_sync로 
                    data_cnt_next = data_cnt_reg - 1;
                end
            end
            STOP: begin
                if (tick_edge_r) begin // rising edge
                    tick_cnt_next = tick_cnt_reg + 1;
                end
                if(tick_cnt_reg == 4) begin
                    n_state = IDLE;
                end
            end
        endcase
    end
endmodule

//10us 틱 발생기
module tick_gen_10us (
    input  clk,
    input  rst,
    output o_tick
);
    parameter F_CNT = 1000;
    reg [$clog2(F_CNT) - 1:0] counter_reg;
    reg tick_reg;

    assign o_tick = tick_reg;

    always @(posedge clk, posedge rst) begin
        if (rst) begin
            counter_reg <= 0;
            tick_reg    <= 0;
        end else begin
            if (counter_reg == F_CNT - 1) begin
                counter_reg <= 0;
                tick_reg    <= 1;
            end else begin
                counter_reg <= counter_reg + 1;
                tick_reg    <= 0;
            end
        end
    end
endmodule
