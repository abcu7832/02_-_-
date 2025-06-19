`timescale 1ns / 1ps

module uart_command_receiver #(
    parameter CMD_BUF_SIZE = 8
)(
    input        clk,
    input        rst,
    input        rx,                // UART RX 입력
    output reg   reset_cmd,// tick으로 나옴
    output reg   rs_cmd,   // tick으로 나옴
    output reg   clear_cmd,// tick으로 나옴
    output reg   sr_cmd,   // tick으로 나옴
    output reg   dht_cmd,  // tick으로 나옴
    output reg   up_cmd,   //tick으로 나옴
    output reg   down_cmd, //tick으로 나옴
    output reg   led       // 스위치 처럼 동작
);

    wire [7:0] data_out;
    wire       empty;
    wire       full;
    reg        pop;

    // 내부 명령어 버퍼
    reg [7:0] cmd_buf [0:7];
    reg [$clog2(CMD_BUF_SIZE):0] cmd_idx;

    // FSM 상태
    parameter IDLE = 0, RECV = 1, CHECK = 2;
    reg [1:0] state, next_state;

    // UART 수신 FIFO
    uart_with_fifo uart0 (
        .clk(clk),
        .rst(rst),
        .rx(rx),
        .pop(pop),
        .data_out(data_out),
        .empty(empty),
        .full(full)
    );
    reg pop_d;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state <= IDLE;
            cmd_idx <= 0;
        end else begin
            state <= next_state;
            pop_d <= pop;

            if (state == RECV && pop) begin
                cmd_buf[cmd_idx] <= data_out;
                cmd_idx <= cmd_idx + 1;
                if(data_out == 8'h0A) begin
                    cmd_idx <= 0;
                end
            end
        end
    end

    always @(*) begin
        next_state = state;
        pop = 0;
        case (state)
            IDLE: begin
                if (!empty) begin
                    next_state = RECV;
                end
            end

            RECV: begin
                if (!empty) begin
                    pop = 1;
                    next_state = CHECK;
                end
            end

            CHECK: begin
                next_state = IDLE;
            end
        endcase
    end

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            rs_cmd <= 0;
            reset_cmd <= 0;
            led <= 0;
            clear_cmd <= 0;
            sr_cmd <= 0;
            dht_cmd <= 0;
            up_cmd <= 0;
            down_cmd <= 0;
        end else begin
            // 기본 비활성화
            rs_cmd <= 0;
            reset_cmd <= 0;
            clear_cmd <= 0;
            sr_cmd <= 0;
            dht_cmd <= 0;
            up_cmd <= 0;
            down_cmd <= 0;
            
            if ((state == CHECK) && pop_d) begin
                if ((cmd_idx == 4) &&
                    (cmd_buf[0] == "R") &&
                    (cmd_buf[1] == "/") &&
                    (cmd_buf[2] == "S")) begin
                    rs_cmd <= 1;
                end else if ((cmd_idx == 6) &&
                    (cmd_buf[0] == "R") &&
                    (cmd_buf[1] == "E") &&
                    (cmd_buf[2] == "S") &&
                    (cmd_buf[3] == "E") &&
                    (cmd_buf[4] == "T")) begin
                    reset_cmd <= 1;
                end else if ((cmd_idx == 6) &&
                    (cmd_buf[0] == "C") &&
                    (cmd_buf[1] == "L") &&
                    (cmd_buf[2] == "E") &&
                    (cmd_buf[3] == "A") &&
                    (cmd_buf[4] == "R")) begin
                    clear_cmd <= 1;
                end else if ((cmd_idx == 3) &&
                    (cmd_buf[0] == "S") &&
                    (cmd_buf[1] == "R")) begin
                    sr_cmd <= 1;
                end else if ((cmd_idx == 4) &&
                    (cmd_buf[0] == "D") &&
                    (cmd_buf[1] == "H") &&
                    (cmd_buf[2] == "T")) begin
                    dht_cmd <= 1;
                end else if ((cmd_idx == 3) &&
                    (cmd_buf[0] == "U") &&
                    (cmd_buf[1] == "P")) begin
                    up_cmd <= 1;
                end else if ((cmd_idx == 5) &&
                    (cmd_buf[0] == "D") &&
                    (cmd_buf[1] == "O") &&
                    (cmd_buf[2] == "W") &&
                    (cmd_buf[3] == "N")) begin
                    down_cmd <= 1;
                end else if ((cmd_idx == 4) &&
                    (cmd_buf[0] == "S") &&
                    (cmd_buf[1] == "/") &&
                    (cmd_buf[2] == "W")) begin
                    led <= led ? 0 : 1;
                end
            end
        end
    end
endmodule

module uart_with_fifo (
    input              clk,
    input              rst,
    input              rx,
    input              pop,              // 외부에서 데이터 읽기 요청

    output [7:0]       data_out,         // 읽은 데이터 출력
    output             empty,
    output             full
);

    wire       w_bd_tick;
    wire       rx_done;
    wire [7:0] rx_data;

    fifo U_FIFO_RX (
        .clk(clk),
        .rst(rst),
        .push(rx_done),
        .pop(pop),
        .push_data(rx_data),
        .full(full), // 사용
        .empty(empty), // 사용
        .pop_data(data_out) // output 8bit
    );

    uart_rx U_RX(
        .clk(clk),
        .rst(rst),
        .bd_tick(w_bd_tick),
        .rx(rx),
        .o_dout(rx_data), // 8비트
        .o_rx_done(rx_done)
    );

    baudrate U_BR (
        .clk       (clk),
        .rst       (rst),
        .baud_tick (w_bd_tick)
    );
endmodule

module uart_rx (
    input        clk,
    input        rst,
    input        bd_tick,
    input        rx,
    output [7:0] o_dout,
    output       o_rx_done
);
    localparam IDLE = 0, START = 1, DATA = 2, DATA_READ = 3, STOP = 4;

    reg [2:0] c_state, n_state;
    reg [3:0] b_cnt_reg, b_cnt_next;
    reg [3:0] d_cnt_reg, d_cnt_next;
    reg [7:0] dout_reg, dout_next;
    reg rx_done_reg, rx_done_next;

    assign o_dout    = dout_reg;
    assign o_rx_done = rx_done_reg;

    // state
    always @(posedge clk, posedge rst) begin
        if (rst) begin
            c_state     <= IDLE;
            b_cnt_reg   <= 0;
            d_cnt_reg   <= 0;
            dout_reg    <= 0;
            rx_done_reg <= 0;
        end else begin
            c_state     <= n_state;
            b_cnt_reg   <= b_cnt_next;
            d_cnt_reg   <= d_cnt_next;
            dout_reg    <= dout_next;
            rx_done_reg <= rx_done_next;
        end
    end

    // next
    always @(*) begin
        n_state      = c_state;
        b_cnt_next   = b_cnt_reg;
        d_cnt_next   = d_cnt_reg;
        dout_next    = dout_reg;
        rx_done_next = rx_done_reg;
        case (c_state)
            IDLE: begin
                b_cnt_next = 0;
                d_cnt_next = 0;
                rx_done_next = 1'b0;
                if (bd_tick) begin
                    if (rx == 1'b0) begin
                        n_state = START;
                    end
                end
            end
            START: begin
                if (bd_tick) begin
                    if (b_cnt_reg == 11) begin
                        n_state = DATA_READ;
                        b_cnt_next = 0;
                    end else begin
                        b_cnt_next = b_cnt_reg + 1;
                    end
                end
            end
            DATA_READ: begin
                dout_next = {rx, dout_reg[7:1]};
                n_state   = DATA;
            end
            DATA: begin
                if (bd_tick) begin
                    if (b_cnt_reg == 7) begin
                        if (d_cnt_reg == 7) begin
                            n_state = STOP;
                        end else begin
                            d_cnt_next = d_cnt_reg + 1;
                            b_cnt_next = 0;
                            n_state = DATA_READ;
                        end
                    end else begin
                        b_cnt_next = b_cnt_reg + 1;
                    end
                end
            end
            STOP: begin
                if (bd_tick) begin
                    n_state = IDLE;
                    rx_done_next = 1'b1;
                end
            end
        endcase
    end
endmodule

module baudrate (
    input clk,
    input rst,
    output baud_tick
);
    //clk 100Mhz
    parameter BAUD = 9600;
    parameter BAUD_COUNT = 100_000_000 / (BAUD * 8);
    reg [$clog2(BAUD_COUNT)-1:0] count_reg, count_next;
    reg baud_tick_reg, baud_tick_next;

    assign baud_tick = baud_tick_reg;

    always @(posedge clk, posedge rst) begin
        if (rst) begin
            count_reg <= 0;
            baud_tick_reg <= 0;
        end else begin
            count_reg <= count_next;
            baud_tick_reg <= baud_tick_next;
        end
    end

    always @(*) begin
        count_next = count_reg;
        baud_tick_next = 0;  // 둘다 상관 없다 baud_tick_reg
        if (count_reg == BAUD_COUNT - 1) begin
            count_next = 0;
            baud_tick_next = 1'b1;
        end else begin
            count_next = count_reg + 1;
            baud_tick_next = 1'b0;
        end
    end
endmodule

module fifo (
    input        clk,
    input        rst,
    input        push,
    input        pop,
    input  [7:0] push_data,
    output       full,
    output       empty,
    output [7:0] pop_data
);

    wire [3:0] w_w_ptr, w_r_ptr;

    fifo_cu U_FIFO_CU (
        .clk  (clk),
        .rst  (rst),
        .push (push),
        .pop  (pop),
        .w_ptr(w_w_ptr),
        .r_ptr(w_r_ptr),
        .full (full),
        .empty(empty)
    );

    register_file U_Reg_File (
        .clk(clk),
        .wr_en(push & (~full)),  // write enable
        .wdata(push_data),
        .w_ptr(w_w_ptr),  // write address
        .r_ptr(w_r_ptr),  // read address
        .rdata(pop_data)
    );

endmodule

module fifo_cu (
    input clk,
    input rst,
    input push,
    input pop,
    output [3:0] w_ptr,
    output [3:0] r_ptr,
    output full,
    output empty
);

    reg [3:0] w_ptr_reg, w_ptr_next, r_ptr_reg, r_ptr_next;
    reg full_reg, full_next, empty_reg, empty_next;

    assign w_ptr = w_ptr_reg;
    assign r_ptr = r_ptr_reg;
    assign full  = full_reg;
    assign empty = empty_reg;

    always @(posedge clk, posedge rst) begin
        if (rst) begin
            w_ptr_reg <= 0;
            r_ptr_reg <= 0;
            full_reg  <= 0;
            empty_reg <= 1;
        end else begin
            w_ptr_reg <= w_ptr_next;
            r_ptr_reg <= r_ptr_next;
            full_reg  <= full_next;
            empty_reg <= empty_next;
        end
    end

    always @(*) begin
        w_ptr_next = w_ptr_reg;
        r_ptr_next = r_ptr_reg;
        full_next  = full_reg;
        empty_next = empty_reg;
        case ({
            pop, push
        })
            2'b01: begin  // push
                if (full_reg == 1'b0) begin
                    w_ptr_next = w_ptr_reg + 1;
                    empty_next = 0;
                    if (w_ptr_next == r_ptr_reg) begin
                        full_next = 1'b1;
                    end
                end
            end
            2'b10: begin  // pop
                if (empty_reg == 1'b0) begin
                    r_ptr_next = r_ptr_reg + 1;
                    full_next  = 1'b0;
                    if (w_ptr_reg == r_ptr_next) begin
                        empty_next = 1;
                    end
                end
            end
            2'b11: begin  // push, pop
                if (empty_reg == 1'b1) begin
                    w_ptr_next = w_ptr_reg + 1;
                    empty_next = 1'b0;
                end else if (full_reg == 1'b1) begin
                    r_ptr_next = r_ptr_reg + 1;
                    full_next  = 1'b0;
                end else begin
                    w_ptr_next = w_ptr_reg + 1;
                    r_ptr_next = r_ptr_reg + 1;
                end
            end
        endcase
    end
endmodule

module register_file #(
    parameter DEPTH = 16,
    WIDTH = 4
) (
    input              clk,
    input              wr_en,  // write enable
    input  [      7:0] wdata,
    input  [WIDTH-1:0] w_ptr,  // write address
    input  [WIDTH-1:0] r_ptr,  // read address
    output [      7:0] rdata
);

    reg [7:0] mem[0:DEPTH - 1];  // mem[0:2**WIDTH - 1]   **:제곱
    assign rdata = mem[r_ptr];

    always @(posedge clk) begin
        if (wr_en) begin
            mem[w_ptr] <= wdata;
        end
        // rdata <= mem[r_ptr];
    end

endmodule
