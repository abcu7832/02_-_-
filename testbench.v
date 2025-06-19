`timescale 1ns / 1ps

module testbench();

    reg clk;
    reg rst;
    reg rx;

    wire reset_cmd;
    wire rs_cmd;
    wire clear_cmd;
    wire sr_cmd;
    wire dht_cmd;
    wire up_cmd;
    wire down_cmd;
    wire led;

    // uut 인스턴스
    uart_command_receiver uut (
        .clk(clk),
        .rst(rst),
        .rx(rx),
        .reset_cmd(reset_cmd),
        .rs_cmd(rs_cmd),
        .clear_cmd(clear_cmd),
        .sr_cmd(sr_cmd),
        .dht_cmd(dht_cmd),
        .up_cmd(up_cmd),
        .down_cmd(down_cmd),
        .led(led)
    );

    // 100MHz 클럭 생성
    initial clk = 0;
    always #5 clk = ~clk;  // 10ns 주기

    // UART 시뮬레이션 파라미터
    parameter CLK_FREQ = 100_000_000;
    parameter BAUD = 9600;
    parameter BIT_TIME = CLK_FREQ / BAUD;  // 약 868 클럭 주기

    // UART TX 시뮬레이션 태스크 (여기서는 rx에 직접 비트 넣기)
    task uart_send_byte;
        input [7:0] data;
        integer i;
        begin
            // start bit (0)
            rx = 0;
            #(BIT_TIME*10);

            // data bits, LSB first
            for (i=0; i<8; i=i+1) begin
                rx = data[i];
                #(BIT_TIME*10);
            end

            // stop bit (1)
            rx = 1;
            #(BIT_TIME*10);
        end
    endtask

    reg prev_run_cmd;
    // 초기화 및 테스트 시나리오
    initial begin
        rst = 1;
        rx = 1;  // idle 상태는 1
        prev_run_cmd = 0;
        #(100);
        rst = 0;

        // "RUN\r\n" 전송
        uart_send_byte("L");
        uart_send_byte("E");
        uart_send_byte("D");
        uart_send_byte(8'h0D);  // \r
        uart_send_byte(8'h0A);  // \n
        #10000000;
        uart_send_byte("L");
        uart_send_byte("E");
        uart_send_byte("D");
        uart_send_byte(8'h0D);  // \r
        uart_send_byte(8'h0A);  // \n
        #10000000;
        uart_send_byte("R");
        uart_send_byte("/");
        uart_send_byte("S");
        uart_send_byte(8'h0D);  // \r
        uart_send_byte(8'h0A);  // \n
        #10000000;
        uart_send_byte("R");
        uart_send_byte("E");
        uart_send_byte("S");
        uart_send_byte("E");
        uart_send_byte("T");
        uart_send_byte(8'h0D);  // \r
        uart_send_byte(8'h0A);  // \n
        #10000000;
        uart_send_byte("C");
        uart_send_byte("L");
        uart_send_byte("E");
        uart_send_byte("A");
        uart_send_byte("R");
        uart_send_byte(8'h0D);  // \r
        uart_send_byte(8'h0A);  // \n
        #10000000;
        uart_send_byte("S");
        uart_send_byte("R");
        uart_send_byte(8'h0D);  // \r
        uart_send_byte(8'h0A);  // \n
        #10000000;
        uart_send_byte("D");
        uart_send_byte("H");
        uart_send_byte("T");
        uart_send_byte(8'h0D);  // \r
        uart_send_byte(8'h0A);  // \n
        #10000000;
        uart_send_byte("U");
        uart_send_byte("P");
        uart_send_byte(8'h0D);  // \r
        uart_send_byte(8'h0A);  // \n
        #10000000;
        uart_send_byte("D");
        uart_send_byte("O");
        uart_send_byte("W");
        uart_send_byte("N");
        uart_send_byte(8'h0D);  // \r
        uart_send_byte(8'h0A);  // \n
        #10000000;
        $stop;
    end

endmodule
