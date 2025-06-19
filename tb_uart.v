`timescale 1ns / 1ps

module tb_uart ();

    reg        clk;
    reg        rst;
    reg        btn_start;
    reg  [7:0] tx_din, send_data, rx_send_data, rand_data;
    reg        rx;
    wire [7:0] rx_data;
    wire       rx_done;
    wire       tx_done;
    wire       tx_busy;
    wire       tx;

    uart_controller DUT (
        .clk(clk),
        .rst(rst),
        .btn_start(btn_start),
        .tx_din(tx_din),
        .rx(rx),
        .rx_data(rx_data),
        .rx_done(rx_done),
        .tx_done(tx_done),
        .tx_busy(tx_busy),
        .tx(tx)
    );
    integer j;

    always #5 clk = ~clk;

    initial begin
        clk = 0;
        rst = 1;
        rx = 1;
        #10;
        rst = 0;
        #100;
        rx = 0; // start
        #104160;    // 9600 1bit
        rx = 1; // data 0
        #104160;    // 9600 1bit
        rx = 0; // data 1
        #104160;    // 9600 1bit
        rx = 0; // data 2
        #104160;    // 9600 1bit
        rx = 0; // data 3
        #104160;    // 9600 1bit
        rx = 1; // data 4
        #104160;    // 9600 1bit
        rx = 1; // data 5
        #104160;    // 9600 1bit
        rx = 0; // data 6
        #104160;    // 9600 1bit
        rx = 0; // data 7
        #104160;    // 9600 1bit
        rx = 1;  // stop

        wait(tx_done);
        #10;
        for(j=0;j<8;j=j+1) begin
            rand_data = $random()%255;
            rx_send_data = rand_data;
            send_data_to_rx(rx_send_data);
            wait_for_rx();
        end

        $stop;
    end

    // to 
    integer i;
    task send_data_to_rx(input [7:0] send_data); 
        begin
            rx = 0;// start condition
            #104160;
            // rx data lsb transfer
            for(i=0;i<8;i=i+1) begin
                rx = send_data[i]; 
                #104160;
            end
            rx = 1;
            #(10416*3); // rx done
            $display("send_data = %h", send_data);
        end
    endtask

    // rx: 수신 완료시 검사기 
    task wait_for_rx();
        begin 
            wait(rx_done);
            if(rx_data == rand_data) begin 
                // pass
                $display("pass! rx_data = %h", rx_data);
            end else begin
                // fail
                $display("fail! rx_data = %h", rx_data);
            end
        end
    endtask
endmodule