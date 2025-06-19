`timescale 1ns / 1ps

module tb_fifo();
    reg        clk, rst, wr, rd;
    reg  [7:0] w_Data;
    wire       full, empty;
    wire [7:0] r_Data;

    fifo dut (
        .clk(clk),
        .rst(rst),     //controller 리셋용
        .wr(wr),      //push
        .rd(rd),      //pop
        .w_Data(w_Data),
        .full(full),
        .empty(empty),
        .r_Data(r_Data)
    );

    always #5 clk = ~clk;

    integer i;
    reg rand_wr;
    reg rand_rd;
    reg [7:0] compare_data[0:15]; // fifo 크기만큼 설정
    integer wr_count, rd_count; // puhs count pop count

    initial begin
        #0; clk = 0; rst = 1; wr = 0; rd = 0; w_Data = 0;
        #20; rst = 0;
        // full test
        for(i = 0; i < 17; i = i + 1) begin
            #10;
            wr = 1; // push
            w_Data = i;
        end
        #10; wr = 0;
        // empty test
        for(i = 0; i < 17; i = i + 1) begin
            #10;
            rd = 1; // pop
            w_Data = i;
        end
        #10;
        rd = 0;
        for(i = 0; i < 20; i = i + 1) begin
            #10;
            wr = 1;
            rd = 1;
            w_Data = i;
        end
        #10;
        wr = 0;
        #10;
        rd = 0;
        #10;
        $stop;
    end
endmodule
