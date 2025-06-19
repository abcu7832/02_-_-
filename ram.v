`timescale 1ns / 1ps

// FIFO: buffer 역할을 함
module ram(
    input            clk,
    input            rst,
    input      [7:0] wdata,
    input            wr,//push
    input            rd,//pop
    output reg [7:0] rdata,
    output full,
    output empty
);
    //reg [7:0] bram_mem;
    reg [7:0] mem[0:255];// address 8bit이므로
    reg [7:0] waddr, raddr;
    // 조합논리 출력
    //assign rdata = mem[addr];

    always @(posedge clk) begin
        if(rst) begin
            mem[addr] <= wdata;
        end else begin
            rdata <= mem[addr];
        end
    end
endmodule
