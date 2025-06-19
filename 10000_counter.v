`timescale 1ns / 1ps

module counter_10000(
    input clk,
    input reset,
    output [13:0] count_data
);
    reg [13:0] r_count_data;
    assign count_data = r_count_data;

    always @(posedge clk, posedge reset) begin
        if(reset) begin
            r_count_data <= 0;// nonblock => f/f 사용할때 일반적으로 
        end else begin
            if(r_count_data == 10_000 - 1) begin
                r_count_data <= 0;
            end else begin
                r_count_data <= r_count_data + 1;
            end
        end
    end

endmodule
