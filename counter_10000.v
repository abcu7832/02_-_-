`timescale 1ns / 1ps

module top (
    input clk,
    input reset,
    output [7:0] fnd_data,
    output [3:0] fnd_com
);

    wire [13:0] w_count_data;
    wire w_clk;

    counter_10000 Counter_10000 (
        .clk(w_clk),
        .reset(reset),
        .count_data(w_count_data)
    );

    sec_generator U_Sec_Gen(
        .clk(clk),
        .reset(reset),
        .o_clk(w_clk)
    );

    fnd_controller U_FND_CNTR (
        .clk(clk),
        .reset(reset),
        .count_data(w_count_data),
        .fnd_data(fnd_data),
        .fnd_com(fnd_com)
    );

endmodule

module sec_generator (
    input clk,
    input reset,
    output o_clk
);
    reg r_clk;
    reg [26:0] r_counter;
    assign o_clk = r_clk;

    always @(posedge clk, posedge reset) begin
        if(reset) begin
            r_counter <= 0;// nonblock => f/f 사용할때 일반적으로 
            r_clk <= 1'b0;
        end else begin
            if(r_counter == 100000000 - 1) begin
                r_counter <= 0;
                r_clk <= 1'b1;
            end else begin
                r_counter <= r_counter + 1;
                r_clk <= 1'b0;
            end
        end
    end    
endmodule

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