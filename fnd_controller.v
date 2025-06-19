`timescale 1ns / 1ps

module fnd_controller (
    input clk,
    input reset,
    input [8:0] sum,
    output [7:0] fnd_data,
    output [3:0] fnd_com
);
    wire [3:0] w_bcd, w_digit_1, w_digit_10, w_digit_100, w_digit_1000;
    wire w_oclk;
    wire [1:0] fnd_sel;

    clk_div U_CLK_Div (
        .clk(clk),
        .reset(reset),
        .o_clk(w_oclk)
    );

    counter_4 U_Counter_4 (
        .clk(w_oclk),
        .reset(reset),
        .fnd_sel(fnd_sel)
    );

    decoder_2x4 U_DECODER_2x4 (
        .fnd_sel(fnd_sel),
        .fnd_com(fnd_com)
    );

    digit_splitter U_DS (
        .sum(sum),
        .digit_1(w_digit_1),
        .digit_10(w_digit_10),
        .digit_100(w_digit_100),
        .digit_1000(w_digit_1000)
    );

    mux_4x1 U_MUX_4x1 (
        .digit_1(w_digit_1),
        .digit_10(w_digit_10),
        .digit_100(w_digit_100),
        .digit_1000(w_digit_1000),
        .s(fnd_sel),
        .bcd(w_bcd)
    );

    bcd U_BCD (
        .bcd(w_bcd),
        .fnd_data(fnd_data)
    );
endmodule

module clk_div (
    input clk,
    input reset,
    output o_clk
);
    reg r_clk;
    reg [16:0] r_counter;
    //reg [$clog(100_000)-1:0] r_counter;
    assign o_clk = r_clk;

    always @(posedge clk, posedge reset) begin
        if(reset) begin
            r_counter <= 0;// nonblock => f/f 사용할때 일반적으로 
            r_clk <= 1'b0;
        end else begin
            if(r_counter == 100_000 - 1) begin
                r_counter <= 0;
                r_clk <= 1'b1;
            end else begin
                r_counter <= r_counter + 1;
                r_clk <= 1'b0;
            end
        end
    end    
endmodule

module counter_4 (
    input clk,
    input reset,
    output [1:0] fnd_sel
);
    reg [1:0] r_counter;
    assign fnd_sel = r_counter;
    
    always @(posedge clk, posedge reset) begin//,: OR 또는
        if(reset) begin
            r_counter <= 0;// nonblock => f/f 사용할때 일반적으로 
        end else begin
            r_counter <= r_counter + 1;
        end
    end

endmodule

module decoder_2x4 (
    input [1:0] fnd_sel,
    output reg [3:0] fnd_com
);
    always @(*) begin
        case (fnd_sel)
            2'b00: fnd_com = 4'b1110;  //1의자리만 실행
            2'b01: fnd_com = 4'b1101;  //10의자리만 실행
            2'b10: fnd_com = 4'b1011;  //100의자리만 실행 
            2'b11: fnd_com = 4'b0111;  //1000의자리만 실행
        endcase
    end
endmodule

module mux_4x1 (
    input [3:0] digit_1,
    input [3:0] digit_10,
    input [3:0] digit_100,
    input [3:0] digit_1000,
    input [1:0] s,
    output reg [3:0] bcd
);
    //4x1 mux
    //case: latch유발 방지 => default나 모든 경우를 선언해야함.
    always @(*) begin  //*: 모든 입력 감시
        case (s)
            2'b00: bcd = digit_1;
            2'b01: bcd = digit_10;
            2'b10: bcd = digit_100;
            2'b11: bcd = digit_1000;
        endcase
    end
endmodule

module digit_splitter (
    input  [8:0] sum,
    output [3:0] digit_1,
    output [3:0] digit_10,
    output [3:0] digit_100,
    output [3:0] digit_1000
);

    assign digit_1 = sum % 10;
    assign digit_10 = (sum / 10) % 10;
    assign digit_100 = (sum / 100) % 10;
    assign digit_1000 = (sum / 1000) % 10;
endmodule

module bcd (
    input [3:0] bcd,  //input은 항상 wire
    output reg [7:0] fnd_data//출력: always에서는 reg/ assign에서는 wire
);

    // 조합논리 combinational, 행위수준 모델링.
    always @(bcd) begin
        case (bcd)
            4'h00:   fnd_data = 8'hC0;
            4'h01:   fnd_data = 8'hF9;
            4'h02:   fnd_data = 8'hA4;
            4'h03:   fnd_data = 8'hB0;
            4'h04:   fnd_data = 8'h99;
            4'h05:   fnd_data = 8'h92;
            4'h06:   fnd_data = 8'h82;
            4'h07:   fnd_data = 8'hF8;
            4'h08:   fnd_data = 8'h80;
            4'h09:   fnd_data = 8'h90;
            default: fnd_data = 8'hff;
        endcase
    end

endmodule
