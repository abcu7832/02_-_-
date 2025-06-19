`timescale 1ns / 1ps

module calculator (
    input [7:0] a,
    input [7:0] b,
    input clk,
    output [7:0] fnd_data,
    output reg [3:0] fnd_com
);

    wire [7:0] w_sum;
    wire led;
    
    reg [1:0] s = 2'b00;
    integer count = 0;

    always @(posedge clk) begin
        count = count + 1;
        if(count == 10000) begin 
            count = 0; 
            if (s == 3) begin  s = 0; end
            else begin s = s + 1; end
            case (s)
                2'b00: fnd_com = 4'b1110;
                2'b01: fnd_com = 4'b1101;
                2'b10: fnd_com = 4'b1011;
                2'b11: fnd_com = 4'b0111;
            endcase
        end
    end

    fnd_controller U_FND_CNTR (
        .sum({led, w_sum}),
        .s(s),
        .fnd_data(fnd_data)
    );
    adder U_ADDER (
        .a(a),
        .b(b),
        .cin(1'b0),
        .s(w_sum),
        .cout(led)
    );
endmodule

// 8bit adder
module adder (
    input  [7:0] a,
    input  [7:0] b,
    input        cin,
    output [7:0] s,
    output       cout
);

    wire w_c0;

    full_adder_4bit U_FA4_H (
        .a(a[7:4]),
        .b(b[7:4]),
        .cin(w_c0),
        .s(s[7:4]),
        .cout(cout)
    );
    full_adder_4bit U_FA4_L (
        .a(a[3:0]),
        .b(b[3:0]),
        .cin(cin),
        .s(s[3:0]),
        .cout(w_c0)
    );
endmodule

module full_adder_4bit (
    input  [3:0] a,
    input  [3:0] b,
    input        cin,
    output [3:0] s,
    output       cout
);
    wire w_c0, w_c1, w_c2;

    full_adder U_FA3 (
        .a(a[3]),
        .b(b[3]),
        .cin(w_c2),
        .s(s[3]),
        .cout(cout)
    );
    full_adder U_FA2 (
        .a(a[2]),
        .b(b[2]),
        .cin(w_c1),
        .s(s[2]),
        .cout(w_c2)
    );
    full_adder U_FA1 (
        .a(a[1]),
        .b(b[1]),
        .cin(w_c0),
        .s(s[1]),
        .cout(w_c1)
    );
    full_adder U_FA0 (
        .a(a[0]),
        .b(b[0]),
        .cin(cin),
        .s(s[0]),
        .cout(w_c0)
    );
endmodule

module full_adder (
    input  a,
    input  b,
    input  cin,
    output s,
    output cout
);

    wire w_s, w_c1, w_c2;
    assign cout = w_c1 | w_c2;

    half_adder U_HA1 (
        .a(w_s),
        .b(cin),
        .s(s),
        .c(w_c2)
    );

    half_adder U_HA0 (
        .a(a),
        .b(b),
        .s(w_s),
        .c(w_c1)
    );
endmodule

module half_adder (
    input  a,
    input  b,
    output s,
    output c
);
    xor (s, a, b);
    and (c, a, b);

endmodule

module fnd_controller (
    input  [8:0] sum,
    input  [1:0] s,
    output [7:0] fnd_data
);

    wire [3:0] w_bcd, w_digit_1, w_digit_10, w_digit_100, w_digit_1000;

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
        .s(s),
        .bcd(w_bcd)
    );

    bcd U_BCD (
        .bcd(w_bcd),
        .fnd_data(fnd_data)
    );

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
            4'h00: fnd_data = 8'hC0;
            4'h01: fnd_data = 8'hF9;
            4'h02: fnd_data = 8'hA4;
            4'h03: fnd_data = 8'hB0;
            4'h04: fnd_data = 8'h99;
            4'h05: fnd_data = 8'h92;
            4'h06: fnd_data = 8'h82;
            4'h07: fnd_data = 8'hF8;
            4'h08: fnd_data = 8'h80;
            4'h09: fnd_data = 8'h90;
            default: fnd_data = 8'hff;
        endcase
    end

endmodule
