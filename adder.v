`timescale 1ns / 1ps

module fb_adder(
    input a3,
    input a2,
    input a1,
    input a0,
    input b3,
    input b2,
    input b1,
    input b0,
    input cin,
    output s3,
    output s2,
    output s1,
    output s0,
    output cout
);

wire c_in0, c_in1, c_in2;

full_adder U_FA3(
    .a(a3),
    .b(b3),
    .cin(c_in2),
    .s(s3),
    .cout(cout)
);

full_adder U_FA2(
    .a(a2),
    .b(b2),
    .cin(c_in1),
    .s(s2),
    .cout(c_in2)
);

full_adder U_FA1(
    .a(a1),
    .b(b1),
    .cin(c_in0),
    .s(s1),
    .cout(c_in1)
);

full_adder U_FA0(
    .a(a0),
    .b(b0),
    .cin(cin),
    .s(s0),
    .cout(c_in0)
);
endmodule

// full adder
module full_adder(
    input a,
    input b,
    input cin,
    output s,
    output cout
);

wire w_ha0_s, w_c0, w_c1;
assign cout = w_c0 | w_c1;

// 인스턴스화 
half_adder U_HA1(
    .a(w_ha0_s),
    .b(cin),
    .s(s),
    .c(w_c1)
);

half_adder U_HA0(
    .a(a),
    .b(b),
    .s(w_ha0_s),
    .c(w_c0)
);

endmodule

module half_adder(
    input a,
    input b,
    output s,
    output c
);
    assign s = a^b;
    assign c = a&b;

endmodule
