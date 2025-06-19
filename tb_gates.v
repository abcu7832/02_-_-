`timescale 1ns / 1ps

module tb_gates();

    reg a,b;//input
    wire y0, y1, y2, y3, y4, y5;//output

gates duts(
    //인스턴스화
    //duts: 이름
    //.? : port 이름
    .a(a),
    .b(b),
    .y0(y0),
    .y1(y1),
    .y2(y2),
    .y3(y3),
    .y4(y4),
    .y5(y5)
);

initial begin
    
    #0;
    a = 1'b0;
    b = 1'b0;
    #10;//delay 값 유지
    a = 1'b1;
    b = 1'b0;
    #10;
    a = 1'b0;
    b = 1'b1;
    #10;
    a = 1'b1;
    b = 1'b1;
    #10;
    
    $finish;
end

endmodule