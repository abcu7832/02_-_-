`timescale 1ns / 1ps

//시뮬레이션 환경 모듈.
//내가 설계한 모듈을 test하기 위해서.

module tb_adder ();
    // 테스트 할 모듈의 input은 reg로 입력
    // 출력은 wire.
    reg [7:0] a, b;
    reg cin;
    wire [7:0] s;
    wire cout;

    adder dut (  //dut:design under test, cut, cct
        .a(a[7:0]),
        .b(b[7:0]),
        .cin(cin),
        .s(s[7:0]),
        .cout(cout)
    );

    integer i, j;

    initial begin
        #0; a = 8'h00; b = 8'h00; cin = 0;
        for (i = 0; i <= 10; i = i + 1) begin
            for (j = 0; j <= 10; j = j + 1) begin
                a = j;
                b = i;
                #1;
            end
        end
        $finish;
        //$stop;
    end
endmodule
