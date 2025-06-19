`timescale 1ns / 1ps

module gates(
    input a,
    input b,
    output y0,
    output y1,
    output y2,
    output y3,
    output y4,
    output y5

    );

    //gates
    //wire사용할때 assign
    assign y0 = a & b;//and
    assign y1 = a | b;//or
    assign y2 = ~(a & b);//nand
    assign y3 = ~(a | b);//nor
    assign y4 = a ^ b;//xor
    assign y5 = ~a;                    
    
endmodule
//C:\Users\kccistc\AppData\Local\Programs\Microsoft VS Code.exe
//https://digilent.com/reference/programmable-logic/basys-3/reference-manual