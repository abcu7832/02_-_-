`timescale 1ns / 1ps

/*
module block();
    reg clk, a,b;

    initial begin
        a=0; b=1; clk=0;
    end
    always clk = #5 ~clk;
    always @(posedge clk) begin
        a=b;//a=1
        b=a;//b=a=1
    end
endmodule

module non_blk ();
    reg clk, a,b;
    initial begin
        a=0;b=1;clk=0;
    end

    always clk = #5 ~clk;
    always @(posedge clk) begin
        a<=b;
        b<=a;
    end
endmodule
*/
/*module mux21_if (
    input [1:0] a, 
    input [1:0] b, 
    input [1:0] c, 
    input [1:0] d, 
    input [1:0] sel, 
    output [1:0] out
);

    always @(*) begin
        case (sel)
            0 : out = a;
            1 : out = b;
            2 : out = c;
            3 : out = d;
        endcase
    end
    
    
    always @(*) begin
        if(sel == 0) out = a;
        else if(sel == 1) out = b;
        else if(sel == 2) out = c;
        else out = d;
    end
    assign out = (sel == 0) ? a : ((sel == 1) ? b : ((sel == 2) ? c : d));
endmodule*/

module mealy_fsm (
    input clk, input reset, input din_bit,
    output dout_bit
);
    reg [2:0] state_reg, next_state;

    parameter start = 3'b000;
    parameter rd0_once = 3'b001;
    parameter rd1_once = 3'b010;
    parameter rd0_twice = 3'b011;
    parameter rd1_twice = 3'b100;

    always @(state_reg, din_bit) begin
        case (state_reg)
            start:  if (din_bit == 0) begin next_state=rd0_once; end 
                    else if(din_bit) begin next_state = rd1_once; end 
                    else begin next_state=start; end
            rd0_once: if (din_bit == 0) begin next_state=rd0_twice; end 
                    else if(din_bit) begin next_state = rd1_once; end 
                    else begin next_state=start; end
            rd0_twice:if (din_bit == 0) begin next_state=rd0_twice; end 
                    else if(din_bit) begin next_state = rd1_once; end 
                    else begin next_state=start; end
            rd1_once:if (din_bit == 0) begin next_state=rd0_once; end 
                    else if(din_bit) begin next_state = rd1_twice; end 
                    else begin next_state=start; end
            rd1_twice:if (din_bit == 0) begin next_state=rd0_once; end 
                    else if(din_bit) begin next_state = rd1_twice; end 
                    else begin next_state=start; end
            default: next_state=start;
        endcase
    end

    always @(posedge clk, posedge reset) begin
        if(reset) state_reg <= start;
        else state_reg<=next_state;
    end

    assign dout_bit = ((state_reg==rd0_twice)||(state_reg==rd1_twice))?1:0;
    //assign dout_bit = (((state_reg==rd0_twice)&&(din_bit==0))||((state_reg==rd1_twice)&&(din_bit==1)))?1:0;
endmodule

module seq_detector_1010 (
    input clk, input reset, input din_bit,
    output dout_bit
);
    reg [3:0] state_reg, next_state;

    parameter A = 1;
    parameter B = 2;
    parameter C = 3;
    parameter D = 4;

    always @(state_reg, din_bit) begin
        case (state_reg)
            A:if (din_bit == 1) begin next_state=B; end 
                    else if(din_bit) begin next_state = A; end 
            B:if (din_bit == 0) begin next_state=C; end 
                    else if(din_bit) begin next_state = A; end 
            C:if (din_bit == 1) begin next_state=D; end 
                    else if(din_bit) begin next_state = A; end 
            D:if (din_bit == 0) begin next_state=A; end 
                    else if(din_bit) begin next_state = B; end 
            default: next_state=A;
        endcase
    end

    always @(posedge clk, posedge reset) begin
        if(reset) state_reg <= A;
        else state_reg<=next_state;
    end

    assign dout_bit = (((state_reg==D)&&(din_bit==0)))?1:0;
endmodule