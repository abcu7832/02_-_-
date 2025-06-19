`timescale 1ns / 1ps

module tb_fsm();
    reg clk, reset;
    reg [2:0] sw;
    wire [2:0] led;

    fsm U_FSM (
        .clk(clk),
        .reset(reset),
        .sw(sw),
        .led(led)
    );
    
    always #5 clk = ~clk;

    integer i;

    initial begin
        #0; clk = 1'b0; reset = 1'b1;
        #20; reset = 1'b0;
        for(i=0;i<12;i=i+1) begin
                case (i)
                    0: sw = 3'b000;
                    1: sw = 3'b001;
                    2: sw = 3'b010;
                    3: sw = 3'b011;
                    4: sw = 3'b100;
                    5: sw = 3'b101;
                    6: sw = 3'b110;
                    7: sw = 3'b001;
                    8: sw = 3'b100;
                    9: sw = 3'b101;
                    10: sw = 3'b110;
                    11: sw = 3'b111;
                endcase
            #10;
        end
        #10000
        $finish;
    end
endmodule
