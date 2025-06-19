`timescale 1ns / 1ps

module stopwatch_cu(
    input  clk,
    input  reset,
    input  i_clear,//BtnR
    input  i_runstop,//BtnL
    input  sw,//sw[0]
    output o_clear,
    output o_runstop,
    output o_mode
);
    reg[1:0] state_reg, next_state;

    parameter STOP = 1;
    parameter RUN = 2;
    parameter CLEAR = 3;

    assign o_clear = (state_reg == CLEAR)?1:0;
    assign o_runstop = (state_reg == RUN)?1:0;
    assign o_mode = (sw)?1:0;
    
    always @(posedge clk, posedge reset) begin
        if(reset) begin state_reg <= STOP; end
        else begin state_reg <= next_state; end
    end

    always @(*) begin
        next_state = state_reg;
        case (state_reg)
            STOP:
                if (i_clear) begin
                    next_state = CLEAR;
                end else if (i_runstop) begin
                    next_state = RUN;
                end else begin
                    next_state = state_reg;
                end
            RUN:
                if (i_runstop) begin
                    next_state = STOP;
                end else begin
                    next_state = state_reg;
                end
            CLEAR:
                if (i_clear) begin
                    next_state = STOP;
                end else begin
                    next_state = state_reg;
                end
        endcase
    end
endmodule