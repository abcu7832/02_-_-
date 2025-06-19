`timescale 1ns / 1ps

module fsm (
    input clk,
    input reset,
    input [2:0] sw,
    output reg [2:0] led
);

    // 상태 정의
    parameter [2:0] IDLE = 3'b000, STATE1 = 3'b001, STATE2 = 3'b010, STATE3 = 3'b011, STATE4 = 3'b100, STATE5 = 3'b111;
    reg [2:0] c_state, n_state;

    // state register
    always @(posedge clk, posedge reset) begin
        if(reset) begin
            c_state <= 3'b000;       
        end else begin
            c_state <= n_state;
        end
    end

    // next state combinational logic
    // combinational logic은 = 사용
    always @(*) begin
        n_state = c_state;
        case (c_state)
            IDLE: begin 
                // 입력조건에 따라 next state를 처리
                if(sw == 3'b001) begin
                    n_state = STATE1;
                end                  
                else if(sw == 3'b100)begin
                    n_state = STATE3;
                end 
            end 
            STATE1: begin
                if(sw == 3'b010) begin
                    n_state = STATE2;
                end
                else if(sw == 3'b100) begin
                    n_state = STATE4;
                end
            end
            STATE2: begin
                if(sw == 3'b011) begin
                    n_state = STATE3;
                end
            end
            STATE3: begin
                if(sw == 3'b100) begin
                    n_state = STATE4;
                end
            end
            STATE4: begin
                if(sw == 3'b101) begin
                    n_state = STATE5;
                end
            end
            STATE5: begin
                if(sw == 3'b110) begin
                    n_state = IDLE;
                end
            end
        endcase
    end

    // output Combinational Logic
    always @(*) begin
        led = 3'b000;
        case (c_state)
            IDLE: begin 
                // 입력조건에 따라 next state를 처리
                led = IDLE;              
            end 
            STATE1: begin
                led = STATE1;
            end
            STATE2: begin
                led = STATE2;

            end
            STATE3: begin
                led = STATE3;
            end
            STATE4: begin
                led = STATE4;
            end
            STATE5: begin
                led = STATE5;
            end
        endcase
    end
    // assign led = (c_state == STOP)? 2'b10:2'b01;
endmodule
