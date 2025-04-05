module driver(
	input clk,
	input reset,
	output r,
	output g,
	output b,
	output reg h_sync,
	output reg v_sync
);
	// Use the PLL to generate a 25MHz clock
	
	parameter [9:0] H_ACTIVE = 10'd639;
	parameter [9:0] H_FRONT = 10'd15;
	parameter [9:0] H_PULSE = 10'd95;
	parameter [9:0] H_BACK = 10'd47;
	parameter [9:0] V_ACTIVE = 10'd479;
	parameter [9:0] V_FRONT = 10'd9;
	parameter [9:0] V_PULSE = 10'd1;
	parameter [9:0] V_BACK = 10'd32;
	parameter [1:0] H_ACTIVE_STATE = 2'd0;
	parameter [1:0] H_FRONT_STATE = 2'd1;
	parameter [1:0] H_PULSE_STATE = 2'd2;
	parameter [1:0] H_BACK_STATE = 2'd3;
	
	parameter [1:0] V_ACTIVE_STATE = 2'd0;
	parameter [1:0] V_FRONT_STATE = 2'd1;
	parameter [1:0] V_PULSE_STATE = 2'd2;
	parameter [1:0] V_BACK_STATE = 2'd3;
	reg [1:0] h_state, v_state, next_h_state, next_v_state;
	reg [9:0] X_Counter, V_Counter, Next_X_Counter, Next_V_Counter;
	reg next_h_sync, next_v_sync, line_done, next_line_done;
    reg [2:0] color, next_color;
	always @* begin
		Next_X_Counter = X_Counter;
		Next_V_Counter = V_Counter;
		next_h_state = h_state;
		next_v_state = v_state;
		next_h_sync = h_sync;
		next_v_sync = v_sync;
        next_line_done   = line_done;
		
		if (h_state == H_ACTIVE_STATE) begin
			Next_X_Counter = (X_Counter == H_ACTIVE) ? 10'd0 : (X_Counter + 10'd1);
			next_h_sync = 1;
			next_line_done = 0;
			next_h_state = (X_Counter == H_ACTIVE) ? H_FRONT_STATE : H_ACTIVE_STATE;
            next_color = (((X_Counter > 320 && X_Counter < 330) | (X_Counter > 340 && X_Counter < 350)) && ((V_Counter > 235 && V_Counter < 230) | (V_Counter > 245 && V_Counter < 255))) ? 1 : 0;
		end else if (h_state == H_FRONT_STATE) begin
			Next_X_Counter = (X_Counter == H_FRONT) ? 10'd0 : (X_Counter + 10'd1);
			next_h_sync = 1;
			next_h_state = (X_Counter == H_FRONT) ? H_PULSE_STATE : H_FRONT_STATE;
		end else if (h_state == H_PULSE_STATE) begin
			Next_X_Counter = (X_Counter == H_PULSE) ? 10'd0 : (X_Counter + 10'd1);
			next_h_sync = 0;
			next_h_state = (X_Counter == H_PULSE) ? H_BACK_STATE : H_PULSE_STATE;
		end else if (h_state == H_BACK_STATE) begin
			Next_X_Counter = (X_Counter == H_BACK) ? 10'd0 : (X_Counter + 10'd1);
			next_h_sync = 1;
			next_h_state = (X_Counter == H_BACK) ? H_ACTIVE_STATE : H_BACK_STATE;
			next_line_done = (X_Counter == (H_BACK - 1)) ? 1 : 0;
		end
		if (v_state == V_ACTIVE_STATE) begin
			Next_V_Counter = (line_done) ? ((V_Counter == V_ACTIVE) ? 10'd0 : (V_Counter + 10'd1)) : V_Counter;
			next_v_sync = 1;
			next_v_state = (line_done) ? ((V_Counter == V_ACTIVE) ? V_FRONT_STATE : V_ACTIVE_STATE) : V_ACTIVE_STATE;
		end else if (v_state == V_FRONT_STATE) begin
			Next_V_Counter = (line_done) ? ((V_Counter == V_FRONT) ? 10'd0 : (V_Counter + 10'd1)) : V_Counter;
			next_v_sync = 1;
			next_v_state = (line_done) ? ((V_Counter == V_FRONT) ? V_PULSE_STATE : V_FRONT_STATE) : V_FRONT_STATE;
		end else if (v_state == V_PULSE_STATE) begin
			Next_V_Counter = (line_done) ? ((V_Counter == V_PULSE) ? 10'd0 : (V_Counter + 10'd1)) : V_Counter;
			next_v_sync = 0;
			next_v_state = (line_done) ? ((V_Counter == V_PULSE) ? V_BACK_STATE : V_PULSE_STATE) : V_PULSE_STATE;
		end else if (v_state == V_BACK_STATE) begin
			Next_V_Counter = (line_done) ? ((V_Counter == V_BACK) ? 10'd0 : (V_Counter + 10'd1)) : V_Counter;
			next_v_sync = 1;
			next_v_state = (line_done) ? ((V_Counter == V_BACK) ? V_ACTIVE_STATE : V_BACK_STATE) : V_BACK_STATE;
		end
	end
	always @(posedge clk) begin
		if (~reset) begin
			X_Counter <= 10'd0;
			V_Counter <= 10'd0;
			h_state <= H_ACTIVE_STATE;
			v_state <= V_ACTIVE_STATE;
			h_sync <= 1;
			v_sync <= 1;
            line_done <= 0;
            color <= 0;
		end else begin 
			X_Counter <= Next_X_Counter;
			V_Counter <= Next_V_Counter;
			h_state <= next_h_state;
			v_state <= next_v_state;
			h_sync <= next_h_sync;
			v_sync <= next_v_sync;
            line_done <= next_line_done;
            color <= next_color;
		end
	end
    assign r = (h_state == H_ACTIVE_STATE && v_state == V_ACTIVE_STATE) ? color[0] : 0;
    assign g = (h_state == H_ACTIVE_STATE && v_state == V_ACTIVE_STATE) ? color[1] : 0;
    assign b = (h_state == H_ACTIVE_STATE && v_state == V_ACTIVE_STATE) ? color[2] : 0;
//    assign r = (h_state == H_ACTIVE_STATE && v_state == V_ACTIVE_STATE) ? (V_Counter[3] | X_Counter==256) : 0;
//    assign g = (h_state == H_ACTIVE_STATE && v_state == V_ACTIVE_STATE) ? (X_Counter[5] ^ X_Counter[6]) | (X_Counter==256) : 0;
//    assign b = (h_state == H_ACTIVE_STATE && v_state == V_ACTIVE_STATE) ? (X_Counter[4] | X_Counter==256) : 0;
endmodule
