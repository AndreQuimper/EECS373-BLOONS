module driver(
	input clk,
	input reset,
//    input x_0,
//    input x_1,
//    input x_2,
//    input x_3,
//    input x_4,
//    input x_5,
//    input x_6,
//    input x_7,
//    input x_8,
//    input x_9,
//    input x_10,
//    input x_11,
	output r,
	output g,
	output b,
	output reg h_sync,
	output reg v_sync
);

    wire NE1; ////true for addresses 0x60000000 to 0x63FFFFFF
    wire NWE; //write/read enable
    wire [3:0] ADDR; //address bus
    wire [7:0] DATA; //bi-directional read/write data bus
    wire NWAIT;
    wire [$clog2(4):0] count;
    wire [4*3*8-1:0] ram;

    coord_list inst(
    .reset(reset),
    .NE1(NE1),
    .NWE(NWE), 
    .ADDR(ADDR),
    .DATA(DATA), 
    .NWAIT(NWAIT),
    .count(count),
    .ram(ram)
    );


    reg signed [13:0] dx, next_dx;
    reg signed [13:0] dy, next_dy;
    reg signed [24:0] dx_sq, next_dx_sq, dy_sq,next_dy_sq;

    reg signed [12:0] x_center;
    wire signed [12:0]  next_x_center;
    parameter signed [12:0] y_center = 13'd320;
    parameter signed [13:0] radius   = 13'd40;
    parameter signed [13:0] radius_sq   = 13'd1600;

    assign next_x_center = {5'b0, ram[15:8]};
	
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
	reg signed [12:0] X_Counter, V_Counter, Next_X_Counter, Next_V_Counter;
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

    always @* begin
        next_color = 3'd0;
        if(h_state == H_ACTIVE_STATE && v_state == V_ACTIVE_STATE) begin
            next_color = ((dx_sq+dy_sq <= radius_sq && dx_sq+dy_sq >= radius_sq-(radius*2)) || dx_sq+dy_sq <= radius) ? 3'd1 : 3'd0;
        end
    end

    assign r = color[2];
    assign g = color[1];
    assign b = color[0];

    always @* begin
//        next_x_center[0] = x_0;
//        next_x_center[1] = x_1;
//        next_x_center[2] = x_2;
//        next_x_center[3] = x_3;
//        next_x_center[4] = x_4;
//        next_x_center[5] = x_5;
//        next_x_center[6] = x_6;
//        next_x_center[7] = x_7;
//        next_x_center[8] = x_8;
//        next_x_center[9] = x_9;
//        next_x_center[10] = x_10;
//        next_x_center[11] = x_11;
//        next_x_center[12] = 0;
        next_dx = (X_Counter > x_center) ? (X_Counter - x_center) : (x_center - X_Counter);
        next_dy = (V_Counter > y_center) ? (V_Counter - y_center) : (y_center - V_Counter);
        next_dx_sq = (dx*dx);
        next_dy_sq = (dy*dy);
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
            x_center <= 13'd0;
            dx <= 14'd0;
            dy <= 14'd0;
            dx_sq <= 25'd0;
            dy_sq <= 25'd0;
//          x_center <= 13'd600;
		end else begin 
			X_Counter <= Next_X_Counter;
			V_Counter <= Next_V_Counter;
			h_state <= next_h_state;
			v_state <= next_v_state;
			h_sync <= next_h_sync;
			v_sync <= next_v_sync;
            line_done <= next_line_done;
            color <= next_color;
            x_center <= next_x_center;
            dx <= next_dx;
            dy <= next_dy;
            dx_sq <= next_dx_sq;
            dy_sq <= next_dy_sq;
		end
	end
endmodule
