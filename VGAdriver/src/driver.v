module driver(
	input clk,
	input NE1, ////true for addresses 0x60000000 to 0x63FFFFFF
	input NWE, //write/read enable
	input [2:0] ADDR, //address bus
	input [7:0] DATA, //bi-directional read/write data bus
	output r,
	output g,
	output b,
	output reg h_sync,
	output reg v_sync
);

    wire [27:0] ram;

    coord_list inst(
    .NE1(NE1),
    .NWE(NWE), 
    .ADDR(ADDR),
    .DATA(DATA), 
    .ram(ram)
    );


    reg signed [13:0] dx, next_dx;
    reg signed [13:0] dy, next_dy;
    reg signed [24:0] dx_sq, next_dx_sq, dy_sq,next_dy_sq;

    reg signed [12:0] x_center, y_center;
    wire signed [12:0]  next_x_center, next_y_center;
    parameter signed [12:0] radius   = 13'd40;
    parameter signed [12:0] radius_sq   = 13'd1600;

    assign next_x_center = ram[12:0];
    assign next_y_center = ram[24:13];
	
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
	reg signed [12:0] H_Counter, V_Counter, Next_H_Counter, Next_V_Counter;
	reg next_h_sync, next_v_sync, line_done, next_line_done;
    reg [2:0] color, next_color;
	always @* begin
		Next_H_Counter = H_Counter;
		Next_V_Counter = V_Counter;
		next_h_state = h_state;
		next_v_state = v_state;
		next_h_sync = h_sync;
		next_v_sync = v_sync;
        next_line_done   = line_done;
		if (h_state == H_ACTIVE_STATE) begin
			Next_H_Counter = (H_Counter == H_ACTIVE) ? 10'd0 : (H_Counter + 10'd1);
			next_h_sync = 1;
			next_line_done = 0;
			next_h_state = (H_Counter == H_ACTIVE) ? H_FRONT_STATE : H_ACTIVE_STATE;
		end else if (h_state == H_FRONT_STATE) begin
			Next_H_Counter = (H_Counter == H_FRONT) ? 10'd0 : (H_Counter + 10'd1);
			next_h_sync = 1;
			next_h_state = (H_Counter == H_FRONT) ? H_PULSE_STATE : H_FRONT_STATE;
		end else if (h_state == H_PULSE_STATE) begin
			Next_H_Counter = (H_Counter == H_PULSE) ? 10'd0 : (H_Counter + 10'd1);
			next_h_sync = 0;
			next_h_state = (H_Counter == H_PULSE) ? H_BACK_STATE : H_PULSE_STATE;
		end else if (h_state == H_BACK_STATE) begin
			Next_H_Counter = (H_Counter == H_BACK) ? 10'd0 : (H_Counter + 10'd1);
			next_h_sync = 1;
			next_h_state = (H_Counter == H_BACK) ? H_ACTIVE_STATE : H_BACK_STATE;
			next_line_done = (H_Counter == (H_BACK - 1)) ? 1 : 0;
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
        
        if(h_state == H_ACTIVE_STATE && v_state == V_ACTIVE_STATE 
        && ((dx_sq+dy_sq <= radius_sq && dx_sq+dy_sq >= radius_sq-{radius, 1'b0}) || dx_sq+dy_sq <= radius)) begin
            next_color[0] = ram[27:26] == 2'd1;
            next_color[1] = ram[27:26] == 2'd2;
            next_color[2] = ram[27:26] == 2'd3;
        end else begin
            next_color = 'b0;
        end
    end

    assign r = color[2];
    assign g = color[1];
    assign b = color[0];

    always @* begin
        next_dx = (H_Counter > x_center) ? (H_Counter - x_center) : (x_center - H_Counter);
        next_dy = (V_Counter > y_center) ? (V_Counter - y_center) : (y_center - V_Counter);
        next_dx_sq = (dx*dx);
        next_dy_sq = (dy*dy);
    end

	always @(posedge clk) begin
		begin 
			H_Counter <= Next_H_Counter;
			V_Counter <= Next_V_Counter;
			h_state <= next_h_state;
			v_state <= next_v_state;
			h_sync <= next_h_sync;
			v_sync <= next_v_sync;
            line_done <= next_line_done;
            color <= next_color;
            x_center <= next_x_center;
            y_center <= next_y_center;
            dx <= next_dx;
            dy <= next_dy;
            dx_sq <= next_dx_sq;
            dy_sq <= next_dy_sq;
		end
	end
endmodule
