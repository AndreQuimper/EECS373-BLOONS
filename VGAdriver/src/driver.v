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

    reg [25:0] lut_input;
wire lut_output;
reg[57:0] w_or;
assign lut_output = |w_or;
always @* begin
	w_or = 'd0;
	w_or[6'd1] = {13'd40, 13'd0} == lut_input;
	w_or[6'd2] = {13'd40, 13'd1} == lut_input;
	w_or[6'd3] = {13'd40, 13'd2} == lut_input;
	w_or[6'd4] = {13'd40, 13'd3} == lut_input;
	w_or[6'd5] = {13'd40, 13'd4} == lut_input;
	w_or[6'd6] = {13'd40, 13'd5} == lut_input;
	w_or[6'd7] = {13'd40, 13'd6} == lut_input;
	w_or[6'd8] = {13'd39, 13'd7} == lut_input;
	w_or[6'd9] = {13'd39, 13'd8} == lut_input;
	w_or[6'd10] = {13'd39, 13'd9} == lut_input;
	w_or[6'd11] = {13'd39, 13'd10} == lut_input;
	w_or[6'd12] = {13'd38, 13'd11} == lut_input;
	w_or[6'd13] = {13'd38, 13'd12} == lut_input;
	w_or[6'd14] = {13'd38, 13'd13} == lut_input;
	w_or[6'd15] = {13'd38, 13'd14} == lut_input;
	w_or[6'd16] = {13'd37, 13'd15} == lut_input;
	w_or[6'd17] = {13'd37, 13'd16} == lut_input;
	w_or[6'd18] = {13'd36, 13'd17} == lut_input;
	w_or[6'd19] = {13'd36, 13'd18} == lut_input;
	w_or[6'd20] = {13'd35, 13'd19} == lut_input;
	w_or[6'd21] = {13'd35, 13'd20} == lut_input;
	w_or[6'd22] = {13'd34, 13'd21} == lut_input;
	w_or[6'd23] = {13'd34, 13'd22} == lut_input;
	w_or[6'd24] = {13'd33, 13'd23} == lut_input;
	w_or[6'd25] = {13'd32, 13'd24} == lut_input;
	w_or[6'd26] = {13'd31, 13'd25} == lut_input;
	w_or[6'd27] = {13'd31, 13'd26} == lut_input;
	w_or[6'd28] = {13'd30, 13'd27} == lut_input;
	w_or[6'd29] = {13'd29, 13'd28} == lut_input;
	w_or[6'd30] = {13'd28, 13'd29} == lut_input;
	w_or[6'd31] = {13'd27, 13'd30} == lut_input;
	w_or[6'd32] = {13'd26, 13'd31} == lut_input;
	w_or[6'd33] = {13'd25, 13'd31} == lut_input;
	w_or[6'd34] = {13'd24, 13'd32} == lut_input;
	w_or[6'd35] = {13'd23, 13'd33} == lut_input;
	w_or[6'd36] = {13'd22, 13'd34} == lut_input;
	w_or[6'd37] = {13'd21, 13'd34} == lut_input;
	w_or[6'd38] = {13'd20, 13'd35} == lut_input;
	w_or[6'd39] = {13'd19, 13'd35} == lut_input;
	w_or[6'd40] = {13'd18, 13'd36} == lut_input;
	w_or[6'd41] = {13'd17, 13'd36} == lut_input;
	w_or[6'd42] = {13'd16, 13'd37} == lut_input;
	w_or[6'd43] = {13'd15, 13'd37} == lut_input;
	w_or[6'd44] = {13'd14, 13'd38} == lut_input;
	w_or[6'd45] = {13'd13, 13'd38} == lut_input;
	w_or[6'd46] = {13'd12, 13'd38} == lut_input;
	w_or[6'd47] = {13'd11, 13'd38} == lut_input;
	w_or[6'd48] = {13'd10, 13'd39} == lut_input;
	w_or[6'd49] = {13'd9, 13'd39} == lut_input;
	w_or[6'd50] = {13'd8, 13'd39} == lut_input;
	w_or[6'd51] = {13'd7, 13'd39} == lut_input;
	w_or[6'd52] = {13'd6, 13'd40} == lut_input;
	w_or[6'd53] = {13'd5, 13'd40} == lut_input;
	w_or[6'd54] = {13'd4, 13'd40} == lut_input;
	w_or[6'd55] = {13'd3, 13'd40} == lut_input;
	w_or[6'd56] = {13'd2, 13'd40} == lut_input;
	w_or[6'd57] = {13'd1, 13'd40} == lut_input;
	w_or[6'd0] = {13'd0, 13'd40} == lut_input;
end

    reg [13:0] dx, next_dx;
    reg [13:0] dy, next_dy;

    reg [12:0] x_center, y_center;
    wire [12:0]  next_x_center, next_y_center;

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
	reg [12:0] H_Counter, V_Counter, Next_H_Counter, Next_V_Counter;
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
        next_color = 3'b000;
        lut_input = 26'd0;
        if(h_state == H_ACTIVE_STATE && v_state == V_ACTIVE_STATE) begin
            lut_input = {{1'b0, dx[11:0]}, {1'b0, dy[11:0]}};
            if(lut_output) begin
                next_color[0] = ram[27:26] == 2'd1;
                next_color[1] = ram[27:26] == 2'd2 || ram[27:26] == 2'd1;
                next_color[2] = ram[27:26] == 2'd3;
            end
        end

    end

    assign r = color[2];
    assign g = color[1];
    assign b = color[0];

    always @* begin
        next_dx = (H_Counter > x_center) ? (H_Counter - x_center) : (x_center - H_Counter);
        next_dy = (V_Counter > y_center) ? (V_Counter - y_center) : (y_center - V_Counter);
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
		end
	end
endmodule
