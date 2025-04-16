module coord_list(
    input reset,
    input NE1, ////true for addresses 0x60000000 to 0x63FFFFFF
    input NWE, //write/read enable
    input [3:0] ADDR, //address bus
    input [7:0] DATA, //bi-directional read/write data bus
    output NWAIT,
    output reg [$clog2(4):0] count,
    output reg [4*3*8-1:0] ram
);

    parameter [3:0] CNT_ADDR = 'd0;
    parameter [$clog2(4):0] POS_ADDR = 'd1;
    parameter [3:0] COLOR_ADDR = 'd2;
    parameter [3:0] XCOORD_ADDR = 'd3;
    parameter [3:0] YCOORD_ADDR = 'd4;

    reg [$clog2(4):0] current_pos, n_current_pos;
    reg [4*3*8-1:0] n_ram;
    reg [$clog2(4):0] n_count;

    wire d_latch_enable;
    assign d_latch_enable = ~NE1 & ~NWE;
    assign NWAIT = 1;
   
    reg [$clog2(4):0] i;
    always @* begin
        for(i = 0; i < 4; i = i + 1) begin : bruh
            n_ram = ram;
            n_count = count;
            n_current_pos = current_pos;
            if(i == current_pos) begin
                
                case(ADDR)
                CNT_ADDR : begin
                    n_count[$clog2(4):0] = DATA[$clog2(4):0];
                    n_ram = 0;
                end
                POS_ADDR : begin
                    n_current_pos = DATA[2:0];
                end
                COLOR_ADDR : begin
                    n_ram[i*3*8] = DATA[0];
                    n_ram[i*3*8 + 1] = DATA[1];
                    n_ram[i*3*8 + 2] = DATA[2];
                    n_ram[i*3*8 + 3] = DATA[3];
                    n_ram[i*3*8 + 4] = DATA[4];
                    n_ram[i*3*8 + 5] = DATA[5];
                    n_ram[i*3*8 + 6] = DATA[6];
                    n_ram[i*3*8 + 7] = DATA[7];
                    n_ram[i*3*8+7] = 1;
                end
                XCOORD_ADDR : begin
                    n_ram[(i*3*8) + 8] = DATA[0];
                    n_ram[(i*3*8) + 8 + 1] = DATA[1];
                    n_ram[(i*3*8) + 8 + 2] = DATA[2];
                    n_ram[(i*3*8) + 8 + 3] = DATA[3];
                    n_ram[(i*3*8) + 8 + 4] = DATA[4];
                    n_ram[(i*3*8) + 8 + 5] = DATA[5];
                    n_ram[(i*3*8) + 8 + 6] = DATA[6];
                    n_ram[(i*3*8) + 8 + 7] = DATA[7];
                end
                YCOORD_ADDR : begin
                    n_ram[(i*3*8) + 2*8] = DATA[0];
                    n_ram[(i*3*8) + 2*8 + 1] = DATA[1];
                    n_ram[(i*3*8) + 2*8 + 2] = DATA[2];
                    n_ram[(i*3*8) + 2*8 + 3] = DATA[3];
                    n_ram[(i*3*8) + 2*8 + 4] = DATA[4];
                    n_ram[(i*3*8) + 2*8 + 5] = DATA[5];
                    n_ram[(i*3*8) + 2*8 + 6] = DATA[6];
                    n_ram[(i*3*8) + 2*8 + 7] = DATA[7];
                end
            endcase
            end
        end
    end

    always@(posedge d_latch_enable) begin
        if(reset) begin
            ram <= 0;
            current_pos <= 0;
            count <= 0;
        end else begin
            ram <= n_ram;
            current_pos <= n_current_pos;
            count <= n_count;
        end
    end
endmodule