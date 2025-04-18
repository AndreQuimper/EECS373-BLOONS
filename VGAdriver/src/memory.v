module coord_list(
    input NE1, ////true for addresses 0x60000000 to 0x63FFFFFF
    input NWE, //write/read enable
    input [2:0] ADDR, //address bus
    input [7:0] DATA, //bi-directional read/write data bus
    output reg [27:0] ram
);

    parameter [2:0] COLOR_ADDR = 'd1;
    parameter [2:0] XCOORD_ADDR_LOWER = 'd2;
    parameter [2:0] XCOORD_ADDR_UPPER = 'd3;
    parameter [2:0] YCOORD_ADDR_LOWER = 'd4;
    parameter [2:0] YCOORD_ADDR_UPPER = 'd5;

    reg [27:0] n_ram;

    wire d_latch_enable;
    assign d_latch_enable = ~NE1 & ~NWE;
   
    always @* begin
        n_ram = ram;
        case(ADDR)
        XCOORD_ADDR_LOWER : begin
            n_ram[0] = DATA[0];
            n_ram[1] = DATA[1];
            n_ram[2] = DATA[2];
            n_ram[3] = DATA[3];
            n_ram[4] = DATA[4];
            n_ram[5] = DATA[5];
            n_ram[6] = DATA[6];
            n_ram[7] = DATA[7];
        end
        XCOORD_ADDR_UPPER : begin
            n_ram[8] = DATA[0];
            n_ram[9] = DATA[1];
            n_ram[10] = DATA[2];
            n_ram[11] = DATA[3];
            n_ram[12] = DATA[4];
        end
        YCOORD_ADDR_LOWER : begin
            n_ram[13+0] = DATA[0];
            n_ram[13+1] = DATA[1];
            n_ram[13+2] = DATA[2];
            n_ram[13+3] = DATA[3];
            n_ram[13+4] = DATA[4];
            n_ram[13+5] = DATA[5];
            n_ram[13+6] = DATA[6];
            n_ram[13+7] = DATA[7];
        end
        YCOORD_ADDR_UPPER : begin
            n_ram[13+8] = DATA[0];
            n_ram[13+9] = DATA[1];
            n_ram[13+10] = DATA[2];
            n_ram[13+11] = DATA[3];
            n_ram[13+12] = DATA[4];
        end
        COLOR_ADDR : begin
            n_ram[26] = DATA[0];
            n_ram[27] = DATA[1];
        end
        endcase
    end

    always@(posedge d_latch_enable) begin
       begin
            ram <= n_ram;
        end
    end
endmodule