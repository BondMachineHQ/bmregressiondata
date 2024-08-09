
`timescale 1ns/1ps
module a0(clock_signal, reset_signal, i0, i0_valid , i0_received, o0, o0_valid, o0_received, vtm0din, vtm0addr, vtm0wren, vtm0en);

	input clock_signal;
	input reset_signal;

	input [15:0] i0;
	input i0_valid;
	output i0_received;
	output [15:0] o0;
	output o0_valid;
	input o0_received;

	output [15:0] vtm0din;
	output [15:0] vtm0addr;
	output vtm0wren;
	output vtm0en;

	wire [6:0] rom_bus;
	wire [15:0] rom_value;

	wire [15:0] a0din;
	wire [15:0] a0dout;
	wire [3:0] a0addr;
	wire a0wren;
	wire a0en;

	p0 p0_instance(clock_signal, reset_signal, rom_bus, rom_value, a0din, a0dout, a0addr, a0wren, a0en, i0, i0_valid , i0_received, o0, o0_valid, o0_received, vtm0din, vtm0addr, vtm0wren, vtm0en);
	p0rom p0rom_instance(rom_bus, rom_value);
	p0ram p0ram_instance(clock_signal, reset_signal, a0din, a0dout, a0addr, a0wren, a0en);

endmodule

`timescale 1ns / 1ps

module debouncer(
    input clk,
    input I,
    output reg O
    );
    parameter COUNT_MAX=255, COUNT_WIDTH=8;
    reg [COUNT_WIDTH-1:0] count;
    reg Iv=0;
    always@(posedge clk)
        if (I == Iv) begin
            if (count == COUNT_MAX)
                O <= I;
            else
                count <= count + 1'b1;
        end else begin
            count <= 'b0;
            Iv <= I;
        end
endmodule
	
`timescale 1ns / 1ps

module PS2Receiver(
	input clk,
	input kclk,
	input kdata,
	output reg [15:0] keycode=0,
	output reg oflag
	);
		
	wire kclkf, kdataf;
	reg [7:0]datacur=0;
	reg [7:0]dataprev=0;
	reg [3:0]cnt=0;
	reg flag=0;
		
	debouncer #(
		.COUNT_MAX(19),
		.COUNT_WIDTH(5)
	) db_clk(
		.clk(clk),
		.I(kclk),
		.O(kclkf)
	);
	debouncer #(
	   .COUNT_MAX(19),
	   .COUNT_WIDTH(5)
	) db_data(
		.clk(clk),
		.I(kdata),
		.O(kdataf)
	);
		
	always@(negedge(kclkf))begin
		case(cnt)
		0:;//Start bit
		1:datacur[0]<=kdataf;
		2:datacur[1]<=kdataf;
		3:datacur[2]<=kdataf;
		4:datacur[3]<=kdataf;
		5:datacur[4]<=kdataf;
		6:datacur[5]<=kdataf;
		7:datacur[6]<=kdataf;
		8:datacur[7]<=kdataf;
		9:flag<=1'b1;
		10:flag<=1'b0;
		
		endcase
			if(cnt<=9) cnt<=cnt+1;
			else if(cnt==10) cnt<=0;
	end
	
	reg pflag;
	always@(posedge clk) begin
		if (flag == 1'b1 && pflag == 1'b0) begin
			keycode <= {dataprev, datacur};
			oflag <= 1'b1;
			dataprev <= datacur;
		end else
			oflag <= 'b0;
		pflag <= flag;
	end
	
endmodule
	
`timescale 1ns / 1ps

	module bondkeydrv(
		input       clk,
		input       PS2Data,
		input       PS2Clk,
		output reg  [15:0] key,
		output reg  keycode_valid,
		input       keycode_recv
	);
	
		reg         start=0;
		reg         CLK50MHZ=0;
		wire [15:0] keycode;
		reg [15:0] keycodev;
		wire        flag;
		reg         cn=0;
	
		reg [5:0]	mods; // 0 Shift - 1 Ctrl - 3 Alt

		always @(posedge(clk))begin
			CLK50MHZ<=~CLK50MHZ;
		end
		
		PS2Receiver uut (
			.clk(CLK50MHZ),
			.kclk(PS2Clk),
			.kdata(PS2Data),
			.keycode(keycode),
			.oflag(flag)
		);
		
		always@(keycode)
			if (keycode[7:0] == 8'hf0) begin
				cn <= 1'b0;
			end else if (keycode[15:8] == 8'hf0) begin
				cn <= keycode != keycodev;
			end else begin
				cn <= keycode[7:0] != keycodev[7:0] || keycodev[15:8] == 8'hf0;
			end
		
		always @(posedge clk)
			if (flag == 1'b1 && cn == 1'b1) begin
				keycodev <= keycode;
				case (keycode[15:8])
					8'hf0: // Break Codes
						case (keycode[7:0])
							// Modificators
							8'h12 : begin // Left Shift
									mods[0] <= 1'b0;
									start <= 1'b0;
								end
							8'h59 : begin // Right Shift
									mods[0] <= 1'b0;
									start <= 1'b0;
								end
							default:
								start <= 1'b0;
						endcase
					default: // Make Codes
						case (keycode[7:0])
							// Modificators
							8'h12 : begin // Left Shift
									mods[0] <= 1'b1;
									start <= 1'b0;
								end
							8'h59 : begin // Right Shift
									mods[0] <= 1'b1;
									start <= 1'b0;
								end
							// Special
							8'h29 : begin
									key <= 16'h0020;  // Space
									start <= 1'b1;
								end
							8'h5a : begin
									key <= 16'h000d;  // Enter
									start <= 1'b1;
								end
							8'h76 : begin
									key <= 16'h001b;  // Escape
									start <= 1'b1;
								end
							8'h66 : begin
									key <= 16'h0008;  // Backspace
									start <= 1'b1;
								end
							8'h0d : begin
									key <= 16'h000b;  // Tab
									start <= 1'b1;
								end
							8'h52 : begin
									if (mods[0])
										key <= 16'h0022;  // "
									else
										key <= 16'h0027;  // '
									start <= 1'b1;
								end
							8'h4c : begin
									if (mods[0])
										key <= 16'h003a;  // :
									else
										key <= 16'h003b;  // ;
									start <= 1'b1;
								end
							8'h54 : begin
									if (mods[0])
										key <= 16'h007b;  // {
									else
										key <= 16'h005b;  // [
									start <= 1'b1;
								end
							8'h5b : begin
									if (mods[0])
										key <= 16'h007d;  // }
									else
										key <= 16'h005d;  // ]
									start <= 1'b1;
								end
							8'h0e : begin
									if (mods[0])
										key <= 16'h007e;  // ~
									else
										key <= 16'h0060;  // reverse '
									start <= 1'b1;
								end
							8'h5d : begin
									if (mods[0])
										key <= 16'h007c;  // |
									else
										key <= 16'h005c;  // \
									start <= 1'b1;
								end
							8'h4e : begin
									if (mods[0])
										key <= 16'h005f;  // _
									else
										key <= 16'h002d;  // -
									start <= 1'b1;
								end
							8'h55 : begin
									if (mods[0])
										key <= 16'h002b;  // +
									else
										key <= 16'h003d;  // =
									start <= 1'b1;
								end
							8'h4a : begin
									if (mods[0])
										key <= 16'h003f;  // ?
									else
										key <= 16'h002f;  // /
									start <= 1'b1;
								end
							8'h41 : begin
									if (mods[0])
										key <= 16'h003c;  // <
									else
										key <= 16'h002c;  // ,
									start <= 1'b1;
								end
							8'h49 : begin
									if (mods[0])
										key <= 16'h003e;  // >
									else
										key <= 16'h002e;  // .
									start <= 1'b1;
								end
							// Numbers 0-9
							8'h45 : begin
									if (mods[0])
										key <= 16'h0029;  // )
									else
										key <= 16'h0030;  // 0
									start <= 1'b1;
								end
							8'h16 : begin
									if (mods[0])
										key <= 16'h0021;  // !
									else
										key <= 16'h0031;  // 1
									start <= 1'b1;
								end
							8'h1e : begin
									if (mods[0])
										key <= 16'h0040;  // @
									else
										key <= 16'h0032;  // 2
									start <= 1'b1;
								end
							8'h26 : begin
									if (mods[0])
										key <= 16'h0023;  // #
									else
										key <= 16'h0033;  // 3
									start <= 1'b1;
								end
							8'h25 : begin
									if (mods[0])
										key <= 16'h0024;  // $
									else
										key <= 16'h0034;  // 4
									start <= 1'b1;
								end
							8'h2e : begin
									if (mods[0])
										key <= 16'h0025;  // %
									else
										key <= 16'h0035;  // 5
									start <= 1'b1;
								end
							8'h36 : begin
									if (mods[0])
										key <= 16'h005e;  // ^
									else
										key <= 16'h0036;  // 6
									start <= 1'b1;
								end
							8'h3d : begin
									if (mods[0])
										key <= 16'h0026;  // &
									else
										key <= 16'h0037;  // 7
									start <= 1'b1;
								end
							8'h3e : begin
									if (mods[0])
										key <= 16'h002a;  // *
									else
										key <= 16'h0038;  // 8
									start <= 1'b1;
								end
							8'h46 : begin
									if (mods[0])
										key <= 16'h0028;  // (
									else
										key <= 16'h0039;  // 9
									start <= 1'b1;
								end
							// Characters
							8'h15 : begin
									if (mods[0])
										key <= 16'h0051;  // Q
									else
										key <= 16'h0071;  // q 
									start <= 1'b1;
								end
							8'h1d : begin
									if (mods[0])
										key <= 16'h0057;  // W
									else
										key <= 16'h0077;  // w
									start <= 1'b1;
								end
							8'h24 : begin
									if (mods[0])
										key <= 16'h0045;  // E
									else
										key <= 16'h0065;  // e
									start <= 1'b1;
								end
							8'h2d : begin
									if (mods[0])
										key <= 16'h0052;  // R
									else
										key <= 16'h0072;  // r
									start <= 1'b1;
								end
							8'h2c : begin
									if (mods[0])
										key <= 16'h0054;  // T
									else
										key <= 16'h0074;  // t
									start <= 1'b1;
								end
							8'h35 : begin
									if (mods[0])
										key <= 16'h0059;  // Y
									else
										key <= 16'h0079;  // y
									start <= 1'b1;
								end
							8'h3c : begin
									if (mods[0])
										key <= 16'h0055;  // U
									else
										key <= 16'h0075;  // u
									start <= 1'b1;
								end
							8'h43 : begin
									if (mods[0])
										key <= 16'h0049;  // I
									else
										key <= 16'h0069;  // i
									start <= 1'b1;
								end
							8'h44 : begin
									if (mods[0])
										key <= 16'h004f;  // O
									else
										key <= 16'h006f;  // o
									start <= 1'b1;
								end
							8'h4d : begin
									if (mods[0])
										key <= 16'h0050;  // P
									else
										key <= 16'h0070;  // p
									start <= 1'b1;
								end
							8'h1c : begin
									if (mods[0])
										key <= 16'h0041;  // A
									else
										key <= 16'h0061;  // a
									start <= 1'b1;
								end
							8'h1b : begin
									if (mods[0])
										key <= 16'h0053;  // S
									else
										key <= 16'h0073;  // s
									start <= 1'b1;
								end
							8'h23 : begin
									if (mods[0])
										key <= 16'h0044;  // D
									else
										key <= 16'h0064;  // d
									start <= 1'b1;
								end
							8'h2b : begin
									if (mods[0])
										key <= 16'h0046;  // F
									else
										key <= 16'h0066;  // f
									start <= 1'b1;
								end
							8'h34 : begin
									if (mods[0])
										key <= 16'h0047;  // G
									else
										key <= 16'h0067;  // g
									start <= 1'b1;
								end
							8'h33 : begin
									if (mods[0])
										key <= 16'h0048;  // H
									else
										key <= 16'h0068;  // h 
									start <= 1'b1;
								end
							8'h3b : begin
									if (mods[0])
										key <= 16'h004a;  // J
									else
										key <= 16'h006a;  // j
									start <= 1'b1;
								end
							8'h42 : begin
									if (mods[0])
										key <= 16'h004b;  // K
									else
										key <= 16'h006b;  // k
									start <= 1'b1;
								end
							8'h4b : begin
									if (mods[0])
										key <= 16'h004c;  // L
									else
										key <= 16'h006c;  // l
									start <= 1'b1;
								end
							8'h1a : begin
									if (mods[0])
										key <= 16'h005a;  // Z
									else
										key <= 16'h007a;  // z
									start <= 1'b1;
								end
							8'h22 : begin
									if (mods[0])
										key <= 16'h0058;  // X
									else
										key <= 16'h0078;  // x
									start <= 1'b1;
								end
							8'h21 : begin
									if (mods[0])
										key <= 16'h0043;  // C
									else
										key <= 16'h0063;  // c
									start <= 1'b1;
								end
							8'h2a : begin
									if (mods[0])
										key <= 16'h0056;  // V
									else
										key <= 16'h0076;  // v
									start <= 1'b1;
								end
							8'h32 : begin
									if (mods[0])
										key <= 16'h0042;  // B
									else
										key <= 16'h0062;  // b
									start <= 1'b1;
								end
							8'h31 : begin
									if (mods[0])
										key <= 16'h004e;  // N
									else
										key <= 16'h006e;  // n
									start <= 1'b1;
								end
							8'h3a : begin
									if (mods[0])
										key <= 16'h004d;  // M
									else
										key <= 16'h006d;  // m
									start <= 1'b1;
								end
							default:
								start <= 1'b0;
						endcase
				endcase
			end else
				start <= 1'b0;
	 
		always @(posedge clk) begin
			if (start == 1'b1) begin
				keycode_valid <= 1'b1;
			end
			else begin 
				if (keycode_recv == 1'b1) begin
					keycode_valid <= 1'b0;
				end
			end
		end
		
	endmodule
	
`timescale 1ns / 1ps

module vga800x600(
	input wire i_clk,           // base clock
	input wire i_pix_stb,       // pixel clock strobe
	input wire i_rst,           // reset: restarts frame
	output wire o_hs,           // horizontal sync
	output wire o_vs,           // vertical sync
	output wire o_blanking,     // high during blanking interval
	output wire o_active,       // high during active pixel drawing
	output wire o_screenend,    // high for one tick at the end of screen
	output wire o_animate,      // high for one tick at end of active drawing
	output wire [10:0] o_x,     // current pixel x position
	output wire  [9:0] o_y      // current pixel y position
	);

	// VGA timings https://timetoexplore.net/blog/video-timings-vga-720p-1080p
	localparam HS_STA = 40;              // horizontal sync start
	localparam HS_END = 40 + 128;        // horizontal sync end
	localparam HA_STA = 40 + 128 + 88;   // horizontal active pixel start
	localparam VS_STA = 600 + 1;         // vertical sync start
	localparam VS_END = 600 + 1 + 4;     // vertical sync end
	localparam VA_END = 600;             // vertical active pixel end
	localparam LINE   = 1056;            // complete line (pixels)
	localparam SCREEN = 628;             // complete screen (lines)

	reg [10:0] h_count; // line position
	reg  [9:0] v_count; // screen position

	// generate sync signals (active high for 800x600)
	assign o_hs = ((h_count >= HS_STA) & (h_count < HS_END));
	assign o_vs = ((v_count >= VS_STA) & (v_count < VS_END));

	// keep x and y bound within the active pixels
	assign o_x = (h_count < HA_STA) ? 0 : (h_count - HA_STA);
	assign o_y = (v_count >= VA_END) ? (VA_END - 1) : (v_count);

	// blanking: high within the blanking period
	assign o_blanking = ((h_count < HA_STA) | (v_count > VA_END - 1));

	// active: high during active pixel drawing
	assign o_active = ~((h_count < HA_STA) | (v_count > VA_END - 1)); 

	// screenend: high for one tick at the end of the screen
	assign o_screenend = ((v_count == SCREEN - 1) & (h_count == LINE));

	// animate: high for one tick at the end of the final active pixel line
	assign o_animate = ((v_count == VA_END - 1) & (h_count == LINE));

	always @ (posedge i_clk)
	begin
		if (i_rst)  // reset to start of frame
		begin
			h_count <= 0;
			v_count <= 0;
		end
		if (i_pix_stb)  // once per pixel
		begin
			if (h_count == LINE)  // end of line
			begin
				h_count <= 0;
				v_count <= v_count + 1;
			end
			else 
				h_count <= h_count + 1;

			if (v_count == SCREEN)  // end of screen
				v_count <= 0;
		end
	end
endmodule

module textvideoram #(parameter ADDR_WIDTH=8, DATA_WIDTH=8, DEPTH=256) (
    input wire clk,
    input wire [ADDR_WIDTH-1:0] addr, 
    input wire wen,
    input wire [DATA_WIDTH-1:0] i_data,
    output reg [DATA_WIDTH-1:0] o_data 
    );

    reg [DATA_WIDTH-1:0] videoram [0:DEPTH-1];

    integer i;    
    initial begin
        for (i=0 ; i<DEPTH ; i=i+1) begin
            videoram[i] = 0;  
        end
    end

    always @ (posedge clk)
    begin
        if(wen) begin
            videoram[addr] <= i_data;
        end
        else begin
            o_data <= videoram[addr];
        end     
    end
endmodule



module romfonts #(parameter ADDR_WIDTH=8, DATA_WIDTH=8, DEPTH=256, FONTSFILE="") (
    input wire [ADDR_WIDTH-1:0] addr, 
    output wire [DATA_WIDTH-1:0] data 
    );

    reg [DATA_WIDTH-1:0] fonts_array [0:DEPTH-1]; 

    initial begin
        if (FONTSFILE > 0)
        begin
            $display("Loading memory init file '" + FONTSFILE + "' into array.");
            $readmemh(FONTSFILE, fonts_array);
        end
    end

	assign data = fonts_array[addr];

endmodule

module bondmachine_main(

	input clk,
	input btnC,
	output reg [15:0] led,
	input PS2Data,
	input PS2Clk,
	output wire VGA_HS_O,
	output wire VGA_VS_O,
	output reg [3:0] VGA_R,
	output reg [3:0] VGA_G,
	output reg [3:0] VGA_B
);

	// External ports creation ended
	assign reset = btnC;
	wire [15:0] Input0;
	wire Input0_valid;
	wire Input0_received;
	wire [15:0] Output0;

	// Processing per-extramodule initializazions

	reg [7:0]   hcount;
	reg [7:0]   header [0:99]; 

	initial begin
		$display("Loading memory init file header into array.");
		$readmemh("../header.mem", header);
		hcount=99;
	end
	wire [7:0] p0vtm0dout;
	wire [7:0] p0vtm0addrfromext;
	// Processing BM ports originated from external modules (not IO)
	bondmachine bondmachine_inst (clk, reset, Input0, Input0_valid, Input0_received, Output0, Output0_valid, Output0_received, p0vtm0dout, p0vtm0addrfromext);

	// Processing BM connected firmwares
	// Eventually create here a module
	bondkeydrv bondkeydrv_inst(clk, PS2Data, PS2Clk, Input0, Input0_valid, Input0_received);

	// Processing BM IO
	always @ (posedge clk) begin
		led[0] <= Output0[0];
		led[1] <= Output0[1];
		led[2] <= Output0[2];
		led[3] <= Output0[3];
		led[4] <= Output0[4];
		led[5] <= Output0[5];
		led[6] <= Output0[6];
		led[7] <= Output0[7];
		led[8] <= Output0[8];
		led[9] <= Output0[9];
		led[10] <= Output0[10];
		led[11] <= Output0[11];
		led[12] <= Output0[12];
		led[13] <= Output0[13];
		led[14] <= Output0[14];
		led[15] <= Output0[15];
	end
	// Processing Extra modules processes


	// generate a 40 MHz pixel strobe
	reg [15:0] cnt;
	reg pix_stb;
	always @(posedge clk)
		{pix_stb, cnt} <= cnt + 16'h6666;  // divide by 2.5: (2^16)/2.5 = 0x6666

	wire [10:0] x;  // current pixel x position: 11-bit value: 0-2047
	wire  [9:0] y;  // current pixel y position: 10-bit value: 0-1023

	// Connect the VGA display
	vga800x600 display (
		.i_clk(clk),
		.i_pix_stb(pix_stb),
		.i_rst(reset),
		.o_hs(VGA_HS_O), 
		.o_vs(VGA_VS_O), 
		.o_x(x), 
		.o_y(y)
	);

	// Font ROM initialization
	localparam FONTROM_DEPTH = 1024; 
	localparam FONTROM_A_WIDTH = 10;
	localparam FONTROM_D_WIDTH = 8;

	reg [FONTROM_A_WIDTH-1:0] fontaddress;
	wire [FONTROM_D_WIDTH-1:0] dataout;

	romfonts #(
		.ADDR_WIDTH(FONTROM_A_WIDTH), 
		.DATA_WIDTH(FONTROM_D_WIDTH), 
		.DEPTH(FONTROM_DEPTH), 
		.FONTSFILE("../fonts.mem"))
		fonts (
		.addr(fontaddress[FONTROM_A_WIDTH-1:0]), 
		.data(dataout)
	);

	// HEADER Video RAM
	localparam HEAD_VIDEORAM_LEFT = 0;
	localparam HEAD_VIDEORAM_TOP = 1;
	localparam HEAD_VIDEORAM_ROWS = 1;
	localparam HEAD_VIDEORAM_COLS = 100;
	localparam HEAD_VIDEORAM_DEPTH = 100; 
	localparam HEAD_VIDEORAM_A_WIDTH = 8;
	localparam HEAD_VIDEORAM_D_WIDTH = 8;

	reg [HEAD_VIDEORAM_A_WIDTH-1:0] headaddress;
	wire [HEAD_VIDEORAM_D_WIDTH-1:0] headdataout;
	reg headwantwrite;
	reg [7:0] headdata;

	textvideoram #(
		.ADDR_WIDTH(HEAD_VIDEORAM_A_WIDTH), 
		.DATA_WIDTH(HEAD_VIDEORAM_D_WIDTH), 
		.DEPTH(HEAD_VIDEORAM_DEPTH))
		headvideoram (
		.addr(headaddress[HEAD_VIDEORAM_A_WIDTH-1:0]), 
		.o_data(headdataout[HEAD_VIDEORAM_D_WIDTH-1:0]),
		.clk(clk),
		.wen(headwantwrite),
		.i_data(headdata[HEAD_VIDEORAM_D_WIDTH-1:0])
	);

	// Pixel processing
	reg [11:0] colour;
	wire [7:0] pix;
    
	wire within_head;
	assign within_head = ((x >= HEAD_VIDEORAM_LEFT*8) & (y >=  HEAD_VIDEORAM_TOP*8) & (x < HEAD_VIDEORAM_LEFT*8+HEAD_VIDEORAM_COLS*8) & (y < HEAD_VIDEORAM_TOP*8+HEAD_VIDEORAM_ROWS*8)) ? 1 : 0;

	localparam CP0_VIDEORAM_LEFT = 3;
	localparam CP0_VIDEORAM_TOP = 3;
	localparam CP0_VIDEORAM_COLS = 16;
	localparam CP0_VIDEORAM_ROWS = 16;

	wire border_t_cp0;
	wire border_b_cp0;
	wire border_l_cp0;
	wire border_r_cp0;
	wire border_cp0;
	wire within_cp0;
	assign within_cp0 = ((x >= CP0_VIDEORAM_LEFT*8) & (y >=  CP0_VIDEORAM_TOP*8) & (x < CP0_VIDEORAM_LEFT*8+CP0_VIDEORAM_COLS*8) & (y < CP0_VIDEORAM_TOP*8+CP0_VIDEORAM_ROWS*8)) ? 1 : 0; ;
	assign within_cp0 = ((x >= CP0_VIDEORAM_LEFT*8) & (y >=  CP0_VIDEORAM_TOP*8) & (x < CP0_VIDEORAM_LEFT*8+CP0_VIDEORAM_COLS*8) & (y < CP0_VIDEORAM_TOP*8+CP0_VIDEORAM_ROWS*8)) ? 1 : 0; ;
	assign border_t_cp0 = ((x >= 24 - 2) & (x <= 24+128 + 2) & ( y == 24 - 2)) ? 1 : 0; ;
	assign border_b_cp0 = ((x >= 24 - 2) & (x <= 24+128 + 2) & ( y == 24+128 + 2)) ? 1 : 0; ;
	assign border_l_cp0 = ((y >= 24 - 2) & (y <= 24+128 + 2) & ( x == 24 - 2)) ? 1 : 0; ;
	assign border_r_cp0 = ((y >= 24 - 2) & (y <= 24+128 + 2) & ( x == 24+128 + 2)) ? 1 : 0; ;
	assign border_cp0 = (( border_t_cp0 ) | ( border_b_cp0 ) | ( border_l_cp0 ) | ( border_r_cp0 )) ? 1 : 0; ;

//	assign pix = dataout >> ((x-1) % 8) ;
	assign pix = dataout >> ((x-2) % 8) ;

	// Font address Process
	always @ (posedge clk)
	begin
		if (within_head)
			fontaddress[FONTROM_A_WIDTH-1:0] <= headdataout * 8 + (y % 8);
		else if (within_cp0)
			fontaddress[FONTROM_A_WIDTH-1:0] <= p0vtm0dout * 8 + (y % 8);
		else
			fontaddress[FONTROM_A_WIDTH-1:0] <= 0;
	end

	// HEADER Processing
	always @ (posedge clk)
	begin
		if (hcount > 0) begin
			hcount <= hcount - 1;
			headaddress[HEAD_VIDEORAM_A_WIDTH-1:0] <= hcount;
			headdata[HEAD_VIDEORAM_D_WIDTH-1:0] <= header[hcount];
			headwantwrite <= 1;
		end 
		else
		begin
			headaddress[HEAD_VIDEORAM_A_WIDTH-1:0] <= ((y / 8) - HEAD_VIDEORAM_TOP) * HEAD_VIDEORAM_COLS +  ((x / 8) - HEAD_VIDEORAM_LEFT);
			headwantwrite <= 0;
		end
	end

	// CP0 Processing

	reg [7:0] cp0address;

	always @ (posedge clk)
	begin
		if (within_cp0)
			cp0address[7:0] <= ((y / 8) - CP0_VIDEORAM_TOP) * CP0_VIDEORAM_COLS +  ((x / 8) - CP0_VIDEORAM_LEFT);
	end

	assign p0vtm0addrfromext[7:0] = cp0address[7:0];

	// Channels processing
	always @ (posedge clk)
	begin
		if (( within_head ) && pix[0] == 1'b1)
			colour <= 12'b111111111111;
		else if ((  within_cp0 ) && pix[0] == 1'b1)
			colour <= 12'b110111011101;
		else if ((  within_cp0 ) && pix[0] == 1'b0)
			colour <= 12'b001100110011;
		else if ( border_cp0 )
			colour <= 12'h01c;

		else
			colour <= 0;
    
		VGA_R <= colour[11:8];
		VGA_G <= colour[7:4];
		VGA_B <= colour[3:0];
	end

endmodule
module bondmachine(clk, reset, i0, i0_valid, i0_received, o0, o0_valid, o0_received, p0vtm0dout, p0vtm0addrfromext);

	// Clock and reset input ports
	input clk, reset;
	input [15:0] i0;
	input i0_valid;
	output i0_received;
	//--------------Output Ports-----------------------
	output [15:0] o0;
	output o0_valid;
	input o0_received;
	output [7:0] p0vtm0dout;
	input [7:0] p0vtm0addrfromext;



	//Analyzing Internal output p0o0
	//Internal output p0o0 is connected to o0
	wire [15:0] p0o0;
	wire p0o0_valid;
	wire p0o0_received;
	wire o0_received;
	//Analyzing Internal output i0
	//Internal output i0 is connected to p0i0
	wire [15:0] i0;
	wire i0_valid;
	wire i0_received;
	wire p0i0_received;

	wire [15:0] p0vtm0din;
	wire [15:0] p0vtm0addrfromcp;
	wire p0vtm0wren;
	wire p0vtm0en;


	//Instantiation of the Processors and Shared Objects
	a0 a0_inst(clk, reset, i0, i0_valid, p0i0_received, p0o0, p0o0_valid, p0o0_received, p0vtm0din, p0vtm0addrfromcp, p0vtm0wren, p0vtm0en);
	vtm0 vtm0_inst (clk, reset, p0vtm0din, p0vtm0addrfromcp, p0vtm0wren, p0vtm0en, p0vtm0dout, p0vtm0addrfromext);

	assign o0 = p0o0;
	assign o0_valid = p0o0_valid;

	assign p0o0_received = o0_received;
	assign i0_received = p0i0_received;

endmodule
`timescale 1ns/1ps
module p0ram(clk, rst, din, dout, addr, wren, en);

	//--------------Input Ports-----------------------
	input clk;
	input rst;
	input [3:0] addr;
	input [15:0] din;
	input wren;
	input en;

	//--------------Inout Ports-----------------------
	output [15:0] dout;

	//--------------Reg-------------------------------
	reg [15:0] mem [0:15];

	reg [15:0] dout_i;

	initial begin
		mem[0] = 16'b0000100000000000;
		mem[1] = 16'b0011100000010000;
		mem[2] = 16'b0000000000000000;
		mem[3] = 16'b0000000000000000;
		mem[4] = 16'b0000000000000000;
		mem[5] = 16'b0000000000000000;
		mem[6] = 16'b0000000000000000;
		mem[7] = 16'b0000000000000000;
		mem[8] = 16'b0000000000000000;
		mem[9] = 16'b0000000000000000;
		mem[10] = 16'b0000000000000000;
		mem[11] = 16'b0000000000000000;
	end
	// Memory Write Block  
	// Write Operation we = 1 
	always @ (posedge clk) 
	begin : MEM_WRITE 
		integer k; 
		if (rst)
		begin 
		end 
		else if (wren)
			mem[addr] <= #1 din;
	end 

	// Memory Read Block
	// Read Operation when we = 0 and oe = 1 
	always @ (posedge clk) 
	begin : MEM_READ 
		if (!wren)
			dout_i <= #1 mem[addr];
	end

	assign dout = dout_i;

endmodule 
`timescale 1ns/1ps
module p0rom(input [6:0] rom_bus, output [15:0] rom_value);
	reg [15:0] _rom [0:127];
	initial
	begin
	_rom[0] = 16'b1001000000000010;
	_rom[1] = 16'b0000000111000000;
	_rom[2] = 16'b1001000000000010;
	_rom[3] = 16'b1001000101000010;
	_rom[4] = 16'b0000001011100000;
	_rom[5] = 16'b1001001101000101;
	_rom[6] = 16'b1000110001100000;
	_rom[7] = 16'b0100110000000000;
	_rom[8] = 16'b1001000000000010;
	_rom[9] = 16'b1001000101000111;
	_rom[10] = 16'b0000001011100000;
	_rom[11] = 16'b1001001101001011;
	_rom[12] = 16'b1000110001100000;
	_rom[13] = 16'b0100110000000000;
	_rom[14] = 16'b0011100000000000;
	_rom[15] = 16'b0011100011110000;
	_rom[16] = 16'b1001000000000001;
	_rom[17] = 16'b0111000000000000;
	_rom[18] = 16'b1001000001001100;
	_rom[19] = 16'b1001000100010000;
	_rom[20] = 16'b0000001110000000;
	_rom[21] = 16'b0011100000000000;
	_rom[22] = 16'b1001000000000000;
	_rom[23] = 16'b0111000000000000;
	_rom[24] = 16'b1001000001010101;
	_rom[25] = 16'b1001000100010000;
	_rom[26] = 16'b0000001110000000;
	_rom[27] = 16'b0011100000000000;
	_rom[28] = 16'b1001001000000000;
	_rom[29] = 16'b1001001100001101;
	_rom[30] = 16'b0010100100000000;
	_rom[31] = 16'b0001000101100000;
	_rom[32] = 16'b0100001001100000;
	_rom[33] = 16'b0111100101000000;
	_rom[34] = 16'b0110100100000000;
	_rom[35] = 16'b0011000000000000;
	_rom[36] = 16'b0011001000000000;
	_rom[37] = 16'b0011100111100000;
	_rom[38] = 16'b0001100001000000;
	_rom[39] = 16'b1001001100000000;
	_rom[40] = 16'b0111101100000000;
	_rom[41] = 16'b0101000001011000;
	_rom[42] = 16'b0010000000000000;
	_rom[43] = 16'b0011101010000000;
	_rom[44] = 16'b0110000000000000;
	_rom[45] = 16'b1000000000000000;
	_rom[46] = 16'b0101101100000000;
	_rom[47] = 16'b1000110000100000;
	_rom[48] = 16'b0001001110000000;
	_rom[49] = 16'b0010001000000000;
	_rom[50] = 16'b0101001001101100;
	_rom[51] = 16'b0011000000000000;
	_rom[52] = 16'b0011000100000000;
	_rom[53] = 16'b0100001011100000;
	_rom[54] = 16'b0110000000000000;
	_rom[55] = 16'b1000000000000000;
	_rom[56] = 16'b0110000000000000;
	_rom[57] = 16'b1000101000000000;
	_rom[58] = 16'b0101001001111110;
	_rom[59] = 16'b0111101000100000;
	_rom[60] = 16'b0011000000000000;
	_rom[61] = 16'b0011000100000000;
	_rom[62] = 16'b0011101110010000;
	_rom[63] = 16'b0110000000000000;
	_rom[64] = 16'b1000000000000000;
	_rom[65] = 16'b0000000000000010;
	_rom[66] = 16'b0000000001101111;
	_rom[67] = 16'b0000000001101110;
	_rom[68] = 16'b0000000000000000;
	_rom[69] = 16'b0000000000010000;
	_rom[70] = 16'b0000000000000011;
	_rom[71] = 16'b0000000001101111;
	_rom[72] = 16'b0000000001100110;
	_rom[73] = 16'b0000000001100110;
	_rom[74] = 16'b0000000000000000;
	_rom[75] = 16'b0000000000010110;
	_rom[76] = 16'b0000000001001100;
	_rom[77] = 16'b0000000001100101;
	_rom[78] = 16'b0000000001100100;
	_rom[79] = 16'b0000000000100000;
	_rom[80] = 16'b0000000001001111;
	_rom[81] = 16'b0000000001101110;
	_rom[82] = 16'b0000000000100001;
	_rom[83] = 16'b0000000000100000;
	_rom[84] = 16'b0000000000000000;
	_rom[85] = 16'b0000000001001100;
	_rom[86] = 16'b0000000001100101;
	_rom[87] = 16'b0000000001100100;
	_rom[88] = 16'b0000000000100000;
	_rom[89] = 16'b0000000001001111;
	_rom[90] = 16'b0000000001100110;
	_rom[91] = 16'b0000000001100110;
	_rom[92] = 16'b0000000000100001;
	_rom[93] = 16'b0000000000000000;
	end
	assign rom_value = _rom[rom_bus];
endmodule
`timescale 1ns/1ps
module p0(clock_signal, reset_signal, rom_bus, rom_value, ram_din, ram_dout, ram_addr, ram_wren, ram_en, i0, i0_valid, i0_received, o0, o0_valid, o0_received, vtm0din, vtm0addr, vtm0wren, vtm0en);

	input clock_signal;
	input reset_signal;
	output  [6:0] rom_bus;
	input  [15:0] rom_value;
	input  [15:0] ram_dout;
	output [15:0] ram_din;
	output  [3:0] ram_addr;
	output ram_wren, ram_en;

	input [15:0] i0;
	input i0_valid;
	output i0_received;
	output [15:0] o0;
	output o0_valid;
	input o0_received;

	output [7:0] vtm0din;
	output [15:0] vtm0addr;
	output vtm0wren;
	output vtm0en;
	reg [7:0] vtm0_din_i;
	reg [15:0] vtm0_addr_i;
	reg vtm0_wren_i;
	wire vtm0_en_i;
	assign vtm0din = vtm0_din_i;
	assign vtm0addr = vtm0_addr_i;
	assign vtm0wren = vtm0_wren_i;
	assign vtm0en = vtm0_en_i;

			// Opcodes in the istructions, lenght accourding the number of the selected.
	localparam	CALLO8S=5'b00000,          // Call a rom subroutine via an hardware stack called s with depth 8
			CLR=5'b00001,          // Clear register
			CMPR=5'b00010,          // Register comparison
			CPY=5'b00011,          // Copy from a register to another
			DEC=5'b00100,          // Decrement a register by 1
			I2RW=5'b00101,          // Input to register
			INC=5'b00110,          // Increment a register by 1
			J=5'b00111,          // Jump to a program location
			JCMPL=5'b01000,          // Jump to a program location conditioned to the comparison flag
			JCMPRIO=5'b01001,          // Register indirect Jump to a program location on ROM conditioned to the comparison flag
			JZ=5'b01010,          // Zero conditional jump
			M2RRI=5'b01011,          // ROM to register
			NOP=5'b01100,          // No operation
			R2MRI=5'b01101,          // Register indirect copy to RAM
			R2O=5'b01110,          // Register to output
			R2VRI=5'b01111,          // Register indirect copy to video RAM
			RET8S=5'b10000,          // Return from a subroutine via an hardware stack called s with depth 8
			RO2RRI=5'b10001,          // ROM to register
			RSETS8=5'b10010;          // Register set value with fixed size

	localparam	R0=3'b000,		// Registers in the intructions
			R1=3'b001,
			R2=3'b010,
			R3=3'b011,
			R4=3'b100,
			R5=3'b101,
			R6=3'b110,
			R7=3'b111;
	localparam			I0=1'b0;
	localparam			O0=1'b0;
	reg [15:0] _auxo0;

	reg [15:0] _ram [0:15];		// Internal processor RAM

	(* KEEP = "TRUE" *) reg [6:0] _pc;		// Program counter

	// The number of registers are 2^R, two letters and an underscore as identifier , maximum R=8 and 265 rigisters
	(* KEEP = "TRUE" *) reg [15:0] _r0;
	(* KEEP = "TRUE" *) reg [15:0] _r1;
	(* KEEP = "TRUE" *) reg [15:0] _r2;
	(* KEEP = "TRUE" *) reg [15:0] _r3;
	(* KEEP = "TRUE" *) reg [15:0] _r4;
	(* KEEP = "TRUE" *) reg [15:0] _r5;
	(* KEEP = "TRUE" *) reg [15:0] _r6;
	(* KEEP = "TRUE" *) reg [15:0] _r7;

	wire [15:0] current_instruction;
	reg [15:0] ram_instruction;
	reg exec_mode; // 0 = harvard , 1=VN
	reg [1:0] vn_state;
	localparam FETCH=2'b00, WAIT=2'b10, EXECUTE=2'b01;
	assign current_instruction= (exec_mode==1'b0) ? rom_value : ram_instruction;


// Start of the component "header" for the opcode callo8s

	reg [1:0] restack0_8sSM;
	localparam	CALL1 = 2'b00,
			CALL2 = 2'b01,
			CALL3 = 2'b10,
			CALL4 = 2'b11;

	reg [7:0] restack0_8ssenderData;
	reg restack0_8ssenderWrite;
	wire restack0_8ssenderAck;

	wire [7:0] restack0_8sreceiverData;
	reg restack0_8sreceiverRead;
	wire restack0_8sreceiverAck;

	wire restack0_8sempty;
	wire restack0_8sfull;

	restack0_8s restack0_8s_inst (
		.clk(clock_signal),
		.reset(reset_signal),
		.senderData(restack0_8ssenderData),
		.senderWrite(restack0_8ssenderWrite),
		.senderAck(restack0_8ssenderAck),
		.receiverData(restack0_8sreceiverData),
		.receiverRead(restack0_8sreceiverRead),
		.receiverAck(restack0_8sreceiverAck),
		.empty(restack0_8sempty),
		.full(restack0_8sfull)
	);

initial begin
	restack0_8sSM <= CALL1;
end

// Start of the component "header" for the opcode clr


// Start of the component "header" for the opcode cmpr

	reg cmpflag;

// Start of the component "header" for the opcode cpy


// Start of the component "header" for the opcode dec


// Start of the component "header" for the opcode i2rw


	reg i0_recv;

	always @(posedge clock_signal, posedge reset_signal)
	begin
		if (reset_signal)
		begin
			i0_recv <= #1 1'b0;
		end
		else
		begin
			case(current_instruction[15:11])
				I2RW: begin
					case (current_instruction[7])
					I0 : begin
						if (i0_valid)
						begin
							i0_recv <= #1 1'b1;
						end else begin
							i0_recv <= #1 1'b0;
						end
					end
					default: begin
						if (!i0_valid)
						begin
							i0_recv <= #1 1'b0;
						end
					end
					endcase
				end
				default: begin
					if (!i0_valid)
					begin
						i0_recv <= #1 1'b0;
					end
				end
			endcase
		end
	end

// Start of the component "header" for the opcode inc


// Start of the component "header" for the opcode j


// Start of the component "header" for the opcode jcmpl


// Start of the component "header" for the opcode jcmprio


// Start of the component "header" for the opcode jz


// Start of the component "header" for the opcode m2rri

	//Internal Reg Wire for M2R opcode
	reg state_read_mem_m2rri;
	reg wait_read_mem;
	reg [3:0] addr_ram_m2rri;


// Start of the component "header" for the opcode nop


// Start of the component "header" for the opcode r2mri

	reg [3:0] addr_ram_to_mem;
	reg [15:0] ram_din_i;
	reg wr_int_ram;

// Start of the component "header" for the opcode r2o


	reg o0_val;

	always @(posedge clock_signal, posedge reset_signal)
	begin
		if (reset_signal)
		begin
			o0_val <= #1 1'b0;
		end
		else
		begin
		if (exec_mode == 1'b0 || vn_state == EXECUTE) begin
			case(current_instruction[15:11])
				R2O: begin
					case (current_instruction[7])
					O0 : begin
						o0_val <= 1'b1;
					end
					default: begin
						if (o0_received)
						begin
							o0_val <= #1 1'b0;
						end
					end
					endcase
				end
				default: begin
					if (o0_received)
					begin
						o0_val <= #1 1'b0;
					end
				end
			endcase
		end
		end
	end

// Start of the component "header" for the opcode r2vri


// Start of the component "header" for the opcode ret8s


// Start of the component "header" for the opcode ro2rri

	wire [15:0] romread_value;
	reg [6:0] romread_bus;
	reg romread_ready;

	p0rom romread_instance(romread_bus,romread_value);

// Start of the component "header" for the opcode rsets8


	always @(posedge clock_signal, posedge reset_signal)
	begin
		if(reset_signal)
		begin
			_pc <= #1 7'h0;
			_r0 <= #1 16'h0;
			_r1 <= #1 16'h0;
			_r2 <= #1 16'h0;
			_r3 <= #1 16'h0;
			_r4 <= #1 16'h0;
			_r5 <= #1 16'h0;
			_r6 <= #1 16'h0;
			_r7 <= #1 16'h0;

// Start of the component "reset" for the opcode callo8s


// Start of the component "reset" for the opcode clr


// Start of the component "reset" for the opcode cmpr


// Start of the component "reset" for the opcode cpy


// Start of the component "reset" for the opcode dec


// Start of the component "reset" for the opcode i2rw


// Start of the component "reset" for the opcode inc


// Start of the component "reset" for the opcode j


// Start of the component "reset" for the opcode jcmpl


// Start of the component "reset" for the opcode jcmprio


// Start of the component "reset" for the opcode jz


// Start of the component "reset" for the opcode m2rri


// Start of the component "reset" for the opcode nop


// Start of the component "reset" for the opcode r2mri


// Start of the component "reset" for the opcode r2o


// Start of the component "reset" for the opcode r2vri


// Start of the component "reset" for the opcode ret8s


// Start of the component "reset" for the opcode ro2rri


// Start of the component "reset" for the opcode rsets8

		end
		else begin
			if (exec_mode == 1'b1 && vn_state == FETCH) begin
				vn_state <= WAIT;
			end
			else if (exec_mode == 1'b1 && vn_state == WAIT) begin
				vn_state <= EXECUTE;
				ram_instruction <= ram_dout;
			end
			else begin
			$display("Program Counter:%d", _pc);
			$display("Instruction:%b", rom_value);
			$display("Registers r0:%b r1:%b r2:%b r3:%b r4:%b r5:%b r6:%b r7:%b ", _r0, _r1, _r2, _r3, _r4, _r5, _r6, _r7);

// Start of the component "internal state" for the opcode callo8s


// Start of the component "internal state" for the opcode clr


// Start of the component "internal state" for the opcode cmpr


// Start of the component "internal state" for the opcode cpy


// Start of the component "internal state" for the opcode dec


// Start of the component "internal state" for the opcode i2rw


// Start of the component "internal state" for the opcode inc


// Start of the component "internal state" for the opcode j


// Start of the component "internal state" for the opcode jcmpl


// Start of the component "internal state" for the opcode jcmprio


// Start of the component "internal state" for the opcode jz


// Start of the component "internal state" for the opcode m2rri


// Start of the component "internal state" for the opcode nop


// Start of the component "internal state" for the opcode r2mri


// Start of the component "internal state" for the opcode r2o


// Start of the component "internal state" for the opcode r2vri


// Start of the component "internal state" for the opcode ret8s


// Start of the component "internal state" for the opcode ro2rri


// Start of the component "internal state" for the opcode rsets8


// Start of the component "default state" for the opcode callo8s


// Start of the component "default state" for the opcode clr


// Start of the component "default state" for the opcode cmpr


// Start of the component "default state" for the opcode cpy


// Start of the component "default state" for the opcode dec


// Start of the component "default state" for the opcode i2rw


// Start of the component "default state" for the opcode inc


// Start of the component "default state" for the opcode j


// Start of the component "default state" for the opcode jcmpl


// Start of the component "default state" for the opcode jcmprio


// Start of the component "default state" for the opcode jz


// Start of the component "default state" for the opcode m2rri


// Start of the component "default state" for the opcode nop


// Start of the component "default state" for the opcode r2mri


// Start of the component "default state" for the opcode r2o


// Start of the component "default state" for the opcode r2vri


// Start of the component "default state" for the opcode ret8s


// Start of the component "default state" for the opcode ro2rri


// Start of the component "default state" for the opcode rsets8

				case(current_instruction[15:11])

// Start of the component of the "state machine" for the opcode callo8s

					CALLO8S: begin
						case (restack0_8sSM)
						CALL1: begin
							if (!restack0_8ssenderAck) begin
							     restack0_8ssenderData[7:0] <= #1 { exec_mode, _pc + 1 };
							     restack0_8ssenderWrite <= #1 1'b1;
							     restack0_8sSM <= CALL2;
							end
						end
						CALL2: begin
							if (restack0_8ssenderAck) begin
								restack0_8ssenderWrite <= #1 1'b0;
								exec_mode <= #1 1'b0;
								_pc <= #1 current_instruction[10:4];
								$display("CALLO8S ", current_instruction[10:4]);
								restack0_8sSM <= CALL1;
							end
						end
						endcase
					end

// Start of the component of the "state machine" for the opcode clr

					CLR: begin
						case (current_instruction[10:8])
						R0 : begin
							_r0 <= #1 'b0;
							$display("CLR R0");
						end
						R1 : begin
							_r1 <= #1 'b0;
							$display("CLR R1");
						end
						R2 : begin
							_r2 <= #1 'b0;
							$display("CLR R2");
						end
						R3 : begin
							_r3 <= #1 'b0;
							$display("CLR R3");
						end
						R4 : begin
							_r4 <= #1 'b0;
							$display("CLR R4");
						end
						R5 : begin
							_r5 <= #1 'b0;
							$display("CLR R5");
						end
						R6 : begin
							_r6 <= #1 'b0;
							$display("CLR R6");
						end
						R7 : begin
							_r7 <= #1 'b0;
							$display("CLR R7");
						end
						endcase
						if (exec_mode == 1'b1) begin
							vn_state <= FETCH;
						end
						_pc <= #1 _pc + 1'b1;
					end

// Start of the component of the "state machine" for the opcode cmpr

					CMPR: begin
						case (current_instruction[10:8])
						R0 : begin
							case (current_instruction[7:5])
							R0 : begin
								if (_r0 == _r0) begin
									cmpflag <= 1'b1;
								end else begin
									cmpflag <= 1'b0;
								end
								$display("CMPR R0 R0");
							end
							R1 : begin
								if (_r1 == _r0) begin
									cmpflag <= 1'b1;
								end else begin
									cmpflag <= 1'b0;
								end
								$display("CMPR R0 R1");
							end
							R2 : begin
								if (_r2 == _r0) begin
									cmpflag <= 1'b1;
								end else begin
									cmpflag <= 1'b0;
								end
								$display("CMPR R0 R2");
							end
							R3 : begin
								if (_r3 == _r0) begin
									cmpflag <= 1'b1;
								end else begin
									cmpflag <= 1'b0;
								end
								$display("CMPR R0 R3");
							end
							R4 : begin
								if (_r4 == _r0) begin
									cmpflag <= 1'b1;
								end else begin
									cmpflag <= 1'b0;
								end
								$display("CMPR R0 R4");
							end
							R5 : begin
								if (_r5 == _r0) begin
									cmpflag <= 1'b1;
								end else begin
									cmpflag <= 1'b0;
								end
								$display("CMPR R0 R5");
							end
							R6 : begin
								if (_r6 == _r0) begin
									cmpflag <= 1'b1;
								end else begin
									cmpflag <= 1'b0;
								end
								$display("CMPR R0 R6");
							end
							R7 : begin
								if (_r7 == _r0) begin
									cmpflag <= 1'b1;
								end else begin
									cmpflag <= 1'b0;
								end
								$display("CMPR R0 R7");
							end
							endcase
						end
						R1 : begin
							case (current_instruction[7:5])
							R0 : begin
								if (_r0 == _r1) begin
									cmpflag <= 1'b1;
								end else begin
									cmpflag <= 1'b0;
								end
								$display("CMPR R1 R0");
							end
							R1 : begin
								if (_r1 == _r1) begin
									cmpflag <= 1'b1;
								end else begin
									cmpflag <= 1'b0;
								end
								$display("CMPR R1 R1");
							end
							R2 : begin
								if (_r2 == _r1) begin
									cmpflag <= 1'b1;
								end else begin
									cmpflag <= 1'b0;
								end
								$display("CMPR R1 R2");
							end
							R3 : begin
								if (_r3 == _r1) begin
									cmpflag <= 1'b1;
								end else begin
									cmpflag <= 1'b0;
								end
								$display("CMPR R1 R3");
							end
							R4 : begin
								if (_r4 == _r1) begin
									cmpflag <= 1'b1;
								end else begin
									cmpflag <= 1'b0;
								end
								$display("CMPR R1 R4");
							end
							R5 : begin
								if (_r5 == _r1) begin
									cmpflag <= 1'b1;
								end else begin
									cmpflag <= 1'b0;
								end
								$display("CMPR R1 R5");
							end
							R6 : begin
								if (_r6 == _r1) begin
									cmpflag <= 1'b1;
								end else begin
									cmpflag <= 1'b0;
								end
								$display("CMPR R1 R6");
							end
							R7 : begin
								if (_r7 == _r1) begin
									cmpflag <= 1'b1;
								end else begin
									cmpflag <= 1'b0;
								end
								$display("CMPR R1 R7");
							end
							endcase
						end
						R2 : begin
							case (current_instruction[7:5])
							R0 : begin
								if (_r0 == _r2) begin
									cmpflag <= 1'b1;
								end else begin
									cmpflag <= 1'b0;
								end
								$display("CMPR R2 R0");
							end
							R1 : begin
								if (_r1 == _r2) begin
									cmpflag <= 1'b1;
								end else begin
									cmpflag <= 1'b0;
								end
								$display("CMPR R2 R1");
							end
							R2 : begin
								if (_r2 == _r2) begin
									cmpflag <= 1'b1;
								end else begin
									cmpflag <= 1'b0;
								end
								$display("CMPR R2 R2");
							end
							R3 : begin
								if (_r3 == _r2) begin
									cmpflag <= 1'b1;
								end else begin
									cmpflag <= 1'b0;
								end
								$display("CMPR R2 R3");
							end
							R4 : begin
								if (_r4 == _r2) begin
									cmpflag <= 1'b1;
								end else begin
									cmpflag <= 1'b0;
								end
								$display("CMPR R2 R4");
							end
							R5 : begin
								if (_r5 == _r2) begin
									cmpflag <= 1'b1;
								end else begin
									cmpflag <= 1'b0;
								end
								$display("CMPR R2 R5");
							end
							R6 : begin
								if (_r6 == _r2) begin
									cmpflag <= 1'b1;
								end else begin
									cmpflag <= 1'b0;
								end
								$display("CMPR R2 R6");
							end
							R7 : begin
								if (_r7 == _r2) begin
									cmpflag <= 1'b1;
								end else begin
									cmpflag <= 1'b0;
								end
								$display("CMPR R2 R7");
							end
							endcase
						end
						R3 : begin
							case (current_instruction[7:5])
							R0 : begin
								if (_r0 == _r3) begin
									cmpflag <= 1'b1;
								end else begin
									cmpflag <= 1'b0;
								end
								$display("CMPR R3 R0");
							end
							R1 : begin
								if (_r1 == _r3) begin
									cmpflag <= 1'b1;
								end else begin
									cmpflag <= 1'b0;
								end
								$display("CMPR R3 R1");
							end
							R2 : begin
								if (_r2 == _r3) begin
									cmpflag <= 1'b1;
								end else begin
									cmpflag <= 1'b0;
								end
								$display("CMPR R3 R2");
							end
							R3 : begin
								if (_r3 == _r3) begin
									cmpflag <= 1'b1;
								end else begin
									cmpflag <= 1'b0;
								end
								$display("CMPR R3 R3");
							end
							R4 : begin
								if (_r4 == _r3) begin
									cmpflag <= 1'b1;
								end else begin
									cmpflag <= 1'b0;
								end
								$display("CMPR R3 R4");
							end
							R5 : begin
								if (_r5 == _r3) begin
									cmpflag <= 1'b1;
								end else begin
									cmpflag <= 1'b0;
								end
								$display("CMPR R3 R5");
							end
							R6 : begin
								if (_r6 == _r3) begin
									cmpflag <= 1'b1;
								end else begin
									cmpflag <= 1'b0;
								end
								$display("CMPR R3 R6");
							end
							R7 : begin
								if (_r7 == _r3) begin
									cmpflag <= 1'b1;
								end else begin
									cmpflag <= 1'b0;
								end
								$display("CMPR R3 R7");
							end
							endcase
						end
						R4 : begin
							case (current_instruction[7:5])
							R0 : begin
								if (_r0 == _r4) begin
									cmpflag <= 1'b1;
								end else begin
									cmpflag <= 1'b0;
								end
								$display("CMPR R4 R0");
							end
							R1 : begin
								if (_r1 == _r4) begin
									cmpflag <= 1'b1;
								end else begin
									cmpflag <= 1'b0;
								end
								$display("CMPR R4 R1");
							end
							R2 : begin
								if (_r2 == _r4) begin
									cmpflag <= 1'b1;
								end else begin
									cmpflag <= 1'b0;
								end
								$display("CMPR R4 R2");
							end
							R3 : begin
								if (_r3 == _r4) begin
									cmpflag <= 1'b1;
								end else begin
									cmpflag <= 1'b0;
								end
								$display("CMPR R4 R3");
							end
							R4 : begin
								if (_r4 == _r4) begin
									cmpflag <= 1'b1;
								end else begin
									cmpflag <= 1'b0;
								end
								$display("CMPR R4 R4");
							end
							R5 : begin
								if (_r5 == _r4) begin
									cmpflag <= 1'b1;
								end else begin
									cmpflag <= 1'b0;
								end
								$display("CMPR R4 R5");
							end
							R6 : begin
								if (_r6 == _r4) begin
									cmpflag <= 1'b1;
								end else begin
									cmpflag <= 1'b0;
								end
								$display("CMPR R4 R6");
							end
							R7 : begin
								if (_r7 == _r4) begin
									cmpflag <= 1'b1;
								end else begin
									cmpflag <= 1'b0;
								end
								$display("CMPR R4 R7");
							end
							endcase
						end
						R5 : begin
							case (current_instruction[7:5])
							R0 : begin
								if (_r0 == _r5) begin
									cmpflag <= 1'b1;
								end else begin
									cmpflag <= 1'b0;
								end
								$display("CMPR R5 R0");
							end
							R1 : begin
								if (_r1 == _r5) begin
									cmpflag <= 1'b1;
								end else begin
									cmpflag <= 1'b0;
								end
								$display("CMPR R5 R1");
							end
							R2 : begin
								if (_r2 == _r5) begin
									cmpflag <= 1'b1;
								end else begin
									cmpflag <= 1'b0;
								end
								$display("CMPR R5 R2");
							end
							R3 : begin
								if (_r3 == _r5) begin
									cmpflag <= 1'b1;
								end else begin
									cmpflag <= 1'b0;
								end
								$display("CMPR R5 R3");
							end
							R4 : begin
								if (_r4 == _r5) begin
									cmpflag <= 1'b1;
								end else begin
									cmpflag <= 1'b0;
								end
								$display("CMPR R5 R4");
							end
							R5 : begin
								if (_r5 == _r5) begin
									cmpflag <= 1'b1;
								end else begin
									cmpflag <= 1'b0;
								end
								$display("CMPR R5 R5");
							end
							R6 : begin
								if (_r6 == _r5) begin
									cmpflag <= 1'b1;
								end else begin
									cmpflag <= 1'b0;
								end
								$display("CMPR R5 R6");
							end
							R7 : begin
								if (_r7 == _r5) begin
									cmpflag <= 1'b1;
								end else begin
									cmpflag <= 1'b0;
								end
								$display("CMPR R5 R7");
							end
							endcase
						end
						R6 : begin
							case (current_instruction[7:5])
							R0 : begin
								if (_r0 == _r6) begin
									cmpflag <= 1'b1;
								end else begin
									cmpflag <= 1'b0;
								end
								$display("CMPR R6 R0");
							end
							R1 : begin
								if (_r1 == _r6) begin
									cmpflag <= 1'b1;
								end else begin
									cmpflag <= 1'b0;
								end
								$display("CMPR R6 R1");
							end
							R2 : begin
								if (_r2 == _r6) begin
									cmpflag <= 1'b1;
								end else begin
									cmpflag <= 1'b0;
								end
								$display("CMPR R6 R2");
							end
							R3 : begin
								if (_r3 == _r6) begin
									cmpflag <= 1'b1;
								end else begin
									cmpflag <= 1'b0;
								end
								$display("CMPR R6 R3");
							end
							R4 : begin
								if (_r4 == _r6) begin
									cmpflag <= 1'b1;
								end else begin
									cmpflag <= 1'b0;
								end
								$display("CMPR R6 R4");
							end
							R5 : begin
								if (_r5 == _r6) begin
									cmpflag <= 1'b1;
								end else begin
									cmpflag <= 1'b0;
								end
								$display("CMPR R6 R5");
							end
							R6 : begin
								if (_r6 == _r6) begin
									cmpflag <= 1'b1;
								end else begin
									cmpflag <= 1'b0;
								end
								$display("CMPR R6 R6");
							end
							R7 : begin
								if (_r7 == _r6) begin
									cmpflag <= 1'b1;
								end else begin
									cmpflag <= 1'b0;
								end
								$display("CMPR R6 R7");
							end
							endcase
						end
						R7 : begin
							case (current_instruction[7:5])
							R0 : begin
								if (_r0 == _r7) begin
									cmpflag <= 1'b1;
								end else begin
									cmpflag <= 1'b0;
								end
								$display("CMPR R7 R0");
							end
							R1 : begin
								if (_r1 == _r7) begin
									cmpflag <= 1'b1;
								end else begin
									cmpflag <= 1'b0;
								end
								$display("CMPR R7 R1");
							end
							R2 : begin
								if (_r2 == _r7) begin
									cmpflag <= 1'b1;
								end else begin
									cmpflag <= 1'b0;
								end
								$display("CMPR R7 R2");
							end
							R3 : begin
								if (_r3 == _r7) begin
									cmpflag <= 1'b1;
								end else begin
									cmpflag <= 1'b0;
								end
								$display("CMPR R7 R3");
							end
							R4 : begin
								if (_r4 == _r7) begin
									cmpflag <= 1'b1;
								end else begin
									cmpflag <= 1'b0;
								end
								$display("CMPR R7 R4");
							end
							R5 : begin
								if (_r5 == _r7) begin
									cmpflag <= 1'b1;
								end else begin
									cmpflag <= 1'b0;
								end
								$display("CMPR R7 R5");
							end
							R6 : begin
								if (_r6 == _r7) begin
									cmpflag <= 1'b1;
								end else begin
									cmpflag <= 1'b0;
								end
								$display("CMPR R7 R6");
							end
							R7 : begin
								if (_r7 == _r7) begin
									cmpflag <= 1'b1;
								end else begin
									cmpflag <= 1'b0;
								end
								$display("CMPR R7 R7");
							end
							endcase
						end
						endcase
						if (exec_mode == 1'b1) begin
							vn_state <= FETCH;
						end
						_pc <= #1 _pc + 1'b1;
					end

// Start of the component of the "state machine" for the opcode cpy

					CPY: begin
						case (current_instruction[10:8])
						R0 : begin
							case (current_instruction[7:5])
							R0 : begin
								_r0 <= #1 _r0;
								$display("CPY R0 R0");
							end
							R1 : begin
								_r0 <= #1 _r1;
								$display("CPY R0 R1");
							end
							R2 : begin
								_r0 <= #1 _r2;
								$display("CPY R0 R2");
							end
							R3 : begin
								_r0 <= #1 _r3;
								$display("CPY R0 R3");
							end
							R4 : begin
								_r0 <= #1 _r4;
								$display("CPY R0 R4");
							end
							R5 : begin
								_r0 <= #1 _r5;
								$display("CPY R0 R5");
							end
							R6 : begin
								_r0 <= #1 _r6;
								$display("CPY R0 R6");
							end
							R7 : begin
								_r0 <= #1 _r7;
								$display("CPY R0 R7");
							end
							endcase
						end
						R1 : begin
							case (current_instruction[7:5])
							R0 : begin
								_r1 <= #1 _r0;
								$display("CPY R1 R0");
							end
							R1 : begin
								_r1 <= #1 _r1;
								$display("CPY R1 R1");
							end
							R2 : begin
								_r1 <= #1 _r2;
								$display("CPY R1 R2");
							end
							R3 : begin
								_r1 <= #1 _r3;
								$display("CPY R1 R3");
							end
							R4 : begin
								_r1 <= #1 _r4;
								$display("CPY R1 R4");
							end
							R5 : begin
								_r1 <= #1 _r5;
								$display("CPY R1 R5");
							end
							R6 : begin
								_r1 <= #1 _r6;
								$display("CPY R1 R6");
							end
							R7 : begin
								_r1 <= #1 _r7;
								$display("CPY R1 R7");
							end
							endcase
						end
						R2 : begin
							case (current_instruction[7:5])
							R0 : begin
								_r2 <= #1 _r0;
								$display("CPY R2 R0");
							end
							R1 : begin
								_r2 <= #1 _r1;
								$display("CPY R2 R1");
							end
							R2 : begin
								_r2 <= #1 _r2;
								$display("CPY R2 R2");
							end
							R3 : begin
								_r2 <= #1 _r3;
								$display("CPY R2 R3");
							end
							R4 : begin
								_r2 <= #1 _r4;
								$display("CPY R2 R4");
							end
							R5 : begin
								_r2 <= #1 _r5;
								$display("CPY R2 R5");
							end
							R6 : begin
								_r2 <= #1 _r6;
								$display("CPY R2 R6");
							end
							R7 : begin
								_r2 <= #1 _r7;
								$display("CPY R2 R7");
							end
							endcase
						end
						R3 : begin
							case (current_instruction[7:5])
							R0 : begin
								_r3 <= #1 _r0;
								$display("CPY R3 R0");
							end
							R1 : begin
								_r3 <= #1 _r1;
								$display("CPY R3 R1");
							end
							R2 : begin
								_r3 <= #1 _r2;
								$display("CPY R3 R2");
							end
							R3 : begin
								_r3 <= #1 _r3;
								$display("CPY R3 R3");
							end
							R4 : begin
								_r3 <= #1 _r4;
								$display("CPY R3 R4");
							end
							R5 : begin
								_r3 <= #1 _r5;
								$display("CPY R3 R5");
							end
							R6 : begin
								_r3 <= #1 _r6;
								$display("CPY R3 R6");
							end
							R7 : begin
								_r3 <= #1 _r7;
								$display("CPY R3 R7");
							end
							endcase
						end
						R4 : begin
							case (current_instruction[7:5])
							R0 : begin
								_r4 <= #1 _r0;
								$display("CPY R4 R0");
							end
							R1 : begin
								_r4 <= #1 _r1;
								$display("CPY R4 R1");
							end
							R2 : begin
								_r4 <= #1 _r2;
								$display("CPY R4 R2");
							end
							R3 : begin
								_r4 <= #1 _r3;
								$display("CPY R4 R3");
							end
							R4 : begin
								_r4 <= #1 _r4;
								$display("CPY R4 R4");
							end
							R5 : begin
								_r4 <= #1 _r5;
								$display("CPY R4 R5");
							end
							R6 : begin
								_r4 <= #1 _r6;
								$display("CPY R4 R6");
							end
							R7 : begin
								_r4 <= #1 _r7;
								$display("CPY R4 R7");
							end
							endcase
						end
						R5 : begin
							case (current_instruction[7:5])
							R0 : begin
								_r5 <= #1 _r0;
								$display("CPY R5 R0");
							end
							R1 : begin
								_r5 <= #1 _r1;
								$display("CPY R5 R1");
							end
							R2 : begin
								_r5 <= #1 _r2;
								$display("CPY R5 R2");
							end
							R3 : begin
								_r5 <= #1 _r3;
								$display("CPY R5 R3");
							end
							R4 : begin
								_r5 <= #1 _r4;
								$display("CPY R5 R4");
							end
							R5 : begin
								_r5 <= #1 _r5;
								$display("CPY R5 R5");
							end
							R6 : begin
								_r5 <= #1 _r6;
								$display("CPY R5 R6");
							end
							R7 : begin
								_r5 <= #1 _r7;
								$display("CPY R5 R7");
							end
							endcase
						end
						R6 : begin
							case (current_instruction[7:5])
							R0 : begin
								_r6 <= #1 _r0;
								$display("CPY R6 R0");
							end
							R1 : begin
								_r6 <= #1 _r1;
								$display("CPY R6 R1");
							end
							R2 : begin
								_r6 <= #1 _r2;
								$display("CPY R6 R2");
							end
							R3 : begin
								_r6 <= #1 _r3;
								$display("CPY R6 R3");
							end
							R4 : begin
								_r6 <= #1 _r4;
								$display("CPY R6 R4");
							end
							R5 : begin
								_r6 <= #1 _r5;
								$display("CPY R6 R5");
							end
							R6 : begin
								_r6 <= #1 _r6;
								$display("CPY R6 R6");
							end
							R7 : begin
								_r6 <= #1 _r7;
								$display("CPY R6 R7");
							end
							endcase
						end
						R7 : begin
							case (current_instruction[7:5])
							R0 : begin
								_r7 <= #1 _r0;
								$display("CPY R7 R0");
							end
							R1 : begin
								_r7 <= #1 _r1;
								$display("CPY R7 R1");
							end
							R2 : begin
								_r7 <= #1 _r2;
								$display("CPY R7 R2");
							end
							R3 : begin
								_r7 <= #1 _r3;
								$display("CPY R7 R3");
							end
							R4 : begin
								_r7 <= #1 _r4;
								$display("CPY R7 R4");
							end
							R5 : begin
								_r7 <= #1 _r5;
								$display("CPY R7 R5");
							end
							R6 : begin
								_r7 <= #1 _r6;
								$display("CPY R7 R6");
							end
							R7 : begin
								_r7 <= #1 _r7;
								$display("CPY R7 R7");
							end
							endcase
						end
						endcase
						if (exec_mode == 1'b1) begin
							vn_state <= FETCH;
						end
						_pc <= #1 _pc + 1'b1;
					end

// Start of the component of the "state machine" for the opcode dec

					DEC: begin
						case (current_instruction[10:8])
						R0 : begin
							_r0 <= _r0 - 1'b1;
							$display("DEC R0");
						end
						R1 : begin
							_r1 <= _r1 - 1'b1;
							$display("DEC R1");
						end
						R2 : begin
							_r2 <= _r2 - 1'b1;
							$display("DEC R2");
						end
						R3 : begin
							_r3 <= _r3 - 1'b1;
							$display("DEC R3");
						end
						R4 : begin
							_r4 <= _r4 - 1'b1;
							$display("DEC R4");
						end
						R5 : begin
							_r5 <= _r5 - 1'b1;
							$display("DEC R5");
						end
						R6 : begin
							_r6 <= _r6 - 1'b1;
							$display("DEC R6");
						end
						R7 : begin
							_r7 <= _r7 - 1'b1;
							$display("DEC R7");
						end
						endcase
						if (exec_mode == 1'b1) begin
							vn_state <= FETCH;
						end
						_pc <= #1 _pc + 1'b1;
					end

// Start of the component of the "state machine" for the opcode i2rw

					I2RW: begin
						case (current_instruction[10:8])
						R0 : begin
							case (current_instruction[7])
							I0 : begin
								if (i0_valid)
								begin
									_r0 <= #1 i0;
									if (exec_mode == 1'b1) begin
				vn_state <= FETCH;
			end
			_pc <= #1 _pc + 1'b1;
									$display("I2RW R0 I0");
								end
							end
							endcase
						end
						R1 : begin
							case (current_instruction[7])
							I0 : begin
								if (i0_valid)
								begin
									_r1 <= #1 i0;
									if (exec_mode == 1'b1) begin
				vn_state <= FETCH;
			end
			_pc <= #1 _pc + 1'b1;
									$display("I2RW R1 I0");
								end
							end
							endcase
						end
						R2 : begin
							case (current_instruction[7])
							I0 : begin
								if (i0_valid)
								begin
									_r2 <= #1 i0;
									if (exec_mode == 1'b1) begin
				vn_state <= FETCH;
			end
			_pc <= #1 _pc + 1'b1;
									$display("I2RW R2 I0");
								end
							end
							endcase
						end
						R3 : begin
							case (current_instruction[7])
							I0 : begin
								if (i0_valid)
								begin
									_r3 <= #1 i0;
									if (exec_mode == 1'b1) begin
				vn_state <= FETCH;
			end
			_pc <= #1 _pc + 1'b1;
									$display("I2RW R3 I0");
								end
							end
							endcase
						end
						R4 : begin
							case (current_instruction[7])
							I0 : begin
								if (i0_valid)
								begin
									_r4 <= #1 i0;
									if (exec_mode == 1'b1) begin
				vn_state <= FETCH;
			end
			_pc <= #1 _pc + 1'b1;
									$display("I2RW R4 I0");
								end
							end
							endcase
						end
						R5 : begin
							case (current_instruction[7])
							I0 : begin
								if (i0_valid)
								begin
									_r5 <= #1 i0;
									if (exec_mode == 1'b1) begin
				vn_state <= FETCH;
			end
			_pc <= #1 _pc + 1'b1;
									$display("I2RW R5 I0");
								end
							end
							endcase
						end
						R6 : begin
							case (current_instruction[7])
							I0 : begin
								if (i0_valid)
								begin
									_r6 <= #1 i0;
									if (exec_mode == 1'b1) begin
				vn_state <= FETCH;
			end
			_pc <= #1 _pc + 1'b1;
									$display("I2RW R6 I0");
								end
							end
							endcase
						end
						R7 : begin
							case (current_instruction[7])
							I0 : begin
								if (i0_valid)
								begin
									_r7 <= #1 i0;
									if (exec_mode == 1'b1) begin
				vn_state <= FETCH;
			end
			_pc <= #1 _pc + 1'b1;
									$display("I2RW R7 I0");
								end
							end
							endcase
						end
						endcase
					end

// Start of the component of the "state machine" for the opcode inc

					INC: begin
						case (current_instruction[10:8])
						R0 : begin
							_r0 <= #1 _r0 + 1'b1;
							$display("INC R0");
						end
						R1 : begin
							_r1 <= #1 _r1 + 1'b1;
							$display("INC R1");
						end
						R2 : begin
							_r2 <= #1 _r2 + 1'b1;
							$display("INC R2");
						end
						R3 : begin
							_r3 <= #1 _r3 + 1'b1;
							$display("INC R3");
						end
						R4 : begin
							_r4 <= #1 _r4 + 1'b1;
							$display("INC R4");
						end
						R5 : begin
							_r5 <= #1 _r5 + 1'b1;
							$display("INC R5");
						end
						R6 : begin
							_r6 <= #1 _r6 + 1'b1;
							$display("INC R6");
						end
						R7 : begin
							_r7 <= #1 _r7 + 1'b1;
							$display("INC R7");
						end
						endcase
						if (exec_mode == 1'b1) begin
							vn_state <= FETCH;
						end
						_pc <= #1 _pc + 1'b1;
					end

// Start of the component of the "state machine" for the opcode j

					J: begin
						if (exec_mode == 1'b1) begin
							vn_state <= FETCH;
						end
						_pc <= #1 current_instruction[10:4];
						$display("J ", current_instruction[10:4]);
					end

// Start of the component of the "state machine" for the opcode jcmpl

					JCMPL: begin
						if (cmpflag == 1'b1) begin
							if (exec_mode == 1'b1) begin
								vn_state <= FETCH;
							end
							_pc <= #1 current_instruction[10:4];
						end
						else begin
							if (exec_mode == 1'b1) begin
								vn_state <= FETCH;
							end
							_pc <= #1 _pc + 1'b1;
						end
						$display("JCMPL ", current_instruction[10:4]);
					end

// Start of the component of the "state machine" for the opcode jcmprio

					JCMPRIO: begin
						case (current_instruction[10:8])
						R0 : begin
							if (cmpflag == 1'b1) begin
								exec_mode <= #1 1'b0;
								_pc <= #1 _r0;
							end else begin
								if (exec_mode == 1'b1) begin
									vn_state <= FETCH;
								end
								_pc <= #1 _pc + 1'b1;
							end
							$display("JCMPRIO R0");
						end
						R1 : begin
							if (cmpflag == 1'b1) begin
								exec_mode <= #1 1'b0;
								_pc <= #1 _r1;
							end else begin
								if (exec_mode == 1'b1) begin
									vn_state <= FETCH;
								end
								_pc <= #1 _pc + 1'b1;
							end
							$display("JCMPRIO R1");
						end
						R2 : begin
							if (cmpflag == 1'b1) begin
								exec_mode <= #1 1'b0;
								_pc <= #1 _r2;
							end else begin
								if (exec_mode == 1'b1) begin
									vn_state <= FETCH;
								end
								_pc <= #1 _pc + 1'b1;
							end
							$display("JCMPRIO R2");
						end
						R3 : begin
							if (cmpflag == 1'b1) begin
								exec_mode <= #1 1'b0;
								_pc <= #1 _r3;
							end else begin
								if (exec_mode == 1'b1) begin
									vn_state <= FETCH;
								end
								_pc <= #1 _pc + 1'b1;
							end
							$display("JCMPRIO R3");
						end
						R4 : begin
							if (cmpflag == 1'b1) begin
								exec_mode <= #1 1'b0;
								_pc <= #1 _r4;
							end else begin
								if (exec_mode == 1'b1) begin
									vn_state <= FETCH;
								end
								_pc <= #1 _pc + 1'b1;
							end
							$display("JCMPRIO R4");
						end
						R5 : begin
							if (cmpflag == 1'b1) begin
								exec_mode <= #1 1'b0;
								_pc <= #1 _r5;
							end else begin
								if (exec_mode == 1'b1) begin
									vn_state <= FETCH;
								end
								_pc <= #1 _pc + 1'b1;
							end
							$display("JCMPRIO R5");
						end
						R6 : begin
							if (cmpflag == 1'b1) begin
								exec_mode <= #1 1'b0;
								_pc <= #1 _r6;
							end else begin
								if (exec_mode == 1'b1) begin
									vn_state <= FETCH;
								end
								_pc <= #1 _pc + 1'b1;
							end
							$display("JCMPRIO R6");
						end
						R7 : begin
							if (cmpflag == 1'b1) begin
								exec_mode <= #1 1'b0;
								_pc <= #1 _r7;
							end else begin
								if (exec_mode == 1'b1) begin
									vn_state <= FETCH;
								end
								_pc <= #1 _pc + 1'b1;
							end
							$display("JCMPRIO R7");
						end
						endcase
					end

// Start of the component of the "state machine" for the opcode jz

					JZ: begin
						case (current_instruction[10:8])
							R0 : begin
								if(_r0 == 'b0) begin
								if (exec_mode == 1'b1) begin
									vn_state <= FETCH;
								end
								_pc <= #1 current_instruction[7:1];
								end
								else begin
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								$display("JZ R0 ",_r0);
							end
							R1 : begin
								if(_r1 == 'b0) begin
								if (exec_mode == 1'b1) begin
									vn_state <= FETCH;
								end
								_pc <= #1 current_instruction[7:1];
								end
								else begin
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								$display("JZ R1 ",_r1);
							end
							R2 : begin
								if(_r2 == 'b0) begin
								if (exec_mode == 1'b1) begin
									vn_state <= FETCH;
								end
								_pc <= #1 current_instruction[7:1];
								end
								else begin
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								$display("JZ R2 ",_r2);
							end
							R3 : begin
								if(_r3 == 'b0) begin
								if (exec_mode == 1'b1) begin
									vn_state <= FETCH;
								end
								_pc <= #1 current_instruction[7:1];
								end
								else begin
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								$display("JZ R3 ",_r3);
							end
							R4 : begin
								if(_r4 == 'b0) begin
								if (exec_mode == 1'b1) begin
									vn_state <= FETCH;
								end
								_pc <= #1 current_instruction[7:1];
								end
								else begin
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								$display("JZ R4 ",_r4);
							end
							R5 : begin
								if(_r5 == 'b0) begin
								if (exec_mode == 1'b1) begin
									vn_state <= FETCH;
								end
								_pc <= #1 current_instruction[7:1];
								end
								else begin
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								$display("JZ R5 ",_r5);
							end
							R6 : begin
								if(_r6 == 'b0) begin
								if (exec_mode == 1'b1) begin
									vn_state <= FETCH;
								end
								_pc <= #1 current_instruction[7:1];
								end
								else begin
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								$display("JZ R6 ",_r6);
							end
							R7 : begin
								if(_r7 == 'b0) begin
								if (exec_mode == 1'b1) begin
									vn_state <= FETCH;
								end
								_pc <= #1 current_instruction[7:1];
								end
								else begin
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								$display("JZ R7 ",_r7);
							end
						endcase
					end

// Start of the component of the "state machine" for the opcode m2rri

					M2RRI: begin
						case (current_instruction[10:8])
						R0 : begin
							case (current_instruction[7:5])
							R0 : begin
								if (state_read_mem_m2rri == 1'b1) begin
									_r0 <= #1 ram_dout;
									state_read_mem_m2rri <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									if (wait_read_mem == 1'b1) begin
										state_read_mem_m2rri <= 1'b1;
										wait_read_mem <= 1'b0;
									end
									else begin
										wait_read_mem <= 1'b1;
										addr_ram_m2rri <= #1 _r0;
									end
								end
								$display("M2RRI R0 ",_r0);
							end
							R1 : begin
								if (state_read_mem_m2rri == 1'b1) begin
									_r0 <= #1 ram_dout;
									state_read_mem_m2rri <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									if (wait_read_mem == 1'b1) begin
										state_read_mem_m2rri <= 1'b1;
										wait_read_mem <= 1'b0;
									end
									else begin
										wait_read_mem <= 1'b1;
										addr_ram_m2rri <= #1 _r1;
									end
								end
								$display("M2RRI R0 ",_r1);
							end
							R2 : begin
								if (state_read_mem_m2rri == 1'b1) begin
									_r0 <= #1 ram_dout;
									state_read_mem_m2rri <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									if (wait_read_mem == 1'b1) begin
										state_read_mem_m2rri <= 1'b1;
										wait_read_mem <= 1'b0;
									end
									else begin
										wait_read_mem <= 1'b1;
										addr_ram_m2rri <= #1 _r2;
									end
								end
								$display("M2RRI R0 ",_r2);
							end
							R3 : begin
								if (state_read_mem_m2rri == 1'b1) begin
									_r0 <= #1 ram_dout;
									state_read_mem_m2rri <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									if (wait_read_mem == 1'b1) begin
										state_read_mem_m2rri <= 1'b1;
										wait_read_mem <= 1'b0;
									end
									else begin
										wait_read_mem <= 1'b1;
										addr_ram_m2rri <= #1 _r3;
									end
								end
								$display("M2RRI R0 ",_r3);
							end
							R4 : begin
								if (state_read_mem_m2rri == 1'b1) begin
									_r0 <= #1 ram_dout;
									state_read_mem_m2rri <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									if (wait_read_mem == 1'b1) begin
										state_read_mem_m2rri <= 1'b1;
										wait_read_mem <= 1'b0;
									end
									else begin
										wait_read_mem <= 1'b1;
										addr_ram_m2rri <= #1 _r4;
									end
								end
								$display("M2RRI R0 ",_r4);
							end
							R5 : begin
								if (state_read_mem_m2rri == 1'b1) begin
									_r0 <= #1 ram_dout;
									state_read_mem_m2rri <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									if (wait_read_mem == 1'b1) begin
										state_read_mem_m2rri <= 1'b1;
										wait_read_mem <= 1'b0;
									end
									else begin
										wait_read_mem <= 1'b1;
										addr_ram_m2rri <= #1 _r5;
									end
								end
								$display("M2RRI R0 ",_r5);
							end
							R6 : begin
								if (state_read_mem_m2rri == 1'b1) begin
									_r0 <= #1 ram_dout;
									state_read_mem_m2rri <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									if (wait_read_mem == 1'b1) begin
										state_read_mem_m2rri <= 1'b1;
										wait_read_mem <= 1'b0;
									end
									else begin
										wait_read_mem <= 1'b1;
										addr_ram_m2rri <= #1 _r6;
									end
								end
								$display("M2RRI R0 ",_r6);
							end
							R7 : begin
								if (state_read_mem_m2rri == 1'b1) begin
									_r0 <= #1 ram_dout;
									state_read_mem_m2rri <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									if (wait_read_mem == 1'b1) begin
										state_read_mem_m2rri <= 1'b1;
										wait_read_mem <= 1'b0;
									end
									else begin
										wait_read_mem <= 1'b1;
										addr_ram_m2rri <= #1 _r7;
									end
								end
								$display("M2RRI R0 ",_r7);
							end
							endcase
						end
						R1 : begin
							case (current_instruction[7:5])
							R0 : begin
								if (state_read_mem_m2rri == 1'b1) begin
									_r1 <= #1 ram_dout;
									state_read_mem_m2rri <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									if (wait_read_mem == 1'b1) begin
										state_read_mem_m2rri <= 1'b1;
										wait_read_mem <= 1'b0;
									end
									else begin
										wait_read_mem <= 1'b1;
										addr_ram_m2rri <= #1 _r0;
									end
								end
								$display("M2RRI R1 ",_r0);
							end
							R1 : begin
								if (state_read_mem_m2rri == 1'b1) begin
									_r1 <= #1 ram_dout;
									state_read_mem_m2rri <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									if (wait_read_mem == 1'b1) begin
										state_read_mem_m2rri <= 1'b1;
										wait_read_mem <= 1'b0;
									end
									else begin
										wait_read_mem <= 1'b1;
										addr_ram_m2rri <= #1 _r1;
									end
								end
								$display("M2RRI R1 ",_r1);
							end
							R2 : begin
								if (state_read_mem_m2rri == 1'b1) begin
									_r1 <= #1 ram_dout;
									state_read_mem_m2rri <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									if (wait_read_mem == 1'b1) begin
										state_read_mem_m2rri <= 1'b1;
										wait_read_mem <= 1'b0;
									end
									else begin
										wait_read_mem <= 1'b1;
										addr_ram_m2rri <= #1 _r2;
									end
								end
								$display("M2RRI R1 ",_r2);
							end
							R3 : begin
								if (state_read_mem_m2rri == 1'b1) begin
									_r1 <= #1 ram_dout;
									state_read_mem_m2rri <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									if (wait_read_mem == 1'b1) begin
										state_read_mem_m2rri <= 1'b1;
										wait_read_mem <= 1'b0;
									end
									else begin
										wait_read_mem <= 1'b1;
										addr_ram_m2rri <= #1 _r3;
									end
								end
								$display("M2RRI R1 ",_r3);
							end
							R4 : begin
								if (state_read_mem_m2rri == 1'b1) begin
									_r1 <= #1 ram_dout;
									state_read_mem_m2rri <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									if (wait_read_mem == 1'b1) begin
										state_read_mem_m2rri <= 1'b1;
										wait_read_mem <= 1'b0;
									end
									else begin
										wait_read_mem <= 1'b1;
										addr_ram_m2rri <= #1 _r4;
									end
								end
								$display("M2RRI R1 ",_r4);
							end
							R5 : begin
								if (state_read_mem_m2rri == 1'b1) begin
									_r1 <= #1 ram_dout;
									state_read_mem_m2rri <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									if (wait_read_mem == 1'b1) begin
										state_read_mem_m2rri <= 1'b1;
										wait_read_mem <= 1'b0;
									end
									else begin
										wait_read_mem <= 1'b1;
										addr_ram_m2rri <= #1 _r5;
									end
								end
								$display("M2RRI R1 ",_r5);
							end
							R6 : begin
								if (state_read_mem_m2rri == 1'b1) begin
									_r1 <= #1 ram_dout;
									state_read_mem_m2rri <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									if (wait_read_mem == 1'b1) begin
										state_read_mem_m2rri <= 1'b1;
										wait_read_mem <= 1'b0;
									end
									else begin
										wait_read_mem <= 1'b1;
										addr_ram_m2rri <= #1 _r6;
									end
								end
								$display("M2RRI R1 ",_r6);
							end
							R7 : begin
								if (state_read_mem_m2rri == 1'b1) begin
									_r1 <= #1 ram_dout;
									state_read_mem_m2rri <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									if (wait_read_mem == 1'b1) begin
										state_read_mem_m2rri <= 1'b1;
										wait_read_mem <= 1'b0;
									end
									else begin
										wait_read_mem <= 1'b1;
										addr_ram_m2rri <= #1 _r7;
									end
								end
								$display("M2RRI R1 ",_r7);
							end
							endcase
						end
						R2 : begin
							case (current_instruction[7:5])
							R0 : begin
								if (state_read_mem_m2rri == 1'b1) begin
									_r2 <= #1 ram_dout;
									state_read_mem_m2rri <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									if (wait_read_mem == 1'b1) begin
										state_read_mem_m2rri <= 1'b1;
										wait_read_mem <= 1'b0;
									end
									else begin
										wait_read_mem <= 1'b1;
										addr_ram_m2rri <= #1 _r0;
									end
								end
								$display("M2RRI R2 ",_r0);
							end
							R1 : begin
								if (state_read_mem_m2rri == 1'b1) begin
									_r2 <= #1 ram_dout;
									state_read_mem_m2rri <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									if (wait_read_mem == 1'b1) begin
										state_read_mem_m2rri <= 1'b1;
										wait_read_mem <= 1'b0;
									end
									else begin
										wait_read_mem <= 1'b1;
										addr_ram_m2rri <= #1 _r1;
									end
								end
								$display("M2RRI R2 ",_r1);
							end
							R2 : begin
								if (state_read_mem_m2rri == 1'b1) begin
									_r2 <= #1 ram_dout;
									state_read_mem_m2rri <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									if (wait_read_mem == 1'b1) begin
										state_read_mem_m2rri <= 1'b1;
										wait_read_mem <= 1'b0;
									end
									else begin
										wait_read_mem <= 1'b1;
										addr_ram_m2rri <= #1 _r2;
									end
								end
								$display("M2RRI R2 ",_r2);
							end
							R3 : begin
								if (state_read_mem_m2rri == 1'b1) begin
									_r2 <= #1 ram_dout;
									state_read_mem_m2rri <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									if (wait_read_mem == 1'b1) begin
										state_read_mem_m2rri <= 1'b1;
										wait_read_mem <= 1'b0;
									end
									else begin
										wait_read_mem <= 1'b1;
										addr_ram_m2rri <= #1 _r3;
									end
								end
								$display("M2RRI R2 ",_r3);
							end
							R4 : begin
								if (state_read_mem_m2rri == 1'b1) begin
									_r2 <= #1 ram_dout;
									state_read_mem_m2rri <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									if (wait_read_mem == 1'b1) begin
										state_read_mem_m2rri <= 1'b1;
										wait_read_mem <= 1'b0;
									end
									else begin
										wait_read_mem <= 1'b1;
										addr_ram_m2rri <= #1 _r4;
									end
								end
								$display("M2RRI R2 ",_r4);
							end
							R5 : begin
								if (state_read_mem_m2rri == 1'b1) begin
									_r2 <= #1 ram_dout;
									state_read_mem_m2rri <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									if (wait_read_mem == 1'b1) begin
										state_read_mem_m2rri <= 1'b1;
										wait_read_mem <= 1'b0;
									end
									else begin
										wait_read_mem <= 1'b1;
										addr_ram_m2rri <= #1 _r5;
									end
								end
								$display("M2RRI R2 ",_r5);
							end
							R6 : begin
								if (state_read_mem_m2rri == 1'b1) begin
									_r2 <= #1 ram_dout;
									state_read_mem_m2rri <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									if (wait_read_mem == 1'b1) begin
										state_read_mem_m2rri <= 1'b1;
										wait_read_mem <= 1'b0;
									end
									else begin
										wait_read_mem <= 1'b1;
										addr_ram_m2rri <= #1 _r6;
									end
								end
								$display("M2RRI R2 ",_r6);
							end
							R7 : begin
								if (state_read_mem_m2rri == 1'b1) begin
									_r2 <= #1 ram_dout;
									state_read_mem_m2rri <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									if (wait_read_mem == 1'b1) begin
										state_read_mem_m2rri <= 1'b1;
										wait_read_mem <= 1'b0;
									end
									else begin
										wait_read_mem <= 1'b1;
										addr_ram_m2rri <= #1 _r7;
									end
								end
								$display("M2RRI R2 ",_r7);
							end
							endcase
						end
						R3 : begin
							case (current_instruction[7:5])
							R0 : begin
								if (state_read_mem_m2rri == 1'b1) begin
									_r3 <= #1 ram_dout;
									state_read_mem_m2rri <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									if (wait_read_mem == 1'b1) begin
										state_read_mem_m2rri <= 1'b1;
										wait_read_mem <= 1'b0;
									end
									else begin
										wait_read_mem <= 1'b1;
										addr_ram_m2rri <= #1 _r0;
									end
								end
								$display("M2RRI R3 ",_r0);
							end
							R1 : begin
								if (state_read_mem_m2rri == 1'b1) begin
									_r3 <= #1 ram_dout;
									state_read_mem_m2rri <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									if (wait_read_mem == 1'b1) begin
										state_read_mem_m2rri <= 1'b1;
										wait_read_mem <= 1'b0;
									end
									else begin
										wait_read_mem <= 1'b1;
										addr_ram_m2rri <= #1 _r1;
									end
								end
								$display("M2RRI R3 ",_r1);
							end
							R2 : begin
								if (state_read_mem_m2rri == 1'b1) begin
									_r3 <= #1 ram_dout;
									state_read_mem_m2rri <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									if (wait_read_mem == 1'b1) begin
										state_read_mem_m2rri <= 1'b1;
										wait_read_mem <= 1'b0;
									end
									else begin
										wait_read_mem <= 1'b1;
										addr_ram_m2rri <= #1 _r2;
									end
								end
								$display("M2RRI R3 ",_r2);
							end
							R3 : begin
								if (state_read_mem_m2rri == 1'b1) begin
									_r3 <= #1 ram_dout;
									state_read_mem_m2rri <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									if (wait_read_mem == 1'b1) begin
										state_read_mem_m2rri <= 1'b1;
										wait_read_mem <= 1'b0;
									end
									else begin
										wait_read_mem <= 1'b1;
										addr_ram_m2rri <= #1 _r3;
									end
								end
								$display("M2RRI R3 ",_r3);
							end
							R4 : begin
								if (state_read_mem_m2rri == 1'b1) begin
									_r3 <= #1 ram_dout;
									state_read_mem_m2rri <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									if (wait_read_mem == 1'b1) begin
										state_read_mem_m2rri <= 1'b1;
										wait_read_mem <= 1'b0;
									end
									else begin
										wait_read_mem <= 1'b1;
										addr_ram_m2rri <= #1 _r4;
									end
								end
								$display("M2RRI R3 ",_r4);
							end
							R5 : begin
								if (state_read_mem_m2rri == 1'b1) begin
									_r3 <= #1 ram_dout;
									state_read_mem_m2rri <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									if (wait_read_mem == 1'b1) begin
										state_read_mem_m2rri <= 1'b1;
										wait_read_mem <= 1'b0;
									end
									else begin
										wait_read_mem <= 1'b1;
										addr_ram_m2rri <= #1 _r5;
									end
								end
								$display("M2RRI R3 ",_r5);
							end
							R6 : begin
								if (state_read_mem_m2rri == 1'b1) begin
									_r3 <= #1 ram_dout;
									state_read_mem_m2rri <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									if (wait_read_mem == 1'b1) begin
										state_read_mem_m2rri <= 1'b1;
										wait_read_mem <= 1'b0;
									end
									else begin
										wait_read_mem <= 1'b1;
										addr_ram_m2rri <= #1 _r6;
									end
								end
								$display("M2RRI R3 ",_r6);
							end
							R7 : begin
								if (state_read_mem_m2rri == 1'b1) begin
									_r3 <= #1 ram_dout;
									state_read_mem_m2rri <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									if (wait_read_mem == 1'b1) begin
										state_read_mem_m2rri <= 1'b1;
										wait_read_mem <= 1'b0;
									end
									else begin
										wait_read_mem <= 1'b1;
										addr_ram_m2rri <= #1 _r7;
									end
								end
								$display("M2RRI R3 ",_r7);
							end
							endcase
						end
						R4 : begin
							case (current_instruction[7:5])
							R0 : begin
								if (state_read_mem_m2rri == 1'b1) begin
									_r4 <= #1 ram_dout;
									state_read_mem_m2rri <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									if (wait_read_mem == 1'b1) begin
										state_read_mem_m2rri <= 1'b1;
										wait_read_mem <= 1'b0;
									end
									else begin
										wait_read_mem <= 1'b1;
										addr_ram_m2rri <= #1 _r0;
									end
								end
								$display("M2RRI R4 ",_r0);
							end
							R1 : begin
								if (state_read_mem_m2rri == 1'b1) begin
									_r4 <= #1 ram_dout;
									state_read_mem_m2rri <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									if (wait_read_mem == 1'b1) begin
										state_read_mem_m2rri <= 1'b1;
										wait_read_mem <= 1'b0;
									end
									else begin
										wait_read_mem <= 1'b1;
										addr_ram_m2rri <= #1 _r1;
									end
								end
								$display("M2RRI R4 ",_r1);
							end
							R2 : begin
								if (state_read_mem_m2rri == 1'b1) begin
									_r4 <= #1 ram_dout;
									state_read_mem_m2rri <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									if (wait_read_mem == 1'b1) begin
										state_read_mem_m2rri <= 1'b1;
										wait_read_mem <= 1'b0;
									end
									else begin
										wait_read_mem <= 1'b1;
										addr_ram_m2rri <= #1 _r2;
									end
								end
								$display("M2RRI R4 ",_r2);
							end
							R3 : begin
								if (state_read_mem_m2rri == 1'b1) begin
									_r4 <= #1 ram_dout;
									state_read_mem_m2rri <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									if (wait_read_mem == 1'b1) begin
										state_read_mem_m2rri <= 1'b1;
										wait_read_mem <= 1'b0;
									end
									else begin
										wait_read_mem <= 1'b1;
										addr_ram_m2rri <= #1 _r3;
									end
								end
								$display("M2RRI R4 ",_r3);
							end
							R4 : begin
								if (state_read_mem_m2rri == 1'b1) begin
									_r4 <= #1 ram_dout;
									state_read_mem_m2rri <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									if (wait_read_mem == 1'b1) begin
										state_read_mem_m2rri <= 1'b1;
										wait_read_mem <= 1'b0;
									end
									else begin
										wait_read_mem <= 1'b1;
										addr_ram_m2rri <= #1 _r4;
									end
								end
								$display("M2RRI R4 ",_r4);
							end
							R5 : begin
								if (state_read_mem_m2rri == 1'b1) begin
									_r4 <= #1 ram_dout;
									state_read_mem_m2rri <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									if (wait_read_mem == 1'b1) begin
										state_read_mem_m2rri <= 1'b1;
										wait_read_mem <= 1'b0;
									end
									else begin
										wait_read_mem <= 1'b1;
										addr_ram_m2rri <= #1 _r5;
									end
								end
								$display("M2RRI R4 ",_r5);
							end
							R6 : begin
								if (state_read_mem_m2rri == 1'b1) begin
									_r4 <= #1 ram_dout;
									state_read_mem_m2rri <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									if (wait_read_mem == 1'b1) begin
										state_read_mem_m2rri <= 1'b1;
										wait_read_mem <= 1'b0;
									end
									else begin
										wait_read_mem <= 1'b1;
										addr_ram_m2rri <= #1 _r6;
									end
								end
								$display("M2RRI R4 ",_r6);
							end
							R7 : begin
								if (state_read_mem_m2rri == 1'b1) begin
									_r4 <= #1 ram_dout;
									state_read_mem_m2rri <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									if (wait_read_mem == 1'b1) begin
										state_read_mem_m2rri <= 1'b1;
										wait_read_mem <= 1'b0;
									end
									else begin
										wait_read_mem <= 1'b1;
										addr_ram_m2rri <= #1 _r7;
									end
								end
								$display("M2RRI R4 ",_r7);
							end
							endcase
						end
						R5 : begin
							case (current_instruction[7:5])
							R0 : begin
								if (state_read_mem_m2rri == 1'b1) begin
									_r5 <= #1 ram_dout;
									state_read_mem_m2rri <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									if (wait_read_mem == 1'b1) begin
										state_read_mem_m2rri <= 1'b1;
										wait_read_mem <= 1'b0;
									end
									else begin
										wait_read_mem <= 1'b1;
										addr_ram_m2rri <= #1 _r0;
									end
								end
								$display("M2RRI R5 ",_r0);
							end
							R1 : begin
								if (state_read_mem_m2rri == 1'b1) begin
									_r5 <= #1 ram_dout;
									state_read_mem_m2rri <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									if (wait_read_mem == 1'b1) begin
										state_read_mem_m2rri <= 1'b1;
										wait_read_mem <= 1'b0;
									end
									else begin
										wait_read_mem <= 1'b1;
										addr_ram_m2rri <= #1 _r1;
									end
								end
								$display("M2RRI R5 ",_r1);
							end
							R2 : begin
								if (state_read_mem_m2rri == 1'b1) begin
									_r5 <= #1 ram_dout;
									state_read_mem_m2rri <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									if (wait_read_mem == 1'b1) begin
										state_read_mem_m2rri <= 1'b1;
										wait_read_mem <= 1'b0;
									end
									else begin
										wait_read_mem <= 1'b1;
										addr_ram_m2rri <= #1 _r2;
									end
								end
								$display("M2RRI R5 ",_r2);
							end
							R3 : begin
								if (state_read_mem_m2rri == 1'b1) begin
									_r5 <= #1 ram_dout;
									state_read_mem_m2rri <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									if (wait_read_mem == 1'b1) begin
										state_read_mem_m2rri <= 1'b1;
										wait_read_mem <= 1'b0;
									end
									else begin
										wait_read_mem <= 1'b1;
										addr_ram_m2rri <= #1 _r3;
									end
								end
								$display("M2RRI R5 ",_r3);
							end
							R4 : begin
								if (state_read_mem_m2rri == 1'b1) begin
									_r5 <= #1 ram_dout;
									state_read_mem_m2rri <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									if (wait_read_mem == 1'b1) begin
										state_read_mem_m2rri <= 1'b1;
										wait_read_mem <= 1'b0;
									end
									else begin
										wait_read_mem <= 1'b1;
										addr_ram_m2rri <= #1 _r4;
									end
								end
								$display("M2RRI R5 ",_r4);
							end
							R5 : begin
								if (state_read_mem_m2rri == 1'b1) begin
									_r5 <= #1 ram_dout;
									state_read_mem_m2rri <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									if (wait_read_mem == 1'b1) begin
										state_read_mem_m2rri <= 1'b1;
										wait_read_mem <= 1'b0;
									end
									else begin
										wait_read_mem <= 1'b1;
										addr_ram_m2rri <= #1 _r5;
									end
								end
								$display("M2RRI R5 ",_r5);
							end
							R6 : begin
								if (state_read_mem_m2rri == 1'b1) begin
									_r5 <= #1 ram_dout;
									state_read_mem_m2rri <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									if (wait_read_mem == 1'b1) begin
										state_read_mem_m2rri <= 1'b1;
										wait_read_mem <= 1'b0;
									end
									else begin
										wait_read_mem <= 1'b1;
										addr_ram_m2rri <= #1 _r6;
									end
								end
								$display("M2RRI R5 ",_r6);
							end
							R7 : begin
								if (state_read_mem_m2rri == 1'b1) begin
									_r5 <= #1 ram_dout;
									state_read_mem_m2rri <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									if (wait_read_mem == 1'b1) begin
										state_read_mem_m2rri <= 1'b1;
										wait_read_mem <= 1'b0;
									end
									else begin
										wait_read_mem <= 1'b1;
										addr_ram_m2rri <= #1 _r7;
									end
								end
								$display("M2RRI R5 ",_r7);
							end
							endcase
						end
						R6 : begin
							case (current_instruction[7:5])
							R0 : begin
								if (state_read_mem_m2rri == 1'b1) begin
									_r6 <= #1 ram_dout;
									state_read_mem_m2rri <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									if (wait_read_mem == 1'b1) begin
										state_read_mem_m2rri <= 1'b1;
										wait_read_mem <= 1'b0;
									end
									else begin
										wait_read_mem <= 1'b1;
										addr_ram_m2rri <= #1 _r0;
									end
								end
								$display("M2RRI R6 ",_r0);
							end
							R1 : begin
								if (state_read_mem_m2rri == 1'b1) begin
									_r6 <= #1 ram_dout;
									state_read_mem_m2rri <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									if (wait_read_mem == 1'b1) begin
										state_read_mem_m2rri <= 1'b1;
										wait_read_mem <= 1'b0;
									end
									else begin
										wait_read_mem <= 1'b1;
										addr_ram_m2rri <= #1 _r1;
									end
								end
								$display("M2RRI R6 ",_r1);
							end
							R2 : begin
								if (state_read_mem_m2rri == 1'b1) begin
									_r6 <= #1 ram_dout;
									state_read_mem_m2rri <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									if (wait_read_mem == 1'b1) begin
										state_read_mem_m2rri <= 1'b1;
										wait_read_mem <= 1'b0;
									end
									else begin
										wait_read_mem <= 1'b1;
										addr_ram_m2rri <= #1 _r2;
									end
								end
								$display("M2RRI R6 ",_r2);
							end
							R3 : begin
								if (state_read_mem_m2rri == 1'b1) begin
									_r6 <= #1 ram_dout;
									state_read_mem_m2rri <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									if (wait_read_mem == 1'b1) begin
										state_read_mem_m2rri <= 1'b1;
										wait_read_mem <= 1'b0;
									end
									else begin
										wait_read_mem <= 1'b1;
										addr_ram_m2rri <= #1 _r3;
									end
								end
								$display("M2RRI R6 ",_r3);
							end
							R4 : begin
								if (state_read_mem_m2rri == 1'b1) begin
									_r6 <= #1 ram_dout;
									state_read_mem_m2rri <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									if (wait_read_mem == 1'b1) begin
										state_read_mem_m2rri <= 1'b1;
										wait_read_mem <= 1'b0;
									end
									else begin
										wait_read_mem <= 1'b1;
										addr_ram_m2rri <= #1 _r4;
									end
								end
								$display("M2RRI R6 ",_r4);
							end
							R5 : begin
								if (state_read_mem_m2rri == 1'b1) begin
									_r6 <= #1 ram_dout;
									state_read_mem_m2rri <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									if (wait_read_mem == 1'b1) begin
										state_read_mem_m2rri <= 1'b1;
										wait_read_mem <= 1'b0;
									end
									else begin
										wait_read_mem <= 1'b1;
										addr_ram_m2rri <= #1 _r5;
									end
								end
								$display("M2RRI R6 ",_r5);
							end
							R6 : begin
								if (state_read_mem_m2rri == 1'b1) begin
									_r6 <= #1 ram_dout;
									state_read_mem_m2rri <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									if (wait_read_mem == 1'b1) begin
										state_read_mem_m2rri <= 1'b1;
										wait_read_mem <= 1'b0;
									end
									else begin
										wait_read_mem <= 1'b1;
										addr_ram_m2rri <= #1 _r6;
									end
								end
								$display("M2RRI R6 ",_r6);
							end
							R7 : begin
								if (state_read_mem_m2rri == 1'b1) begin
									_r6 <= #1 ram_dout;
									state_read_mem_m2rri <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									if (wait_read_mem == 1'b1) begin
										state_read_mem_m2rri <= 1'b1;
										wait_read_mem <= 1'b0;
									end
									else begin
										wait_read_mem <= 1'b1;
										addr_ram_m2rri <= #1 _r7;
									end
								end
								$display("M2RRI R6 ",_r7);
							end
							endcase
						end
						R7 : begin
							case (current_instruction[7:5])
							R0 : begin
								if (state_read_mem_m2rri == 1'b1) begin
									_r7 <= #1 ram_dout;
									state_read_mem_m2rri <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									if (wait_read_mem == 1'b1) begin
										state_read_mem_m2rri <= 1'b1;
										wait_read_mem <= 1'b0;
									end
									else begin
										wait_read_mem <= 1'b1;
										addr_ram_m2rri <= #1 _r0;
									end
								end
								$display("M2RRI R7 ",_r0);
							end
							R1 : begin
								if (state_read_mem_m2rri == 1'b1) begin
									_r7 <= #1 ram_dout;
									state_read_mem_m2rri <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									if (wait_read_mem == 1'b1) begin
										state_read_mem_m2rri <= 1'b1;
										wait_read_mem <= 1'b0;
									end
									else begin
										wait_read_mem <= 1'b1;
										addr_ram_m2rri <= #1 _r1;
									end
								end
								$display("M2RRI R7 ",_r1);
							end
							R2 : begin
								if (state_read_mem_m2rri == 1'b1) begin
									_r7 <= #1 ram_dout;
									state_read_mem_m2rri <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									if (wait_read_mem == 1'b1) begin
										state_read_mem_m2rri <= 1'b1;
										wait_read_mem <= 1'b0;
									end
									else begin
										wait_read_mem <= 1'b1;
										addr_ram_m2rri <= #1 _r2;
									end
								end
								$display("M2RRI R7 ",_r2);
							end
							R3 : begin
								if (state_read_mem_m2rri == 1'b1) begin
									_r7 <= #1 ram_dout;
									state_read_mem_m2rri <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									if (wait_read_mem == 1'b1) begin
										state_read_mem_m2rri <= 1'b1;
										wait_read_mem <= 1'b0;
									end
									else begin
										wait_read_mem <= 1'b1;
										addr_ram_m2rri <= #1 _r3;
									end
								end
								$display("M2RRI R7 ",_r3);
							end
							R4 : begin
								if (state_read_mem_m2rri == 1'b1) begin
									_r7 <= #1 ram_dout;
									state_read_mem_m2rri <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									if (wait_read_mem == 1'b1) begin
										state_read_mem_m2rri <= 1'b1;
										wait_read_mem <= 1'b0;
									end
									else begin
										wait_read_mem <= 1'b1;
										addr_ram_m2rri <= #1 _r4;
									end
								end
								$display("M2RRI R7 ",_r4);
							end
							R5 : begin
								if (state_read_mem_m2rri == 1'b1) begin
									_r7 <= #1 ram_dout;
									state_read_mem_m2rri <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									if (wait_read_mem == 1'b1) begin
										state_read_mem_m2rri <= 1'b1;
										wait_read_mem <= 1'b0;
									end
									else begin
										wait_read_mem <= 1'b1;
										addr_ram_m2rri <= #1 _r5;
									end
								end
								$display("M2RRI R7 ",_r5);
							end
							R6 : begin
								if (state_read_mem_m2rri == 1'b1) begin
									_r7 <= #1 ram_dout;
									state_read_mem_m2rri <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									if (wait_read_mem == 1'b1) begin
										state_read_mem_m2rri <= 1'b1;
										wait_read_mem <= 1'b0;
									end
									else begin
										wait_read_mem <= 1'b1;
										addr_ram_m2rri <= #1 _r6;
									end
								end
								$display("M2RRI R7 ",_r6);
							end
							R7 : begin
								if (state_read_mem_m2rri == 1'b1) begin
									_r7 <= #1 ram_dout;
									state_read_mem_m2rri <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									if (wait_read_mem == 1'b1) begin
										state_read_mem_m2rri <= 1'b1;
										wait_read_mem <= 1'b0;
									end
									else begin
										wait_read_mem <= 1'b1;
										addr_ram_m2rri <= #1 _r7;
									end
								end
								$display("M2RRI R7 ",_r7);
							end
							endcase
						end
						endcase
					end

// Start of the component of the "state machine" for the opcode nop

					NOP: begin
						$display("NOP");
						if (exec_mode == 1'b1) begin
							vn_state <= FETCH;
						end
						_pc <= #1 _pc + 1'b1;
					end

// Start of the component of the "state machine" for the opcode r2mri

					R2MRI: begin
						if (wr_int_ram == 0) begin
							wr_int_ram <= #1 1'b1;
							case (current_instruction[10:8])
								R0 : begin
									case (current_instruction[7:5])
										R0 : begin
											addr_ram_to_mem <= _r0;
											ram_din_i <= _r0;
											$display("R2MRI R0 ",_r0);
										end
										R1 : begin
											addr_ram_to_mem <= _r1;
											ram_din_i <= _r0;
											$display("R2MRI R0 ",_r1);
										end
										R2 : begin
											addr_ram_to_mem <= _r2;
											ram_din_i <= _r0;
											$display("R2MRI R0 ",_r2);
										end
										R3 : begin
											addr_ram_to_mem <= _r3;
											ram_din_i <= _r0;
											$display("R2MRI R0 ",_r3);
										end
										R4 : begin
											addr_ram_to_mem <= _r4;
											ram_din_i <= _r0;
											$display("R2MRI R0 ",_r4);
										end
										R5 : begin
											addr_ram_to_mem <= _r5;
											ram_din_i <= _r0;
											$display("R2MRI R0 ",_r5);
										end
										R6 : begin
											addr_ram_to_mem <= _r6;
											ram_din_i <= _r0;
											$display("R2MRI R0 ",_r6);
										end
										R7 : begin
											addr_ram_to_mem <= _r7;
											ram_din_i <= _r0;
											$display("R2MRI R0 ",_r7);
										end
									endcase
								end
								R1 : begin
									case (current_instruction[7:5])
										R0 : begin
											addr_ram_to_mem <= _r0;
											ram_din_i <= _r1;
											$display("R2MRI R1 ",_r0);
										end
										R1 : begin
											addr_ram_to_mem <= _r1;
											ram_din_i <= _r1;
											$display("R2MRI R1 ",_r1);
										end
										R2 : begin
											addr_ram_to_mem <= _r2;
											ram_din_i <= _r1;
											$display("R2MRI R1 ",_r2);
										end
										R3 : begin
											addr_ram_to_mem <= _r3;
											ram_din_i <= _r1;
											$display("R2MRI R1 ",_r3);
										end
										R4 : begin
											addr_ram_to_mem <= _r4;
											ram_din_i <= _r1;
											$display("R2MRI R1 ",_r4);
										end
										R5 : begin
											addr_ram_to_mem <= _r5;
											ram_din_i <= _r1;
											$display("R2MRI R1 ",_r5);
										end
										R6 : begin
											addr_ram_to_mem <= _r6;
											ram_din_i <= _r1;
											$display("R2MRI R1 ",_r6);
										end
										R7 : begin
											addr_ram_to_mem <= _r7;
											ram_din_i <= _r1;
											$display("R2MRI R1 ",_r7);
										end
									endcase
								end
								R2 : begin
									case (current_instruction[7:5])
										R0 : begin
											addr_ram_to_mem <= _r0;
											ram_din_i <= _r2;
											$display("R2MRI R2 ",_r0);
										end
										R1 : begin
											addr_ram_to_mem <= _r1;
											ram_din_i <= _r2;
											$display("R2MRI R2 ",_r1);
										end
										R2 : begin
											addr_ram_to_mem <= _r2;
											ram_din_i <= _r2;
											$display("R2MRI R2 ",_r2);
										end
										R3 : begin
											addr_ram_to_mem <= _r3;
											ram_din_i <= _r2;
											$display("R2MRI R2 ",_r3);
										end
										R4 : begin
											addr_ram_to_mem <= _r4;
											ram_din_i <= _r2;
											$display("R2MRI R2 ",_r4);
										end
										R5 : begin
											addr_ram_to_mem <= _r5;
											ram_din_i <= _r2;
											$display("R2MRI R2 ",_r5);
										end
										R6 : begin
											addr_ram_to_mem <= _r6;
											ram_din_i <= _r2;
											$display("R2MRI R2 ",_r6);
										end
										R7 : begin
											addr_ram_to_mem <= _r7;
											ram_din_i <= _r2;
											$display("R2MRI R2 ",_r7);
										end
									endcase
								end
								R3 : begin
									case (current_instruction[7:5])
										R0 : begin
											addr_ram_to_mem <= _r0;
											ram_din_i <= _r3;
											$display("R2MRI R3 ",_r0);
										end
										R1 : begin
											addr_ram_to_mem <= _r1;
											ram_din_i <= _r3;
											$display("R2MRI R3 ",_r1);
										end
										R2 : begin
											addr_ram_to_mem <= _r2;
											ram_din_i <= _r3;
											$display("R2MRI R3 ",_r2);
										end
										R3 : begin
											addr_ram_to_mem <= _r3;
											ram_din_i <= _r3;
											$display("R2MRI R3 ",_r3);
										end
										R4 : begin
											addr_ram_to_mem <= _r4;
											ram_din_i <= _r3;
											$display("R2MRI R3 ",_r4);
										end
										R5 : begin
											addr_ram_to_mem <= _r5;
											ram_din_i <= _r3;
											$display("R2MRI R3 ",_r5);
										end
										R6 : begin
											addr_ram_to_mem <= _r6;
											ram_din_i <= _r3;
											$display("R2MRI R3 ",_r6);
										end
										R7 : begin
											addr_ram_to_mem <= _r7;
											ram_din_i <= _r3;
											$display("R2MRI R3 ",_r7);
										end
									endcase
								end
								R4 : begin
									case (current_instruction[7:5])
										R0 : begin
											addr_ram_to_mem <= _r0;
											ram_din_i <= _r4;
											$display("R2MRI R4 ",_r0);
										end
										R1 : begin
											addr_ram_to_mem <= _r1;
											ram_din_i <= _r4;
											$display("R2MRI R4 ",_r1);
										end
										R2 : begin
											addr_ram_to_mem <= _r2;
											ram_din_i <= _r4;
											$display("R2MRI R4 ",_r2);
										end
										R3 : begin
											addr_ram_to_mem <= _r3;
											ram_din_i <= _r4;
											$display("R2MRI R4 ",_r3);
										end
										R4 : begin
											addr_ram_to_mem <= _r4;
											ram_din_i <= _r4;
											$display("R2MRI R4 ",_r4);
										end
										R5 : begin
											addr_ram_to_mem <= _r5;
											ram_din_i <= _r4;
											$display("R2MRI R4 ",_r5);
										end
										R6 : begin
											addr_ram_to_mem <= _r6;
											ram_din_i <= _r4;
											$display("R2MRI R4 ",_r6);
										end
										R7 : begin
											addr_ram_to_mem <= _r7;
											ram_din_i <= _r4;
											$display("R2MRI R4 ",_r7);
										end
									endcase
								end
								R5 : begin
									case (current_instruction[7:5])
										R0 : begin
											addr_ram_to_mem <= _r0;
											ram_din_i <= _r5;
											$display("R2MRI R5 ",_r0);
										end
										R1 : begin
											addr_ram_to_mem <= _r1;
											ram_din_i <= _r5;
											$display("R2MRI R5 ",_r1);
										end
										R2 : begin
											addr_ram_to_mem <= _r2;
											ram_din_i <= _r5;
											$display("R2MRI R5 ",_r2);
										end
										R3 : begin
											addr_ram_to_mem <= _r3;
											ram_din_i <= _r5;
											$display("R2MRI R5 ",_r3);
										end
										R4 : begin
											addr_ram_to_mem <= _r4;
											ram_din_i <= _r5;
											$display("R2MRI R5 ",_r4);
										end
										R5 : begin
											addr_ram_to_mem <= _r5;
											ram_din_i <= _r5;
											$display("R2MRI R5 ",_r5);
										end
										R6 : begin
											addr_ram_to_mem <= _r6;
											ram_din_i <= _r5;
											$display("R2MRI R5 ",_r6);
										end
										R7 : begin
											addr_ram_to_mem <= _r7;
											ram_din_i <= _r5;
											$display("R2MRI R5 ",_r7);
										end
									endcase
								end
								R6 : begin
									case (current_instruction[7:5])
										R0 : begin
											addr_ram_to_mem <= _r0;
											ram_din_i <= _r6;
											$display("R2MRI R6 ",_r0);
										end
										R1 : begin
											addr_ram_to_mem <= _r1;
											ram_din_i <= _r6;
											$display("R2MRI R6 ",_r1);
										end
										R2 : begin
											addr_ram_to_mem <= _r2;
											ram_din_i <= _r6;
											$display("R2MRI R6 ",_r2);
										end
										R3 : begin
											addr_ram_to_mem <= _r3;
											ram_din_i <= _r6;
											$display("R2MRI R6 ",_r3);
										end
										R4 : begin
											addr_ram_to_mem <= _r4;
											ram_din_i <= _r6;
											$display("R2MRI R6 ",_r4);
										end
										R5 : begin
											addr_ram_to_mem <= _r5;
											ram_din_i <= _r6;
											$display("R2MRI R6 ",_r5);
										end
										R6 : begin
											addr_ram_to_mem <= _r6;
											ram_din_i <= _r6;
											$display("R2MRI R6 ",_r6);
										end
										R7 : begin
											addr_ram_to_mem <= _r7;
											ram_din_i <= _r6;
											$display("R2MRI R6 ",_r7);
										end
									endcase
								end
								R7 : begin
									case (current_instruction[7:5])
										R0 : begin
											addr_ram_to_mem <= _r0;
											ram_din_i <= _r7;
											$display("R2MRI R7 ",_r0);
										end
										R1 : begin
											addr_ram_to_mem <= _r1;
											ram_din_i <= _r7;
											$display("R2MRI R7 ",_r1);
										end
										R2 : begin
											addr_ram_to_mem <= _r2;
											ram_din_i <= _r7;
											$display("R2MRI R7 ",_r2);
										end
										R3 : begin
											addr_ram_to_mem <= _r3;
											ram_din_i <= _r7;
											$display("R2MRI R7 ",_r3);
										end
										R4 : begin
											addr_ram_to_mem <= _r4;
											ram_din_i <= _r7;
											$display("R2MRI R7 ",_r4);
										end
										R5 : begin
											addr_ram_to_mem <= _r5;
											ram_din_i <= _r7;
											$display("R2MRI R7 ",_r5);
										end
										R6 : begin
											addr_ram_to_mem <= _r6;
											ram_din_i <= _r7;
											$display("R2MRI R7 ",_r6);
										end
										R7 : begin
											addr_ram_to_mem <= _r7;
											ram_din_i <= _r7;
											$display("R2MRI R7 ",_r7);
										end
									endcase
								end
							endcase
						end
						else begin
							wr_int_ram <= #1 1'b0;
							if (exec_mode == 1'b1) begin
								vn_state <= FETCH;
							end
							_pc <= #1 _pc + 1'b1;
						end
					end

// Start of the component of the "state machine" for the opcode r2o

					R2O: begin
						case (current_instruction[10:8])
						R0 : begin
							case (current_instruction[7])
							O0 : begin
								_auxo0 <= #1 _r0;
								$display("R2O R0 O0");
							end
							endcase
						end
						R1 : begin
							case (current_instruction[7])
							O0 : begin
								_auxo0 <= #1 _r1;
								$display("R2O R1 O0");
							end
							endcase
						end
						R2 : begin
							case (current_instruction[7])
							O0 : begin
								_auxo0 <= #1 _r2;
								$display("R2O R2 O0");
							end
							endcase
						end
						R3 : begin
							case (current_instruction[7])
							O0 : begin
								_auxo0 <= #1 _r3;
								$display("R2O R3 O0");
							end
							endcase
						end
						R4 : begin
							case (current_instruction[7])
							O0 : begin
								_auxo0 <= #1 _r4;
								$display("R2O R4 O0");
							end
							endcase
						end
						R5 : begin
							case (current_instruction[7])
							O0 : begin
								_auxo0 <= #1 _r5;
								$display("R2O R5 O0");
							end
							endcase
						end
						R6 : begin
							case (current_instruction[7])
							O0 : begin
								_auxo0 <= #1 _r6;
								$display("R2O R6 O0");
							end
							endcase
						end
						R7 : begin
							case (current_instruction[7])
							O0 : begin
								_auxo0 <= #1 _r7;
								$display("R2O R7 O0");
							end
							endcase
						end
						endcase
						if (exec_mode == 1'b1) begin
							vn_state <= FETCH;
						end
						_pc <= #1 _pc + 1'b1;
					end

// Start of the component of the "state machine" for the opcode r2vri

					R2VRI: begin
						if (exec_mode == 1'b1) begin
							vn_state <= FETCH;
						end
						_pc <= #1 _pc + 1'b1;
					end

// Start of the component of the "state machine" for the opcode ret8s

					RET8S: begin
						if (restack0_8sreceiverAck && restack0_8sreceiverRead) begin
							restack0_8sreceiverRead <= #1 1'b0;
							_pc[6:0] <= #1 restack0_8sreceiverData[6:0];
							exec_mode <= #1 restack0_8sreceiverData[7];
							vn_state <= FETCH;
						end
						else begin
							restack0_8sreceiverRead <= #1 1'b1;
						end
					end

// Start of the component of the "state machine" for the opcode ro2rri

					RO2RRI: begin
						case (current_instruction[10:8])
						R0 : begin
							case (current_instruction[7:5])
							R0 : begin
								if (romread_ready == 1'b1) begin
									_r0[15:0] <= #1 romread_value[15:0];
									romread_ready <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									romread_bus[6:0] <= _r0[6:0];
									romread_ready <= 1'b1;
								end
								$display("RO2RRI R0 ",_r0);
							end
							R1 : begin
								if (romread_ready == 1'b1) begin
									_r0[15:0] <= #1 romread_value[15:0];
									romread_ready <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									romread_bus[6:0] <= _r1[6:0];
									romread_ready <= 1'b1;
								end
								$display("RO2RRI R0 ",_r0);
							end
							R2 : begin
								if (romread_ready == 1'b1) begin
									_r0[15:0] <= #1 romread_value[15:0];
									romread_ready <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									romread_bus[6:0] <= _r2[6:0];
									romread_ready <= 1'b1;
								end
								$display("RO2RRI R0 ",_r0);
							end
							R3 : begin
								if (romread_ready == 1'b1) begin
									_r0[15:0] <= #1 romread_value[15:0];
									romread_ready <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									romread_bus[6:0] <= _r3[6:0];
									romread_ready <= 1'b1;
								end
								$display("RO2RRI R0 ",_r0);
							end
							R4 : begin
								if (romread_ready == 1'b1) begin
									_r0[15:0] <= #1 romread_value[15:0];
									romread_ready <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									romread_bus[6:0] <= _r4[6:0];
									romread_ready <= 1'b1;
								end
								$display("RO2RRI R0 ",_r0);
							end
							R5 : begin
								if (romread_ready == 1'b1) begin
									_r0[15:0] <= #1 romread_value[15:0];
									romread_ready <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									romread_bus[6:0] <= _r5[6:0];
									romread_ready <= 1'b1;
								end
								$display("RO2RRI R0 ",_r0);
							end
							R6 : begin
								if (romread_ready == 1'b1) begin
									_r0[15:0] <= #1 romread_value[15:0];
									romread_ready <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									romread_bus[6:0] <= _r6[6:0];
									romread_ready <= 1'b1;
								end
								$display("RO2RRI R0 ",_r0);
							end
							R7 : begin
								if (romread_ready == 1'b1) begin
									_r0[15:0] <= #1 romread_value[15:0];
									romread_ready <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									romread_bus[6:0] <= _r7[6:0];
									romread_ready <= 1'b1;
								end
								$display("RO2RRI R0 ",_r0);
							end
							endcase
						end
						R1 : begin
							case (current_instruction[7:5])
							R0 : begin
								if (romread_ready == 1'b1) begin
									_r1[15:0] <= #1 romread_value[15:0];
									romread_ready <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									romread_bus[6:0] <= _r0[6:0];
									romread_ready <= 1'b1;
								end
								$display("RO2RRI R1 ",_r1);
							end
							R1 : begin
								if (romread_ready == 1'b1) begin
									_r1[15:0] <= #1 romread_value[15:0];
									romread_ready <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									romread_bus[6:0] <= _r1[6:0];
									romread_ready <= 1'b1;
								end
								$display("RO2RRI R1 ",_r1);
							end
							R2 : begin
								if (romread_ready == 1'b1) begin
									_r1[15:0] <= #1 romread_value[15:0];
									romread_ready <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									romread_bus[6:0] <= _r2[6:0];
									romread_ready <= 1'b1;
								end
								$display("RO2RRI R1 ",_r1);
							end
							R3 : begin
								if (romread_ready == 1'b1) begin
									_r1[15:0] <= #1 romread_value[15:0];
									romread_ready <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									romread_bus[6:0] <= _r3[6:0];
									romread_ready <= 1'b1;
								end
								$display("RO2RRI R1 ",_r1);
							end
							R4 : begin
								if (romread_ready == 1'b1) begin
									_r1[15:0] <= #1 romread_value[15:0];
									romread_ready <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									romread_bus[6:0] <= _r4[6:0];
									romread_ready <= 1'b1;
								end
								$display("RO2RRI R1 ",_r1);
							end
							R5 : begin
								if (romread_ready == 1'b1) begin
									_r1[15:0] <= #1 romread_value[15:0];
									romread_ready <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									romread_bus[6:0] <= _r5[6:0];
									romread_ready <= 1'b1;
								end
								$display("RO2RRI R1 ",_r1);
							end
							R6 : begin
								if (romread_ready == 1'b1) begin
									_r1[15:0] <= #1 romread_value[15:0];
									romread_ready <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									romread_bus[6:0] <= _r6[6:0];
									romread_ready <= 1'b1;
								end
								$display("RO2RRI R1 ",_r1);
							end
							R7 : begin
								if (romread_ready == 1'b1) begin
									_r1[15:0] <= #1 romread_value[15:0];
									romread_ready <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									romread_bus[6:0] <= _r7[6:0];
									romread_ready <= 1'b1;
								end
								$display("RO2RRI R1 ",_r1);
							end
							endcase
						end
						R2 : begin
							case (current_instruction[7:5])
							R0 : begin
								if (romread_ready == 1'b1) begin
									_r2[15:0] <= #1 romread_value[15:0];
									romread_ready <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									romread_bus[6:0] <= _r0[6:0];
									romread_ready <= 1'b1;
								end
								$display("RO2RRI R2 ",_r2);
							end
							R1 : begin
								if (romread_ready == 1'b1) begin
									_r2[15:0] <= #1 romread_value[15:0];
									romread_ready <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									romread_bus[6:0] <= _r1[6:0];
									romread_ready <= 1'b1;
								end
								$display("RO2RRI R2 ",_r2);
							end
							R2 : begin
								if (romread_ready == 1'b1) begin
									_r2[15:0] <= #1 romread_value[15:0];
									romread_ready <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									romread_bus[6:0] <= _r2[6:0];
									romread_ready <= 1'b1;
								end
								$display("RO2RRI R2 ",_r2);
							end
							R3 : begin
								if (romread_ready == 1'b1) begin
									_r2[15:0] <= #1 romread_value[15:0];
									romread_ready <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									romread_bus[6:0] <= _r3[6:0];
									romread_ready <= 1'b1;
								end
								$display("RO2RRI R2 ",_r2);
							end
							R4 : begin
								if (romread_ready == 1'b1) begin
									_r2[15:0] <= #1 romread_value[15:0];
									romread_ready <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									romread_bus[6:0] <= _r4[6:0];
									romread_ready <= 1'b1;
								end
								$display("RO2RRI R2 ",_r2);
							end
							R5 : begin
								if (romread_ready == 1'b1) begin
									_r2[15:0] <= #1 romread_value[15:0];
									romread_ready <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									romread_bus[6:0] <= _r5[6:0];
									romread_ready <= 1'b1;
								end
								$display("RO2RRI R2 ",_r2);
							end
							R6 : begin
								if (romread_ready == 1'b1) begin
									_r2[15:0] <= #1 romread_value[15:0];
									romread_ready <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									romread_bus[6:0] <= _r6[6:0];
									romread_ready <= 1'b1;
								end
								$display("RO2RRI R2 ",_r2);
							end
							R7 : begin
								if (romread_ready == 1'b1) begin
									_r2[15:0] <= #1 romread_value[15:0];
									romread_ready <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									romread_bus[6:0] <= _r7[6:0];
									romread_ready <= 1'b1;
								end
								$display("RO2RRI R2 ",_r2);
							end
							endcase
						end
						R3 : begin
							case (current_instruction[7:5])
							R0 : begin
								if (romread_ready == 1'b1) begin
									_r3[15:0] <= #1 romread_value[15:0];
									romread_ready <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									romread_bus[6:0] <= _r0[6:0];
									romread_ready <= 1'b1;
								end
								$display("RO2RRI R3 ",_r3);
							end
							R1 : begin
								if (romread_ready == 1'b1) begin
									_r3[15:0] <= #1 romread_value[15:0];
									romread_ready <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									romread_bus[6:0] <= _r1[6:0];
									romread_ready <= 1'b1;
								end
								$display("RO2RRI R3 ",_r3);
							end
							R2 : begin
								if (romread_ready == 1'b1) begin
									_r3[15:0] <= #1 romread_value[15:0];
									romread_ready <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									romread_bus[6:0] <= _r2[6:0];
									romread_ready <= 1'b1;
								end
								$display("RO2RRI R3 ",_r3);
							end
							R3 : begin
								if (romread_ready == 1'b1) begin
									_r3[15:0] <= #1 romread_value[15:0];
									romread_ready <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									romread_bus[6:0] <= _r3[6:0];
									romread_ready <= 1'b1;
								end
								$display("RO2RRI R3 ",_r3);
							end
							R4 : begin
								if (romread_ready == 1'b1) begin
									_r3[15:0] <= #1 romread_value[15:0];
									romread_ready <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									romread_bus[6:0] <= _r4[6:0];
									romread_ready <= 1'b1;
								end
								$display("RO2RRI R3 ",_r3);
							end
							R5 : begin
								if (romread_ready == 1'b1) begin
									_r3[15:0] <= #1 romread_value[15:0];
									romread_ready <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									romread_bus[6:0] <= _r5[6:0];
									romread_ready <= 1'b1;
								end
								$display("RO2RRI R3 ",_r3);
							end
							R6 : begin
								if (romread_ready == 1'b1) begin
									_r3[15:0] <= #1 romread_value[15:0];
									romread_ready <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									romread_bus[6:0] <= _r6[6:0];
									romread_ready <= 1'b1;
								end
								$display("RO2RRI R3 ",_r3);
							end
							R7 : begin
								if (romread_ready == 1'b1) begin
									_r3[15:0] <= #1 romread_value[15:0];
									romread_ready <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									romread_bus[6:0] <= _r7[6:0];
									romread_ready <= 1'b1;
								end
								$display("RO2RRI R3 ",_r3);
							end
							endcase
						end
						R4 : begin
							case (current_instruction[7:5])
							R0 : begin
								if (romread_ready == 1'b1) begin
									_r4[15:0] <= #1 romread_value[15:0];
									romread_ready <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									romread_bus[6:0] <= _r0[6:0];
									romread_ready <= 1'b1;
								end
								$display("RO2RRI R4 ",_r4);
							end
							R1 : begin
								if (romread_ready == 1'b1) begin
									_r4[15:0] <= #1 romread_value[15:0];
									romread_ready <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									romread_bus[6:0] <= _r1[6:0];
									romread_ready <= 1'b1;
								end
								$display("RO2RRI R4 ",_r4);
							end
							R2 : begin
								if (romread_ready == 1'b1) begin
									_r4[15:0] <= #1 romread_value[15:0];
									romread_ready <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									romread_bus[6:0] <= _r2[6:0];
									romread_ready <= 1'b1;
								end
								$display("RO2RRI R4 ",_r4);
							end
							R3 : begin
								if (romread_ready == 1'b1) begin
									_r4[15:0] <= #1 romread_value[15:0];
									romread_ready <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									romread_bus[6:0] <= _r3[6:0];
									romread_ready <= 1'b1;
								end
								$display("RO2RRI R4 ",_r4);
							end
							R4 : begin
								if (romread_ready == 1'b1) begin
									_r4[15:0] <= #1 romread_value[15:0];
									romread_ready <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									romread_bus[6:0] <= _r4[6:0];
									romread_ready <= 1'b1;
								end
								$display("RO2RRI R4 ",_r4);
							end
							R5 : begin
								if (romread_ready == 1'b1) begin
									_r4[15:0] <= #1 romread_value[15:0];
									romread_ready <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									romread_bus[6:0] <= _r5[6:0];
									romread_ready <= 1'b1;
								end
								$display("RO2RRI R4 ",_r4);
							end
							R6 : begin
								if (romread_ready == 1'b1) begin
									_r4[15:0] <= #1 romread_value[15:0];
									romread_ready <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									romread_bus[6:0] <= _r6[6:0];
									romread_ready <= 1'b1;
								end
								$display("RO2RRI R4 ",_r4);
							end
							R7 : begin
								if (romread_ready == 1'b1) begin
									_r4[15:0] <= #1 romread_value[15:0];
									romread_ready <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									romread_bus[6:0] <= _r7[6:0];
									romread_ready <= 1'b1;
								end
								$display("RO2RRI R4 ",_r4);
							end
							endcase
						end
						R5 : begin
							case (current_instruction[7:5])
							R0 : begin
								if (romread_ready == 1'b1) begin
									_r5[15:0] <= #1 romread_value[15:0];
									romread_ready <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									romread_bus[6:0] <= _r0[6:0];
									romread_ready <= 1'b1;
								end
								$display("RO2RRI R5 ",_r5);
							end
							R1 : begin
								if (romread_ready == 1'b1) begin
									_r5[15:0] <= #1 romread_value[15:0];
									romread_ready <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									romread_bus[6:0] <= _r1[6:0];
									romread_ready <= 1'b1;
								end
								$display("RO2RRI R5 ",_r5);
							end
							R2 : begin
								if (romread_ready == 1'b1) begin
									_r5[15:0] <= #1 romread_value[15:0];
									romread_ready <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									romread_bus[6:0] <= _r2[6:0];
									romread_ready <= 1'b1;
								end
								$display("RO2RRI R5 ",_r5);
							end
							R3 : begin
								if (romread_ready == 1'b1) begin
									_r5[15:0] <= #1 romread_value[15:0];
									romread_ready <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									romread_bus[6:0] <= _r3[6:0];
									romread_ready <= 1'b1;
								end
								$display("RO2RRI R5 ",_r5);
							end
							R4 : begin
								if (romread_ready == 1'b1) begin
									_r5[15:0] <= #1 romread_value[15:0];
									romread_ready <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									romread_bus[6:0] <= _r4[6:0];
									romread_ready <= 1'b1;
								end
								$display("RO2RRI R5 ",_r5);
							end
							R5 : begin
								if (romread_ready == 1'b1) begin
									_r5[15:0] <= #1 romread_value[15:0];
									romread_ready <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									romread_bus[6:0] <= _r5[6:0];
									romread_ready <= 1'b1;
								end
								$display("RO2RRI R5 ",_r5);
							end
							R6 : begin
								if (romread_ready == 1'b1) begin
									_r5[15:0] <= #1 romread_value[15:0];
									romread_ready <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									romread_bus[6:0] <= _r6[6:0];
									romread_ready <= 1'b1;
								end
								$display("RO2RRI R5 ",_r5);
							end
							R7 : begin
								if (romread_ready == 1'b1) begin
									_r5[15:0] <= #1 romread_value[15:0];
									romread_ready <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									romread_bus[6:0] <= _r7[6:0];
									romread_ready <= 1'b1;
								end
								$display("RO2RRI R5 ",_r5);
							end
							endcase
						end
						R6 : begin
							case (current_instruction[7:5])
							R0 : begin
								if (romread_ready == 1'b1) begin
									_r6[15:0] <= #1 romread_value[15:0];
									romread_ready <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									romread_bus[6:0] <= _r0[6:0];
									romread_ready <= 1'b1;
								end
								$display("RO2RRI R6 ",_r6);
							end
							R1 : begin
								if (romread_ready == 1'b1) begin
									_r6[15:0] <= #1 romread_value[15:0];
									romread_ready <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									romread_bus[6:0] <= _r1[6:0];
									romread_ready <= 1'b1;
								end
								$display("RO2RRI R6 ",_r6);
							end
							R2 : begin
								if (romread_ready == 1'b1) begin
									_r6[15:0] <= #1 romread_value[15:0];
									romread_ready <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									romread_bus[6:0] <= _r2[6:0];
									romread_ready <= 1'b1;
								end
								$display("RO2RRI R6 ",_r6);
							end
							R3 : begin
								if (romread_ready == 1'b1) begin
									_r6[15:0] <= #1 romread_value[15:0];
									romread_ready <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									romread_bus[6:0] <= _r3[6:0];
									romread_ready <= 1'b1;
								end
								$display("RO2RRI R6 ",_r6);
							end
							R4 : begin
								if (romread_ready == 1'b1) begin
									_r6[15:0] <= #1 romread_value[15:0];
									romread_ready <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									romread_bus[6:0] <= _r4[6:0];
									romread_ready <= 1'b1;
								end
								$display("RO2RRI R6 ",_r6);
							end
							R5 : begin
								if (romread_ready == 1'b1) begin
									_r6[15:0] <= #1 romread_value[15:0];
									romread_ready <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									romread_bus[6:0] <= _r5[6:0];
									romread_ready <= 1'b1;
								end
								$display("RO2RRI R6 ",_r6);
							end
							R6 : begin
								if (romread_ready == 1'b1) begin
									_r6[15:0] <= #1 romread_value[15:0];
									romread_ready <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									romread_bus[6:0] <= _r6[6:0];
									romread_ready <= 1'b1;
								end
								$display("RO2RRI R6 ",_r6);
							end
							R7 : begin
								if (romread_ready == 1'b1) begin
									_r6[15:0] <= #1 romread_value[15:0];
									romread_ready <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									romread_bus[6:0] <= _r7[6:0];
									romread_ready <= 1'b1;
								end
								$display("RO2RRI R6 ",_r6);
							end
							endcase
						end
						R7 : begin
							case (current_instruction[7:5])
							R0 : begin
								if (romread_ready == 1'b1) begin
									_r7[15:0] <= #1 romread_value[15:0];
									romread_ready <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									romread_bus[6:0] <= _r0[6:0];
									romread_ready <= 1'b1;
								end
								$display("RO2RRI R7 ",_r7);
							end
							R1 : begin
								if (romread_ready == 1'b1) begin
									_r7[15:0] <= #1 romread_value[15:0];
									romread_ready <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									romread_bus[6:0] <= _r1[6:0];
									romread_ready <= 1'b1;
								end
								$display("RO2RRI R7 ",_r7);
							end
							R2 : begin
								if (romread_ready == 1'b1) begin
									_r7[15:0] <= #1 romread_value[15:0];
									romread_ready <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									romread_bus[6:0] <= _r2[6:0];
									romread_ready <= 1'b1;
								end
								$display("RO2RRI R7 ",_r7);
							end
							R3 : begin
								if (romread_ready == 1'b1) begin
									_r7[15:0] <= #1 romread_value[15:0];
									romread_ready <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									romread_bus[6:0] <= _r3[6:0];
									romread_ready <= 1'b1;
								end
								$display("RO2RRI R7 ",_r7);
							end
							R4 : begin
								if (romread_ready == 1'b1) begin
									_r7[15:0] <= #1 romread_value[15:0];
									romread_ready <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									romread_bus[6:0] <= _r4[6:0];
									romread_ready <= 1'b1;
								end
								$display("RO2RRI R7 ",_r7);
							end
							R5 : begin
								if (romread_ready == 1'b1) begin
									_r7[15:0] <= #1 romread_value[15:0];
									romread_ready <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									romread_bus[6:0] <= _r5[6:0];
									romread_ready <= 1'b1;
								end
								$display("RO2RRI R7 ",_r7);
							end
							R6 : begin
								if (romread_ready == 1'b1) begin
									_r7[15:0] <= #1 romread_value[15:0];
									romread_ready <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									romread_bus[6:0] <= _r6[6:0];
									romread_ready <= 1'b1;
								end
								$display("RO2RRI R7 ",_r7);
							end
							R7 : begin
								if (romread_ready == 1'b1) begin
									_r7[15:0] <= #1 romread_value[15:0];
									romread_ready <= 1'b0;
									if (exec_mode == 1'b1) begin
										vn_state <= FETCH;
									end
									_pc <= #1 _pc + 1'b1;
								end
								else begin
									romread_bus[6:0] <= _r7[6:0];
									romread_ready <= 1'b1;
								end
								$display("RO2RRI R7 ",_r7);
							end
							endcase
						end
						endcase
					end

// Start of the component of the "state machine" for the opcode rsets8

					RSETS8: begin
						case (current_instruction[10:8])
						R0 : begin
							_r0 <= #1 current_instruction[7:0];
							$display("RSET R0 ",_r0);
						end
						R1 : begin
							_r1 <= #1 current_instruction[7:0];
							$display("RSET R1 ",_r1);
						end
						R2 : begin
							_r2 <= #1 current_instruction[7:0];
							$display("RSET R2 ",_r2);
						end
						R3 : begin
							_r3 <= #1 current_instruction[7:0];
							$display("RSET R3 ",_r3);
						end
						R4 : begin
							_r4 <= #1 current_instruction[7:0];
							$display("RSET R4 ",_r4);
						end
						R5 : begin
							_r5 <= #1 current_instruction[7:0];
							$display("RSET R5 ",_r5);
						end
						R6 : begin
							_r6 <= #1 current_instruction[7:0];
							$display("RSET R6 ",_r6);
						end
						R7 : begin
							_r7 <= #1 current_instruction[7:0];
							$display("RSET R7 ",_r7);
						end
						endcase
						if (exec_mode == 1'b1) begin
							vn_state <= FETCH;
						end
						_pc <= #1 _pc + 1'b1;
					end
					default : begin
						$display("Unknown Opcode");
						if (exec_mode == 1'b1) begin
							vn_state <= FETCH;
						end
						_pc <= #1 _pc + 1'b1;
					end
				endcase
			end
		end
	end
	assign rom_bus = _pc;
	assign ram_en = 1'b1;
	assign ram_addr =  (exec_mode == 1'b1 && vn_state == FETCH) ? _pc :  (current_instruction[15:11]==M2RRI) ? addr_ram_m2rri: addr_ram_to_mem;
	assign ram_din = ram_din_i;
	assign ram_wren = wr_int_ram;
	always @(posedge clock_signal)
	begin
		case (current_instruction[15:11])
		R2VRI: begin
			case (current_instruction[10:8])
				R0 : begin
						case (current_instruction[7:5])
						R0 : begin
							vtm0_wren_i <= 1'b1;
							vtm0_addr_i[7:0] <= _r0[7:0];
							vtm0_din_i[7:0] <= _r0[7:0];
							$display("R2VRI R0 ",_r0);
						end
						R1 : begin
							vtm0_wren_i <= 1'b1;
							vtm0_addr_i[7:0] <= _r1[7:0];
							vtm0_din_i[7:0] <= _r0[7:0];
							$display("R2VRI R0 ",_r0);
						end
						R2 : begin
							vtm0_wren_i <= 1'b1;
							vtm0_addr_i[7:0] <= _r2[7:0];
							vtm0_din_i[7:0] <= _r0[7:0];
							$display("R2VRI R0 ",_r0);
						end
						R3 : begin
							vtm0_wren_i <= 1'b1;
							vtm0_addr_i[7:0] <= _r3[7:0];
							vtm0_din_i[7:0] <= _r0[7:0];
							$display("R2VRI R0 ",_r0);
						end
						R4 : begin
							vtm0_wren_i <= 1'b1;
							vtm0_addr_i[7:0] <= _r4[7:0];
							vtm0_din_i[7:0] <= _r0[7:0];
							$display("R2VRI R0 ",_r0);
						end
						R5 : begin
							vtm0_wren_i <= 1'b1;
							vtm0_addr_i[7:0] <= _r5[7:0];
							vtm0_din_i[7:0] <= _r0[7:0];
							$display("R2VRI R0 ",_r0);
						end
						R6 : begin
							vtm0_wren_i <= 1'b1;
							vtm0_addr_i[7:0] <= _r6[7:0];
							vtm0_din_i[7:0] <= _r0[7:0];
							$display("R2VRI R0 ",_r0);
						end
						R7 : begin
							vtm0_wren_i <= 1'b1;
							vtm0_addr_i[7:0] <= _r7[7:0];
							vtm0_din_i[7:0] <= _r0[7:0];
							$display("R2VRI R0 ",_r0);
						end
							endcase
				end
				R1 : begin
						case (current_instruction[7:5])
						R0 : begin
							vtm0_wren_i <= 1'b1;
							vtm0_addr_i[7:0] <= _r0[7:0];
							vtm0_din_i[7:0] <= _r1[7:0];
							$display("R2VRI R1 ",_r1);
						end
						R1 : begin
							vtm0_wren_i <= 1'b1;
							vtm0_addr_i[7:0] <= _r1[7:0];
							vtm0_din_i[7:0] <= _r1[7:0];
							$display("R2VRI R1 ",_r1);
						end
						R2 : begin
							vtm0_wren_i <= 1'b1;
							vtm0_addr_i[7:0] <= _r2[7:0];
							vtm0_din_i[7:0] <= _r1[7:0];
							$display("R2VRI R1 ",_r1);
						end
						R3 : begin
							vtm0_wren_i <= 1'b1;
							vtm0_addr_i[7:0] <= _r3[7:0];
							vtm0_din_i[7:0] <= _r1[7:0];
							$display("R2VRI R1 ",_r1);
						end
						R4 : begin
							vtm0_wren_i <= 1'b1;
							vtm0_addr_i[7:0] <= _r4[7:0];
							vtm0_din_i[7:0] <= _r1[7:0];
							$display("R2VRI R1 ",_r1);
						end
						R5 : begin
							vtm0_wren_i <= 1'b1;
							vtm0_addr_i[7:0] <= _r5[7:0];
							vtm0_din_i[7:0] <= _r1[7:0];
							$display("R2VRI R1 ",_r1);
						end
						R6 : begin
							vtm0_wren_i <= 1'b1;
							vtm0_addr_i[7:0] <= _r6[7:0];
							vtm0_din_i[7:0] <= _r1[7:0];
							$display("R2VRI R1 ",_r1);
						end
						R7 : begin
							vtm0_wren_i <= 1'b1;
							vtm0_addr_i[7:0] <= _r7[7:0];
							vtm0_din_i[7:0] <= _r1[7:0];
							$display("R2VRI R1 ",_r1);
						end
							endcase
				end
				R2 : begin
						case (current_instruction[7:5])
						R0 : begin
							vtm0_wren_i <= 1'b1;
							vtm0_addr_i[7:0] <= _r0[7:0];
							vtm0_din_i[7:0] <= _r2[7:0];
							$display("R2VRI R2 ",_r2);
						end
						R1 : begin
							vtm0_wren_i <= 1'b1;
							vtm0_addr_i[7:0] <= _r1[7:0];
							vtm0_din_i[7:0] <= _r2[7:0];
							$display("R2VRI R2 ",_r2);
						end
						R2 : begin
							vtm0_wren_i <= 1'b1;
							vtm0_addr_i[7:0] <= _r2[7:0];
							vtm0_din_i[7:0] <= _r2[7:0];
							$display("R2VRI R2 ",_r2);
						end
						R3 : begin
							vtm0_wren_i <= 1'b1;
							vtm0_addr_i[7:0] <= _r3[7:0];
							vtm0_din_i[7:0] <= _r2[7:0];
							$display("R2VRI R2 ",_r2);
						end
						R4 : begin
							vtm0_wren_i <= 1'b1;
							vtm0_addr_i[7:0] <= _r4[7:0];
							vtm0_din_i[7:0] <= _r2[7:0];
							$display("R2VRI R2 ",_r2);
						end
						R5 : begin
							vtm0_wren_i <= 1'b1;
							vtm0_addr_i[7:0] <= _r5[7:0];
							vtm0_din_i[7:0] <= _r2[7:0];
							$display("R2VRI R2 ",_r2);
						end
						R6 : begin
							vtm0_wren_i <= 1'b1;
							vtm0_addr_i[7:0] <= _r6[7:0];
							vtm0_din_i[7:0] <= _r2[7:0];
							$display("R2VRI R2 ",_r2);
						end
						R7 : begin
							vtm0_wren_i <= 1'b1;
							vtm0_addr_i[7:0] <= _r7[7:0];
							vtm0_din_i[7:0] <= _r2[7:0];
							$display("R2VRI R2 ",_r2);
						end
							endcase
				end
				R3 : begin
						case (current_instruction[7:5])
						R0 : begin
							vtm0_wren_i <= 1'b1;
							vtm0_addr_i[7:0] <= _r0[7:0];
							vtm0_din_i[7:0] <= _r3[7:0];
							$display("R2VRI R3 ",_r3);
						end
						R1 : begin
							vtm0_wren_i <= 1'b1;
							vtm0_addr_i[7:0] <= _r1[7:0];
							vtm0_din_i[7:0] <= _r3[7:0];
							$display("R2VRI R3 ",_r3);
						end
						R2 : begin
							vtm0_wren_i <= 1'b1;
							vtm0_addr_i[7:0] <= _r2[7:0];
							vtm0_din_i[7:0] <= _r3[7:0];
							$display("R2VRI R3 ",_r3);
						end
						R3 : begin
							vtm0_wren_i <= 1'b1;
							vtm0_addr_i[7:0] <= _r3[7:0];
							vtm0_din_i[7:0] <= _r3[7:0];
							$display("R2VRI R3 ",_r3);
						end
						R4 : begin
							vtm0_wren_i <= 1'b1;
							vtm0_addr_i[7:0] <= _r4[7:0];
							vtm0_din_i[7:0] <= _r3[7:0];
							$display("R2VRI R3 ",_r3);
						end
						R5 : begin
							vtm0_wren_i <= 1'b1;
							vtm0_addr_i[7:0] <= _r5[7:0];
							vtm0_din_i[7:0] <= _r3[7:0];
							$display("R2VRI R3 ",_r3);
						end
						R6 : begin
							vtm0_wren_i <= 1'b1;
							vtm0_addr_i[7:0] <= _r6[7:0];
							vtm0_din_i[7:0] <= _r3[7:0];
							$display("R2VRI R3 ",_r3);
						end
						R7 : begin
							vtm0_wren_i <= 1'b1;
							vtm0_addr_i[7:0] <= _r7[7:0];
							vtm0_din_i[7:0] <= _r3[7:0];
							$display("R2VRI R3 ",_r3);
						end
							endcase
				end
				R4 : begin
						case (current_instruction[7:5])
						R0 : begin
							vtm0_wren_i <= 1'b1;
							vtm0_addr_i[7:0] <= _r0[7:0];
							vtm0_din_i[7:0] <= _r4[7:0];
							$display("R2VRI R4 ",_r4);
						end
						R1 : begin
							vtm0_wren_i <= 1'b1;
							vtm0_addr_i[7:0] <= _r1[7:0];
							vtm0_din_i[7:0] <= _r4[7:0];
							$display("R2VRI R4 ",_r4);
						end
						R2 : begin
							vtm0_wren_i <= 1'b1;
							vtm0_addr_i[7:0] <= _r2[7:0];
							vtm0_din_i[7:0] <= _r4[7:0];
							$display("R2VRI R4 ",_r4);
						end
						R3 : begin
							vtm0_wren_i <= 1'b1;
							vtm0_addr_i[7:0] <= _r3[7:0];
							vtm0_din_i[7:0] <= _r4[7:0];
							$display("R2VRI R4 ",_r4);
						end
						R4 : begin
							vtm0_wren_i <= 1'b1;
							vtm0_addr_i[7:0] <= _r4[7:0];
							vtm0_din_i[7:0] <= _r4[7:0];
							$display("R2VRI R4 ",_r4);
						end
						R5 : begin
							vtm0_wren_i <= 1'b1;
							vtm0_addr_i[7:0] <= _r5[7:0];
							vtm0_din_i[7:0] <= _r4[7:0];
							$display("R2VRI R4 ",_r4);
						end
						R6 : begin
							vtm0_wren_i <= 1'b1;
							vtm0_addr_i[7:0] <= _r6[7:0];
							vtm0_din_i[7:0] <= _r4[7:0];
							$display("R2VRI R4 ",_r4);
						end
						R7 : begin
							vtm0_wren_i <= 1'b1;
							vtm0_addr_i[7:0] <= _r7[7:0];
							vtm0_din_i[7:0] <= _r4[7:0];
							$display("R2VRI R4 ",_r4);
						end
							endcase
				end
				R5 : begin
						case (current_instruction[7:5])
						R0 : begin
							vtm0_wren_i <= 1'b1;
							vtm0_addr_i[7:0] <= _r0[7:0];
							vtm0_din_i[7:0] <= _r5[7:0];
							$display("R2VRI R5 ",_r5);
						end
						R1 : begin
							vtm0_wren_i <= 1'b1;
							vtm0_addr_i[7:0] <= _r1[7:0];
							vtm0_din_i[7:0] <= _r5[7:0];
							$display("R2VRI R5 ",_r5);
						end
						R2 : begin
							vtm0_wren_i <= 1'b1;
							vtm0_addr_i[7:0] <= _r2[7:0];
							vtm0_din_i[7:0] <= _r5[7:0];
							$display("R2VRI R5 ",_r5);
						end
						R3 : begin
							vtm0_wren_i <= 1'b1;
							vtm0_addr_i[7:0] <= _r3[7:0];
							vtm0_din_i[7:0] <= _r5[7:0];
							$display("R2VRI R5 ",_r5);
						end
						R4 : begin
							vtm0_wren_i <= 1'b1;
							vtm0_addr_i[7:0] <= _r4[7:0];
							vtm0_din_i[7:0] <= _r5[7:0];
							$display("R2VRI R5 ",_r5);
						end
						R5 : begin
							vtm0_wren_i <= 1'b1;
							vtm0_addr_i[7:0] <= _r5[7:0];
							vtm0_din_i[7:0] <= _r5[7:0];
							$display("R2VRI R5 ",_r5);
						end
						R6 : begin
							vtm0_wren_i <= 1'b1;
							vtm0_addr_i[7:0] <= _r6[7:0];
							vtm0_din_i[7:0] <= _r5[7:0];
							$display("R2VRI R5 ",_r5);
						end
						R7 : begin
							vtm0_wren_i <= 1'b1;
							vtm0_addr_i[7:0] <= _r7[7:0];
							vtm0_din_i[7:0] <= _r5[7:0];
							$display("R2VRI R5 ",_r5);
						end
							endcase
				end
				R6 : begin
						case (current_instruction[7:5])
						R0 : begin
							vtm0_wren_i <= 1'b1;
							vtm0_addr_i[7:0] <= _r0[7:0];
							vtm0_din_i[7:0] <= _r6[7:0];
							$display("R2VRI R6 ",_r6);
						end
						R1 : begin
							vtm0_wren_i <= 1'b1;
							vtm0_addr_i[7:0] <= _r1[7:0];
							vtm0_din_i[7:0] <= _r6[7:0];
							$display("R2VRI R6 ",_r6);
						end
						R2 : begin
							vtm0_wren_i <= 1'b1;
							vtm0_addr_i[7:0] <= _r2[7:0];
							vtm0_din_i[7:0] <= _r6[7:0];
							$display("R2VRI R6 ",_r6);
						end
						R3 : begin
							vtm0_wren_i <= 1'b1;
							vtm0_addr_i[7:0] <= _r3[7:0];
							vtm0_din_i[7:0] <= _r6[7:0];
							$display("R2VRI R6 ",_r6);
						end
						R4 : begin
							vtm0_wren_i <= 1'b1;
							vtm0_addr_i[7:0] <= _r4[7:0];
							vtm0_din_i[7:0] <= _r6[7:0];
							$display("R2VRI R6 ",_r6);
						end
						R5 : begin
							vtm0_wren_i <= 1'b1;
							vtm0_addr_i[7:0] <= _r5[7:0];
							vtm0_din_i[7:0] <= _r6[7:0];
							$display("R2VRI R6 ",_r6);
						end
						R6 : begin
							vtm0_wren_i <= 1'b1;
							vtm0_addr_i[7:0] <= _r6[7:0];
							vtm0_din_i[7:0] <= _r6[7:0];
							$display("R2VRI R6 ",_r6);
						end
						R7 : begin
							vtm0_wren_i <= 1'b1;
							vtm0_addr_i[7:0] <= _r7[7:0];
							vtm0_din_i[7:0] <= _r6[7:0];
							$display("R2VRI R6 ",_r6);
						end
							endcase
				end
				R7 : begin
						case (current_instruction[7:5])
						R0 : begin
							vtm0_wren_i <= 1'b1;
							vtm0_addr_i[7:0] <= _r0[7:0];
							vtm0_din_i[7:0] <= _r7[7:0];
							$display("R2VRI R7 ",_r7);
						end
						R1 : begin
							vtm0_wren_i <= 1'b1;
							vtm0_addr_i[7:0] <= _r1[7:0];
							vtm0_din_i[7:0] <= _r7[7:0];
							$display("R2VRI R7 ",_r7);
						end
						R2 : begin
							vtm0_wren_i <= 1'b1;
							vtm0_addr_i[7:0] <= _r2[7:0];
							vtm0_din_i[7:0] <= _r7[7:0];
							$display("R2VRI R7 ",_r7);
						end
						R3 : begin
							vtm0_wren_i <= 1'b1;
							vtm0_addr_i[7:0] <= _r3[7:0];
							vtm0_din_i[7:0] <= _r7[7:0];
							$display("R2VRI R7 ",_r7);
						end
						R4 : begin
							vtm0_wren_i <= 1'b1;
							vtm0_addr_i[7:0] <= _r4[7:0];
							vtm0_din_i[7:0] <= _r7[7:0];
							$display("R2VRI R7 ",_r7);
						end
						R5 : begin
							vtm0_wren_i <= 1'b1;
							vtm0_addr_i[7:0] <= _r5[7:0];
							vtm0_din_i[7:0] <= _r7[7:0];
							$display("R2VRI R7 ",_r7);
						end
						R6 : begin
							vtm0_wren_i <= 1'b1;
							vtm0_addr_i[7:0] <= _r6[7:0];
							vtm0_din_i[7:0] <= _r7[7:0];
							$display("R2VRI R7 ",_r7);
						end
						R7 : begin
							vtm0_wren_i <= 1'b1;
							vtm0_addr_i[7:0] <= _r7[7:0];
							vtm0_din_i[7:0] <= _r7[7:0];
							$display("R2VRI R7 ",_r7);
						end
							endcase
				end
			endcase
		end
		default:
			vtm0_wren_i <= 1'b0;
		endcase
	end
	assign i0_received = i0_recv;
	assign o0 = _auxo0;
	assign o0_valid = o0_val;
endmodule

module restack0_8s(clk,
    reset,
    senderData,
    senderWrite,
    senderAck,
    receiverData,
    receiverRead,
    receiverAck,
    empty,
    full
);
    input clk;
    input reset;
    output empty;
    output full;
    input [7:0] senderData;
    input senderWrite;
    output reg senderAck;
    output reg [7:0] receiverData;
    input receiverRead;
    output reg receiverAck;

    reg [7:0] memory[7:0];
    reg [3:0] sp;

    assign empty = (sp==0)? 1'b1:1'b0; 
    assign full = (sp==8)? 1'b1:1'b0;
    
    wire readneed;
    wire writeneed;

    assign writeneed = ( 1'b0
            | senderWrite );

    assign readneed = ( 1'b0
            | receiverRead );

    reg [0:0] sendSM;
    //
    //localparam sendSMsender = 1'd0;
    //
    
    reg [0:0] recvSM;
    //
    //localparam recvSMreceiver = 1'd0;
    //

    integer i;

    always @(posedge clk) begin
        if (reset) begin
            sp <= 4'd0;
            receiverData <= 8'd0;
            receiverAck <= 1'b0;
            senderAck <= 1'b0;
            sendSM <= 1'd0;
            recvSM <= 1'd0;
            for (i=0;i<8;i=i+1) begin
                memory[i]<=8'd0;
            end
        end
        else begin
            // Read state machine part
            if (readneed && !empty) begin
                case (recvSM)
                1'd0: begin
                    if (receiverRead && !receiverAck) begin
                        receiverData[7:0] <= memory[sp-1];
                        sp <= sp - 1;
                    end
                    recvSM <= 1'd0;
                end
                endcase
            end
            // Write state machine part
            else if (writeneed && !full) begin
                case (sendSM)
                1'd0: begin
                    if (senderWrite && !senderAck) begin
                        memory[sp] <= senderData[7:0];
                        sp <= sp + 1;
                    end
                    sendSM <= 1'd0;
                end
                endcase
            end

            // Read ack process
            if (receiverRead && !receiverAck && recvSM==1'd0 && !empty) begin
                receiverAck <= 1'b1;
            end
            else begin
                if (!receiverRead) begin
                    receiverAck <= 1'b0;
                end
            end

            // Write ack process
            if (!(readneed && !empty) && senderWrite && !senderAck && sendSM==1'd0 && !full) begin
                senderAck <= 1'b1;
            end
            else begin
                if (!senderWrite) begin
                    senderAck <= 1'b0;
                end
            end
        end
    end
endmodule

module cptextvideoram #(parameter ADDR_WIDTH=8, DATA_WIDTH=8, DEPTH=256) (
    input wire clk,
    input wire [ADDR_WIDTH-1:0] addr, 
    input wire wen,
    input wire [DATA_WIDTH-1:0] i_data,
    output reg [DATA_WIDTH-1:0] o_data 
    );

    reg [DATA_WIDTH-1:0] videoram [0:DEPTH-1];

    integer i;    
    initial begin
        for (i=0 ; i<DEPTH ; i=i+1) begin
            videoram[i] = 0;  
        end
    end

    always @ (posedge clk)
    begin
        if(wen) begin
            videoram[addr] <= i_data;
        end
        else begin
            o_data <= videoram[addr];
        end     
    end
endmodule
`timescale 1ns/1ps
module vtm0(clk, rst, p0din, p0addrfromcp, p0wren, p0en, p0dout, p0addrfromext);

	input clk;
	input rst;

	input [7:0] p0din;
	input [7:0] p0addrfromcp;
	input p0wren;
	input p0en;
	output [7:0] p0dout;
	input [7:0] p0addrfromext;


	localparam P0_VIDEORAM_LEFT = 3;
	localparam P0_VIDEORAM_TOP = 3;
	localparam P0_VIDEORAM_ROWS = 16;
	localparam P0_VIDEORAM_COLS = 16;
	localparam P0_VIDEORAM_DEPTH = 256; 
	localparam P0_VIDEORAM_A_WIDTH = 8;
	localparam P0_VIDEORAM_D_WIDTH = 8;

	reg [P0_VIDEORAM_A_WIDTH-1:0] p0address;
	wire [P0_VIDEORAM_D_WIDTH-1:0] p0dataout;
	reg [7:0] p0data;
	reg p0wantwrite;

	cptextvideoram #(
		.ADDR_WIDTH(P0_VIDEORAM_A_WIDTH), 
		.DATA_WIDTH(P0_VIDEORAM_D_WIDTH), 
		.DEPTH(P0_VIDEORAM_DEPTH))
		p0videoram (
		.addr(p0address[P0_VIDEORAM_A_WIDTH-1:0]), 
		.o_data(p0dataout[P0_VIDEORAM_D_WIDTH-1:0]),
		.clk(clk),
		.wen(p0wantwrite),
		.i_data(p0data[P0_VIDEORAM_D_WIDTH-1:0])
	);

	always @ (posedge clk)
	begin

		if (p0wren) begin
			p0address[P0_VIDEORAM_A_WIDTH-1:0] <= p0addrfromcp[P0_VIDEORAM_A_WIDTH-1:0];
			p0wantwrite <= 1;
			p0data[P0_VIDEORAM_D_WIDTH-1:0] <= p0din[P0_VIDEORAM_D_WIDTH-1:0];
		end
		else
		begin
			p0address[P0_VIDEORAM_A_WIDTH-1:0] <= p0addrfromext[P0_VIDEORAM_A_WIDTH-1:0];
			p0wantwrite <= 0;
		end
	end

	assign p0dout[P0_VIDEORAM_D_WIDTH-1:0] = p0dataout[P0_VIDEORAM_D_WIDTH-1:0];


endmodule 
