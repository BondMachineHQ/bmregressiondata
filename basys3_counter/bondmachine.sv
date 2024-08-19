
`timescale 1ns/1ps
module a0(clock_signal, reset_signal, o0, o0_valid, o0_received);

	input clock_signal;
	input reset_signal;

	output [15:0] o0;
	output o0_valid;
	input o0_received;

	wire [1:0] rom_bus;
	wire [3:0] rom_value;

	wire [15:0] a0din;
	wire [15:0] a0dout;
	wire [7:0] a0addr;
	wire a0wren;
	wire a0en;

	p0 p0_instance(clock_signal, reset_signal, rom_bus, rom_value, a0din, a0dout, a0addr, a0wren, a0en, o0, o0_valid, o0_received);
	p0rom p0rom_instance(rom_bus, rom_value);
	p0ram p0ram_instance(clock_signal, reset_signal, a0din, a0dout, a0addr, a0wren, a0en);

endmodule
module bondmachine_main(

	input clk,
	input btnC,
	output reg [15:0] led
);

	assign reset = btnC;
	wire [15:0] Output0;

	reg [31:0] divider;

	bondmachine bondmachine_inst (divider[23], reset, Output0, Output0_valid, Output0_received);


	always @ (posedge clk) begin
		divider <= divider + 1;
	end
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
endmodule
module bondmachine(clk, reset, o0, o0_valid, o0_received);

	input clk, reset;
	//--------------Output Ports-----------------------
	output [15:0] o0;
	output o0_valid;
	input o0_received;



	wire [15:0] p0o0;
	wire p0o0_valid;
	wire p0o0_received;
	wire o0_received;


	//Instantiation of the Processors and Shared Objects
	a0 a0_inst(clk, reset, p0o0, p0o0_valid, p0o0_received);

	assign o0 = p0o0;
	assign o0_valid = p0o0_valid;

	assign p0o0_received = o0_received;

endmodule
`timescale 1ns/1ps
module p0ram(clk, rst, din, dout, addr, wren, en);

	//--------------Input Ports-----------------------
	input clk;
	input rst;
	input [7:0] addr;
	input [15:0] din;
	input wren;
	input en;

	//--------------Inout Ports-----------------------
	output [15:0] dout;

	//--------------Reg-------------------------------
	reg [15:0] mem [0:255];

	reg [15:0] dout_i;

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
module p0rom(input [1:0] rom_bus, output [3:0] rom_value);
	reg [3:0] _rom [0:3];
	initial
	begin
	_rom[0] = 4'b0000;
	_rom[1] = 4'b0100;
	_rom[2] = 4'b1100;
	_rom[3] = 4'b1001;
	end
	assign rom_value = _rom[rom_bus];
endmodule
`timescale 1ns/1ps
module p0(clock_signal, reset_signal, rom_bus, rom_value, ram_din, ram_dout, ram_addr, ram_wren, ram_en, o0, o0_valid, o0_received);

	input clock_signal;
	input reset_signal;
	output  [1:0] rom_bus;
	input  [3:0] rom_value;
	input  [15:0] ram_dout;
	output [15:0] ram_din;
	output  [7:0] ram_addr;
	output ram_wren, ram_en;

	output [15:0] o0;
	output o0_valid;
	input o0_received;

			// Opcodes in the istructions, lenght accourding the number of the selected.
	localparam	CLR=2'b00,          // Clear register
			INC=2'b01,          // Increment a register by 1
			J=2'b10,          // Jump to a program location
			R2O=2'b11;          // Register to output

	localparam	R0=1'b0,		// Registers in the intructions
			R1=1'b1;
	localparam			O0=1'b0;
	reg [15:0] _auxo0;

	reg [15:0] _ram [0:255];		// Internal processor RAM

	(* KEEP = "TRUE" *) reg [1:0] _pc;		// Program counter

	// The number of registers are 2^R, two letters and an underscore as identifier , maximum R=8 and 265 rigisters
	(* KEEP = "TRUE" *) reg [15:0] _r0;
	(* KEEP = "TRUE" *) reg [15:0] _r1;

	wire [3:0] current_instruction;
	assign current_instruction=rom_value;


	reg o0_val;

	always @(posedge clock_signal, posedge reset_signal)
	begin
		if (reset_signal)
		begin
			o0_val <= #1 1'b0;
		end
		else
		begin
			case(current_instruction[3:2])
				R2O: begin
					case (current_instruction[0])
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

	always @(posedge clock_signal, posedge reset_signal)
	begin
		if(reset_signal)
		begin
			_pc <= #1 2'h0;
			_r0 <= #1 16'h0;
			_r1 <= #1 16'h0;
		end
		else begin
			// ha placeholder
			$display("Program Counter:%d", _pc);
			$display("Instruction:%b", rom_value);
			$display("Registers r0:%b r1:%b ", _r0, _r1);
				case(current_instruction[3:2])
					CLR: begin
						case (current_instruction[1])
						R0 : begin
							_r0 <= #1 'b0;
							$display("CLR R0");
						end
						R1 : begin
							_r1 <= #1 'b0;
							$display("CLR R1");
						end
						endcase
						_pc <= #1 _pc + 1'b1;
					end
					INC: begin
						case (current_instruction[1])
						R0 : begin
							_r0 <= #1 _r0 + 1'b1;
							$display("INC R0");
						end
						R1 : begin
							_r1 <= #1 _r1 + 1'b1;
							$display("INC R1");
						end
						endcase
						_pc <= #1 _pc + 1'b1;
					end
					J: begin
						_pc <= #1 current_instruction[1:0];
						$display("J ", current_instruction[1:0]);
					end
					R2O: begin
						case (current_instruction[1])
						R0 : begin
							case (current_instruction[0])
							O0 : begin
								_auxo0 <= #1 _r0;
								$display("R2O R0 O0");
							end
							endcase
						end
						R1 : begin
							case (current_instruction[0])
							O0 : begin
								_auxo0 <= #1 _r1;
								$display("R2O R1 O0");
							end
							endcase
						end
						endcase
						_pc <= #1 _pc + 1'b1;
					end
					default : begin
						$display("Unknown Opcode");
						_pc <= #1 _pc + 1'b1;
					end
				endcase
			// ha placeholder
		end
	end
	assign rom_bus = _pc;
	assign o0 = _auxo0;
	assign o0_valid = o0_val;
endmodule
