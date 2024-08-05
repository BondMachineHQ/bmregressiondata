
`timescale 1ns/1ps
module a0(clock_signal, reset_signal, o0, o0_valid, o0_received);

	input clock_signal;
	input reset_signal;

	output [7:0] o0;
	output o0_valid;
	input o0_received;

	wire [2:0] rom_bus;
	wire [11:0] rom_value;


	p0 p0_instance(clock_signal, reset_signal, rom_bus, rom_value, o0, o0_valid, o0_received);
	p0rom p0rom_instance(rom_bus, rom_value);

endmodule
module bondmachine_main(

	input clk,
	input btnC,
	output reg [7:0] led
);

	assign reset = btnC;
	wire [7:0] Output0;

	bondmachine bondmachine_inst (clk, reset, Output0, Output0_valid, Output0_received);


	always @ (posedge clk) begin
		led[0] <= Output0[0];
		led[1] <= Output0[1];
		led[2] <= Output0[2];
		led[3] <= Output0[3];
		led[4] <= Output0[4];
		led[5] <= Output0[5];
		led[6] <= Output0[6];
		led[7] <= Output0[7];
	end
endmodule
module bondmachine(clk, reset, o0, o0_valid, o0_received);

	input clk, reset;
	//--------------Output Ports-----------------------
	output [7:0] o0;
	output o0_valid;
	input o0_received;



	wire [7:0] p0o0;
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
module p0rom(input [2:0] rom_bus, output [11:0] rom_value);
	reg [11:0] _rom [0:7];
	initial
	begin
	_rom[0] = 12'b000000000000;
	_rom[1] = 12'b101100000000;
	_rom[2] = 12'b001010000000;
	_rom[3] = 12'b001100000000;
	_rom[4] = 12'b100100000000;
	_rom[5] = 12'b010000000000;
	_rom[6] = 12'b011011000000;
	end
	assign rom_value = _rom[rom_bus];
endmodule
`timescale 1ns/1ps
module p0(clock_signal, reset_signal, rom_bus, rom_value, o0, o0_valid, o0_received);

	input clock_signal;
	input reset_signal;
	output  [2:0] rom_bus;
	input  [11:0] rom_value;

	output [7:0] o0;
	output o0_valid;
	input o0_received;

			// Opcodes in the istructions, lenght accourding the number of the selected.
	localparam	CLR=3'b000,          // Clear register
			CPY=3'b001,          // Copy from a register to another
			INC=3'b010,          // Increment a register by 1
			J=3'b011,          // Jump to a program location
			R2O=3'b100,          // Register to output
			RSET=3'b101;          // Register set value

	localparam	R0=1'b0,		// Registers in the intructions
			R1=1'b1;
	localparam			O0=1'b0;
	reg [7:0] _auxo0;

	reg [7:0] _ram [0:0];		// Internal processor RAM

	(* KEEP = "TRUE" *) reg [2:0] _pc;		// Program counter

	// The number of registers are 2^R, two letters and an unserscore as identifier , maximum R=8 and 265 rigisters
	(* KEEP = "TRUE" *) reg [7:0] _r0;
	(* KEEP = "TRUE" *) reg [7:0] _r1;

	wire [11:0] current_instruction;
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
			case(current_instruction[11:9])
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

	always @(posedge clock_signal, posedge reset_signal)
	begin
		if(reset_signal)
		begin
			_pc <= #1 3'h0;
			_r0 <= #1 8'h0;
			_r1 <= #1 8'h0;
		end
		else begin
			// ha placeholder
			$display("Program Counter:%d", _pc);
			$display("Instruction:%b", rom_value);
			$display("Registers r0:%b r1:%b ", _r0, _r1);
				case(current_instruction[11:9])
					CLR: begin
						case (current_instruction[8])
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
					CPY: begin
						case (current_instruction[8])
						R0 : begin
							case (current_instruction[7])
							R0 : begin
								_r0 <= #1 _r0;
								$display("CPY R0 R0");
							end
							R1 : begin
								_r0 <= #1 _r1;
								$display("CPY R0 R1");
							end
							endcase
						end
						R1 : begin
							case (current_instruction[7])
							R0 : begin
								_r1 <= #1 _r0;
								$display("CPY R1 R0");
							end
							R1 : begin
								_r1 <= #1 _r1;
								$display("CPY R1 R1");
							end
							endcase
						end
						endcase
						_pc <= #1 _pc + 1'b1;
					end
					INC: begin
						case (current_instruction[8])
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
						_pc <= #1 current_instruction[8:6];
						$display("J ", current_instruction[8:6]);
					end
					R2O: begin
						case (current_instruction[8])
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
						endcase
						_pc <= #1 _pc + 1'b1;
					end
					RSET: begin
						case (current_instruction[8])
						R0 : begin
							_r0 <= #1 current_instruction[7:0];
							$display("RSET R0 ",_r0);
						end
						R1 : begin
							_r1 <= #1 current_instruction[7:0];
							$display("RSET R1 ",_r1);
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
