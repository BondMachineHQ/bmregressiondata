
`timescale 1ns/1ps
module a0(clock_signal, reset_signal, o0, o0_valid, o0_received);

	input clock_signal;
	input reset_signal;

	output [15:0] o0;
	output o0_valid;
	input o0_received;

	wire [4:0] rom_bus;
	wire [22:0] rom_value;


	p0 p0_instance(clock_signal, reset_signal, rom_bus, rom_value, o0, o0_valid, o0_received);
	p0rom p0rom_instance(rom_bus, rom_value);

endmodule
module bondmachine_main(

	input clk,
	input btnC,
	output reg [15:0] led
);

	// External ports creation ended
	assign reset = btnC;
	wire [15:0] Output0;

	// Processing per-extramodule initializations
	// Processing BM ports originated from external modules (not IO)
	bondmachine bondmachine_inst (clk, reset, Output0, Output0_valid, Output0_received);

	// Processing BM connected firmwares

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
endmodule
module bondmachine(clk, reset, o0, o0_valid, o0_received);

	// Clock and reset input ports
	input clk, reset;
	//--------------Output Ports-----------------------
	output [15:0] o0;
	output o0_valid;
	input o0_received;



	//Analyzing Internal output p0o0
	//Internal output p0o0 is connected to o0
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
module p0rom(input [4:0] rom_bus, output [22:0] rom_value);
	reg [22:0] _rom [0:31];
	initial
	begin
	_rom[0] = 23'b10100110000000000000001;
	_rom[1] = 23'b10101000000000000000000;
	_rom[2] = 23'b10100000010011100010000;
	_rom[3] = 23'b10100010000001111101000;
	_rom[4] = 23'b00001010000000000000000;
	_rom[5] = 23'b01101000110000000000000;
	_rom[6] = 23'b00100110000000000000000;
	_rom[7] = 23'b01100110100100000000000;
	_rom[8] = 23'b01011001000000000000000;
	_rom[9] = 23'b10101000000000000000000;
	_rom[10] = 23'b10100110000000000000001;
	_rom[11] = 23'b01011001000000000000000;
	_rom[12] = 23'b00010110000000000000000;
	_rom[13] = 23'b01100110111100000000000;
	_rom[14] = 23'b01011001000000000000000;
	_rom[15] = 23'b10101000000000000000001;
	_rom[16] = 23'b10100111000000000000000;
	_rom[17] = 23'b01011001000000000000000;
	_rom[18] = 23'b10000110000000000000000;
	_rom[19] = 23'b01010001000000000000000;
	_rom[20] = 23'b01100001101100000000000;
	_rom[21] = 23'b00110100010000000000000;
	_rom[22] = 23'b01000000000000000000000;
	_rom[23] = 23'b01100101101000000000000;
	_rom[24] = 23'b01000100000000000000000;
	_rom[25] = 23'b01011011100000000000000;
	_rom[26] = 23'b01011010000000000000000;
	_rom[27] = 23'b01110000000000000000000;
	_rom[28] = 23'b10010000000000000000000;
	end
	assign rom_value = _rom[rom_bus];
endmodule
`timescale 1ns/1ps
module p0(clock_signal, reset_signal, rom_bus, rom_value, o0, o0_valid, o0_received);

	input clock_signal;
	input reset_signal;
	output  [4:0] rom_bus;
	input  [22:0] rom_value;

	output [15:0] o0;
	output o0_valid;
	input o0_received;

			// Opcodes in the instructions, length according the number of the selected.
	localparam	CALLO8S=4'b0000,          // Call a rom subroutine via an hardware stack called s with depth 8
			CIL=4'b0001,          // Register left shift
			CIR=4'b0010,          // Register right shift
			CPY=4'b0011,          // Copy from a register to another
			DEC=4'b0100,          // Decrement a register by 1
			J=4'b0101,          // Jump to a program location
			JZ=4'b0110,          // Zero conditional jump
			NOP=4'b0111,          // No operation
			R2O=4'b1000,          // Register to output
			RET8S=4'b1001,          // Return from a subroutine via an hardware stack called s with depth 8
			RSETS16=4'b1010;          // Register set value with fixed size

	localparam	R0=3'b000,		// Registers in the intructions
			R1=3'b001,
			R2=3'b010,
			R3=3'b011,
			R4=3'b100,
			R5=3'b101,
			R6=3'b110,
			R7=3'b111;
	localparam			O0=1'b0;
	reg [15:0] _auxo0;

	reg [15:0] _ram [0:0];		// Internal processor RAM

	(* KEEP = "TRUE" *) reg [4:0] _pc;		// Program counter

	// The number of registers are 2^R, two letters and an underscore as identifier , maximum R=8 and 265 rigisters
	(* KEEP = "TRUE" *) reg [15:0] _r0;
	(* KEEP = "TRUE" *) reg [15:0] _r1;
	(* KEEP = "TRUE" *) reg [15:0] _r2;
	(* KEEP = "TRUE" *) reg [15:0] _r3;
	(* KEEP = "TRUE" *) reg [15:0] _r4;
	(* KEEP = "TRUE" *) reg [15:0] _r5;
	(* KEEP = "TRUE" *) reg [15:0] _r6;
	(* KEEP = "TRUE" *) reg [15:0] _r7;

	wire [22:0] current_instruction;
	assign current_instruction=rom_value;


// Start of the component "header" for the opcode callo8s

	reg [1:0] restack0_8sSM;
	localparam	CALL1 = 2'b00,
			CALL2 = 2'b01,
			CALL3 = 2'b10,
			CALL4 = 2'b11;

	reg [4:0] restack0_8ssenderData;
	reg restack0_8ssenderWrite;
	wire restack0_8ssenderAck;

	wire [4:0] restack0_8sreceiverData;
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

// Start of the component "header" for the opcode cil


// Start of the component "header" for the opcode cir


// Start of the component "header" for the opcode cpy


// Start of the component "header" for the opcode dec


// Start of the component "header" for the opcode j


// Start of the component "header" for the opcode jz


// Start of the component "header" for the opcode nop


// Start of the component "header" for the opcode r2o


	reg o0_val;
	reg waitsm;
	initial waitsm = 1'b0;

	always @(posedge clock_signal, posedge reset_signal)
	begin
		if (reset_signal)
		begin
			o0_val <= #1 1'b0;
		end
		else
		begin
			case(current_instruction[22:19])
				R2O: begin
					case (current_instruction[15])
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

// Start of the component "header" for the opcode ret8s


// Start of the component "header" for the opcode rsets16


	always @(posedge clock_signal, posedge reset_signal)
	begin
		if(reset_signal)
		begin
			_pc <= #1 5'h0;
			_r0 <= #1 16'h0;
			_r1 <= #1 16'h0;
			_r2 <= #1 16'h0;
			_r3 <= #1 16'h0;
			_r4 <= #1 16'h0;
			_r5 <= #1 16'h0;
			_r6 <= #1 16'h0;
			_r7 <= #1 16'h0;

// Start of the component "reset" for the opcode callo8s


// Start of the component "reset" for the opcode cil


// Start of the component "reset" for the opcode cir


// Start of the component "reset" for the opcode cpy


// Start of the component "reset" for the opcode dec


// Start of the component "reset" for the opcode j


// Start of the component "reset" for the opcode jz


// Start of the component "reset" for the opcode nop


// Start of the component "reset" for the opcode r2o


// Start of the component "reset" for the opcode ret8s


// Start of the component "reset" for the opcode rsets16

		end
		else begin
			// ha placeholder
			$display("Program Counter:%d", _pc);
			$display("Instruction:%b", rom_value);
			$display("Registers r0:%b r1:%b r2:%b r3:%b r4:%b r5:%b r6:%b r7:%b ", _r0, _r1, _r2, _r3, _r4, _r5, _r6, _r7);

// Start of the component "internal state" for the opcode callo8s


// Start of the component "internal state" for the opcode cil


// Start of the component "internal state" for the opcode cir


// Start of the component "internal state" for the opcode cpy


// Start of the component "internal state" for the opcode dec


// Start of the component "internal state" for the opcode j


// Start of the component "internal state" for the opcode jz


// Start of the component "internal state" for the opcode nop


// Start of the component "internal state" for the opcode r2o


// Start of the component "internal state" for the opcode ret8s


// Start of the component "internal state" for the opcode rsets16


// Start of the component "default state" for the opcode callo8s


// Start of the component "default state" for the opcode cil


// Start of the component "default state" for the opcode cir


// Start of the component "default state" for the opcode cpy


// Start of the component "default state" for the opcode dec


// Start of the component "default state" for the opcode j


// Start of the component "default state" for the opcode jz


// Start of the component "default state" for the opcode nop


// Start of the component "default state" for the opcode r2o


// Start of the component "default state" for the opcode ret8s


// Start of the component "default state" for the opcode rsets16

				case(current_instruction[22:19])

// Start of the component of the "state machine" for the opcode callo8s

					CALLO8S: begin
						case (restack0_8sSM)
						CALL1: begin
							if (!restack0_8ssenderAck) begin
							     restack0_8ssenderData[4:0] <= #1 _pc + 1;
							     restack0_8ssenderWrite <= #1 1'b1;
							     restack0_8sSM <= CALL2;
							end
						end
						CALL2: begin
							if (restack0_8ssenderAck) begin
								restack0_8ssenderWrite <= #1 1'b0;
								_pc <= #1 current_instruction[18:14];
								$display("CALLO8S ", current_instruction[18:14]);
								restack0_8sSM <= CALL1;
							end
						end
						endcase
					end

// Start of the component of the "state machine" for the opcode cil

					CIL: begin
						case (current_instruction[18:16])
						R0 : begin
								_r0 <= #1 _r0<< 1'b1;
								$display("CIL R0");
						end
						R1 : begin
								_r1 <= #1 _r1<< 1'b1;
								$display("CIL R1");
						end
						R2 : begin
								_r2 <= #1 _r2<< 1'b1;
								$display("CIL R2");
						end
						R3 : begin
								_r3 <= #1 _r3<< 1'b1;
								$display("CIL R3");
						end
						R4 : begin
								_r4 <= #1 _r4<< 1'b1;
								$display("CIL R4");
						end
						R5 : begin
								_r5 <= #1 _r5<< 1'b1;
								$display("CIL R5");
						end
						R6 : begin
								_r6 <= #1 _r6<< 1'b1;
								$display("CIL R6");
						end
						R7 : begin
								_r7 <= #1 _r7<< 1'b1;
								$display("CIL R7");
						end
						endcase
						_pc <= #1 _pc + 1'b1;
					end

// Start of the component of the "state machine" for the opcode cir

					CIR: begin
						case (current_instruction[18:16])
						R0 : begin
								_r0 <= #1 _r0>> 1'b1;
								$display("CIR R0");
						end
						R1 : begin
								_r1 <= #1 _r1>> 1'b1;
								$display("CIR R1");
						end
						R2 : begin
								_r2 <= #1 _r2>> 1'b1;
								$display("CIR R2");
						end
						R3 : begin
								_r3 <= #1 _r3>> 1'b1;
								$display("CIR R3");
						end
						R4 : begin
								_r4 <= #1 _r4>> 1'b1;
								$display("CIR R4");
						end
						R5 : begin
								_r5 <= #1 _r5>> 1'b1;
								$display("CIR R5");
						end
						R6 : begin
								_r6 <= #1 _r6>> 1'b1;
								$display("CIR R6");
						end
						R7 : begin
								_r7 <= #1 _r7>> 1'b1;
								$display("CIR R7");
						end
						endcase
						_pc <= #1 _pc + 1'b1;
					end

// Start of the component of the "state machine" for the opcode cpy

					CPY: begin
						case (current_instruction[18:16])
						R0 : begin
							case (current_instruction[15:13])
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
							case (current_instruction[15:13])
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
							case (current_instruction[15:13])
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
							case (current_instruction[15:13])
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
							case (current_instruction[15:13])
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
							case (current_instruction[15:13])
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
							case (current_instruction[15:13])
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
							case (current_instruction[15:13])
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
						_pc <= #1 _pc + 1'b1;
					end

// Start of the component of the "state machine" for the opcode dec

					DEC: begin
						case (current_instruction[18:16])
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
						_pc <= #1 _pc + 1'b1;
					end

// Start of the component of the "state machine" for the opcode j

					J: begin
						_pc <= #1 current_instruction[18:14];
						$display("J ", current_instruction[18:14]);
					end

// Start of the component of the "state machine" for the opcode jz

					JZ: begin
						case (current_instruction[18:16])
							R0 : begin
								if(_r0 == 'b0) begin
								_pc <= #1 current_instruction[15:11];
								end
								else begin
									_pc <= #1 _pc + 1'b1;
								end
								$display("JZ R0 ",_r0);
							end
							R1 : begin
								if(_r1 == 'b0) begin
								_pc <= #1 current_instruction[15:11];
								end
								else begin
									_pc <= #1 _pc + 1'b1;
								end
								$display("JZ R1 ",_r1);
							end
							R2 : begin
								if(_r2 == 'b0) begin
								_pc <= #1 current_instruction[15:11];
								end
								else begin
									_pc <= #1 _pc + 1'b1;
								end
								$display("JZ R2 ",_r2);
							end
							R3 : begin
								if(_r3 == 'b0) begin
								_pc <= #1 current_instruction[15:11];
								end
								else begin
									_pc <= #1 _pc + 1'b1;
								end
								$display("JZ R3 ",_r3);
							end
							R4 : begin
								if(_r4 == 'b0) begin
								_pc <= #1 current_instruction[15:11];
								end
								else begin
									_pc <= #1 _pc + 1'b1;
								end
								$display("JZ R4 ",_r4);
							end
							R5 : begin
								if(_r5 == 'b0) begin
								_pc <= #1 current_instruction[15:11];
								end
								else begin
									_pc <= #1 _pc + 1'b1;
								end
								$display("JZ R5 ",_r5);
							end
							R6 : begin
								if(_r6 == 'b0) begin
								_pc <= #1 current_instruction[15:11];
								end
								else begin
									_pc <= #1 _pc + 1'b1;
								end
								$display("JZ R6 ",_r6);
							end
							R7 : begin
								if(_r7 == 'b0) begin
								_pc <= #1 current_instruction[15:11];
								end
								else begin
									_pc <= #1 _pc + 1'b1;
								end
								$display("JZ R7 ",_r7);
							end
						endcase
					end

// Start of the component of the "state machine" for the opcode nop

					NOP: begin
						$display("NOP");
						_pc <= #1 _pc + 1'b1;
					end

// Start of the component of the "state machine" for the opcode r2o

					R2O: begin
						case (current_instruction[18:16])
						R0 : begin
							case (current_instruction[15])
							O0 : begin
								_auxo0 <= #1 _r0;
								$display("R2O R0 O0");
							end
							endcase
						end
						R1 : begin
							case (current_instruction[15])
							O0 : begin
								_auxo0 <= #1 _r1;
								$display("R2O R1 O0");
							end
							endcase
						end
						R2 : begin
							case (current_instruction[15])
							O0 : begin
								_auxo0 <= #1 _r2;
								$display("R2O R2 O0");
							end
							endcase
						end
						R3 : begin
							case (current_instruction[15])
							O0 : begin
								_auxo0 <= #1 _r3;
								$display("R2O R3 O0");
							end
							endcase
						end
						R4 : begin
							case (current_instruction[15])
							O0 : begin
								_auxo0 <= #1 _r4;
								$display("R2O R4 O0");
							end
							endcase
						end
						R5 : begin
							case (current_instruction[15])
							O0 : begin
								_auxo0 <= #1 _r5;
								$display("R2O R5 O0");
							end
							endcase
						end
						R6 : begin
							case (current_instruction[15])
							O0 : begin
								_auxo0 <= #1 _r6;
								$display("R2O R6 O0");
							end
							endcase
						end
						R7 : begin
							case (current_instruction[15])
							O0 : begin
								_auxo0 <= #1 _r7;
								$display("R2O R7 O0");
							end
							endcase
						end
						endcase
						_pc <= #1 _pc + 1'b1;
					end

// Start of the component of the "state machine" for the opcode ret8s

					RET8S: begin
						if (restack0_8sreceiverAck && restack0_8sreceiverRead) begin
							restack0_8sreceiverRead <= #1 1'b0;
							_pc[4:0] <= #1 restack0_8sreceiverData[4:0];
						end
						else begin
							restack0_8sreceiverRead <= #1 1'b1;
						end
					end

// Start of the component of the "state machine" for the opcode rsets16

					RSETS16: begin
						case (current_instruction[18:16])
						R0 : begin
							_r0 <= #1 current_instruction[15:0];
							$display("RSET R0 ",_r0);
						end
						R1 : begin
							_r1 <= #1 current_instruction[15:0];
							$display("RSET R1 ",_r1);
						end
						R2 : begin
							_r2 <= #1 current_instruction[15:0];
							$display("RSET R2 ",_r2);
						end
						R3 : begin
							_r3 <= #1 current_instruction[15:0];
							$display("RSET R3 ",_r3);
						end
						R4 : begin
							_r4 <= #1 current_instruction[15:0];
							$display("RSET R4 ",_r4);
						end
						R5 : begin
							_r5 <= #1 current_instruction[15:0];
							$display("RSET R5 ",_r5);
						end
						R6 : begin
							_r6 <= #1 current_instruction[15:0];
							$display("RSET R6 ",_r6);
						end
						R7 : begin
							_r7 <= #1 current_instruction[15:0];
							$display("RSET R7 ",_r7);
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
    input [4:0] senderData;
    input senderWrite;
    output reg senderAck;
    output reg [4:0] receiverData;
    input receiverRead;
    output reg receiverAck;

    reg [4:0] memory[7:0];
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
            receiverData <= 5'd0;
            receiverAck <= 1'b0;
            senderAck <= 1'b0;
            sendSM <= 1'd0;
            recvSM <= 1'd0;
            for (i=0;i<8;i=i+1) begin
                memory[i]<=5'd0;
            end
        end
        else begin
            // Read state machine part
            if (readneed && !empty) begin
                case (recvSM)
                1'd0: begin
                    if (receiverRead && !receiverAck) begin
                        receiverData[4:0] <= memory[sp-1];
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
                        memory[sp] <= senderData[4:0];
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
