
`timescale 1ns/1ps
module a0(clock_signal, reset_signal, i0, i0_valid , i0_received, o0, o0_valid, o0_received);

	input clock_signal;
	input reset_signal;

	input [31:0] i0;
	input i0_valid;
	output i0_received;
	output [31:0] o0;
	output o0_valid;
	input o0_received;

	wire [1:0] rom_bus;
	wire [3:0] rom_value;


	p0 p0_instance(clock_signal, reset_signal, rom_bus, rom_value, i0, i0_valid , i0_received, o0, o0_valid, o0_received);
	p0rom p0rom_instance(rom_bus, rom_value);

endmodule
`timescale 1ns/1ps
module a1(clock_signal, reset_signal, i0, i0_valid , i0_received, i1, i1_valid , i1_received, o0, o0_valid, o0_received);

	input clock_signal;
	input reset_signal;

	input [31:0] i0;
	input i0_valid;
	output i0_received;
	input [31:0] i1;
	input i1_valid;
	output i1_received;
	output [31:0] o0;
	output o0_valid;
	input o0_received;

	wire [1:0] rom_bus;
	wire [3:0] rom_value;


	p1 p1_instance(clock_signal, reset_signal, rom_bus, rom_value, i0, i0_valid , i0_received, i1, i1_valid , i1_received, o0, o0_valid, o0_received);
	p1rom p1rom_instance(rom_bus, rom_value);

endmodule

	module bmaccelerator_v1_0 #
	(
		parameter integer C_S00_AXIS_TDATA_WIDTH	= 32,
		parameter integer C_M00_AXIS_TDATA_WIDTH	= 32,
		parameter integer C_M00_AXIS_START_COUNT	= 32
	)
	(
 
		input wire  s00_axis_aclk,
		input wire  s00_axis_aresetn,
		output wire  s00_axis_tready,
		input wire [C_S00_AXIS_TDATA_WIDTH-1 : 0] s00_axis_tdata,
		input wire  s00_axis_tlast,
		input wire  s00_axis_tvalid,
 
		input wire  m00_axis_aclk,
		input wire  m00_axis_aresetn,
		output wire  m00_axis_tvalid,
		output wire [C_M00_AXIS_TDATA_WIDTH-1 : 0] m00_axis_tdata,
		output wire  m00_axis_tlast,
		input wire  m00_axis_tready
	);
 
    /*
        NOW START THE AXIS SLAVE SECTION
    */

 
    localparam samples = 16; // number of samples that I expect from the client
    localparam bminputs = 1;  // number of bminputs for each sample (or bminputs)
    localparam bmoutputs = 1; // number of output for the classification
	localparam NUMBER_OF_INPUTS  = samples*bminputs;                                     
    localparam NUMBER_OF_OUTPUTS = samples*bmoutputs;
	localparam precision = 32; // precision bit
	localparam maxfifoloop = (C_S00_AXIS_TDATA_WIDTH / precision) - 1;
 
	// Machine state for the slave stream part
	parameter [1:0] IDLE = 1'b0,
	                WRITE_FIFO  = 1'b1; 
 
	wire  	   axis_tready;
	reg        mst_exec_state;     
	wire       fifo_wren;
	reg        fifo_full_flag;
	reg        writes_done;
    wire       test;
	reg [31:0] read_pointer;
	reg [31:0] read_pointer_output;
 
	reg [31:0] read_state;
	reg [31:0] out_read_state;
	reg  [31:0] outputs_counter = 0;
    reg  [31:0] outputs_counter_incr = 0;
	reg  [31:0] outputs_counter_pointer = 0;
    reg  [31:0] stream_output_counter = 0;
	reg  [31:0] maxfifoloopcounter = 0;
	reg [31:0] write_pointer;
 
	assign s00_axis_tready	= axis_tready;
 
	always @(posedge s00_axis_aclk) 
	begin  
	  if (!s00_axis_aresetn) 
	    begin
	      mst_exec_state <= IDLE;
	    end  
	  else
	    case (mst_exec_state)
	      IDLE:
	          if (s00_axis_tvalid)
	            begin
	              mst_exec_state <= WRITE_FIFO;
	            end
	          else
	            begin
	              mst_exec_state <= IDLE;
	            end
	      WRITE_FIFO:
	        if (writes_done)
	          begin
	            mst_exec_state <= IDLE;
	          end
	        else
	          begin
	            mst_exec_state <= WRITE_FIFO;
	          end
 
	    endcase
	end
 
	assign axis_tready = ((mst_exec_state == WRITE_FIFO) && (write_pointer <= (NUMBER_OF_INPUTS/(maxfifoloop+1))-1));
 
	always@(posedge s00_axis_aclk)
	begin
 
	  if (tx_done) begin
	       write_pointer <= 0;
	       writes_done <= 1'b0;
	  end
	  else begin
	  if(!s00_axis_aresetn)
	    begin
	      writes_done <= 1'b0;
	    end  
	  else
	    if (write_pointer <= (NUMBER_OF_INPUTS/(maxfifoloop+1))-1)
	      begin
	        if (fifo_wren)
	          begin
	            write_pointer <= write_pointer + 1;
	            writes_done <= 1'b0;
	          end
	          if ((write_pointer == (NUMBER_OF_INPUTS/(maxfifoloop+1))-1)|| s00_axis_tlast)
	            begin
	              writes_done <= 1'b1;
	            end
	      end
	      end
	end 
 

	localparam NUM_WORDS = NUMBER_OF_INPUTS / (C_S00_AXIS_TDATA_WIDTH / precision);
	assign fifo_wren = s00_axis_tvalid && axis_tready;
 
	(* ram_style = "block" *)
    reg [(C_S00_AXIS_TDATA_WIDTH)-1:0] stream_data_fifo [0 : NUM_WORDS - 1];
 
	(* ram_style = "block" *)
    reg [(C_S00_AXIS_TDATA_WIDTH)-1:0] stream_data_fifo_backup [0 : NUM_WORDS - 1];

	(* ram_style = "block" *)
    reg [(C_S00_AXIS_TDATA_WIDTH)-1:0] stream_data_fifo_backup_2 [0 : NUM_WORDS - 1];

	(* ram_style = "block" *)
    reg [(C_S00_AXIS_TDATA_WIDTH)-1:0] stream_data_fifo_backup_3 [0 : NUM_WORDS - 1];
 
 
	always @( posedge s00_axis_aclk )
    begin
		if (tx_done) begin
	       maxfifoloopcounter <= 0;
	  end
      if (fifo_wren)
        begin
          if (precision == 32) begin
				stream_data_fifo[write_pointer] <= s00_axis_tdata;
          end
          else if (precision == 16) begin
		  	stream_data_fifo[write_pointer][15:0]   <= s00_axis_tdata[15:0];
			stream_data_fifo_backup[write_pointer][15:0]  <= s00_axis_tdata[31:16];
          end else if (precision == 8) begin
		 	 stream_data_fifo[write_pointer][7:0]   <= s00_axis_tdata[7:0];
			 stream_data_fifo_backup[write_pointer][7:0]  <= s00_axis_tdata[15:8];
			 stream_data_fifo_backup_2[write_pointer][7:0]  <= s00_axis_tdata[23:16];
			 stream_data_fifo_backup_3[write_pointer][7:0]  <= s00_axis_tdata[31:24];
		  end
        end   
     end    
 
    /*
        NOW START THE MASTER AXIS SECTION
    */
 
	parameter [1:0] IDLE_M = 2'b00,                                             
	                INIT_COUNTER_M  = 2'b01, 
	                PROCESS_BM = 2'B10,   
	                SEND_STREAM_M   = 2'b11; 
 
	reg [1:0]   mst_exec_state_M;
    reg [8:0] 	count;
 
    wire  	axis_tvalid;
    reg  	axis_tvalid_delay;
    wire  	axis_tlast;
    reg  	axis_tlast_delay;
    wire  	tx_en;
	reg [C_M00_AXIS_TDATA_WIDTH-1 : 0] 	stream_data_out;
	reg  	tx_done;
    wire     bm_done;
 
	(* ram_style = "block" *)
    reg  [(C_S00_AXIS_TDATA_WIDTH)-1:0] output_stream_data_fifo [0 : NUMBER_OF_OUTPUTS-1];
 
    assign m00_axis_tvalid	= axis_tvalid_delay;
	assign m00_axis_tdata	= stream_data_out;
	assign m00_axis_tlast	= axis_tlast_delay;
 
	always @(posedge m00_axis_aclk)                                             
	begin                                                                     
	  if (!m00_axis_aresetn)                                                  
	    begin                                                                 
	      mst_exec_state_M <= IDLE_M;                                             
	      count    <= 0;                                                      
	    end                                                                   
	  else                                                                    
	    case (mst_exec_state_M)                                                 
	      IDLE_M:                                                         
	            mst_exec_state_M  <= INIT_COUNTER_M; 
 
	      INIT_COUNTER_M:                              
	        if ( count == 32 - 1 )                               
	          begin                                                           
	            mst_exec_state_M  <= PROCESS_BM;                               
	          end                                                             
	        else                                                              
	          begin                                                           
	            count <= count + 1;                                           
	            mst_exec_state_M  <= INIT_COUNTER_M;                              
	          end                                                             
 
	      PROCESS_BM:
	           if (!bm_done) 
	           begin
	               mst_exec_state_M <= PROCESS_BM;
	           end
	           else
	           begin
	               mst_exec_state_M <= SEND_STREAM_M;   
	           end
 
	      SEND_STREAM_M:                          
	        if (tx_done)                                                      
	          begin                                                           
	            mst_exec_state_M <= IDLE_M;                                       
	          end                                                             
	        else                                                              
	          begin                                                           
	            mst_exec_state_M <= SEND_STREAM_M;                                
	          end                                                             
	    endcase                                                               
	end
 
	assign axis_tvalid = ((mst_exec_state_M == SEND_STREAM_M) && (writes_done) && (bm_done));
    assign axis_tlast = (stream_output_counter == (NUMBER_OF_OUTPUTS/(maxfifoloop+1)) - 1);
 
    always @(posedge m00_axis_aclk)                                                                  
	begin        
	if (tx_done) begin
	       axis_tvalid_delay <= 1'b0;                                                               
	      axis_tlast_delay <= 1'b0;         
	end     
	else begin                                                                             
	  if (!m00_axis_aresetn)                                                                         
	    begin                                                                                      
	      axis_tvalid_delay <= 1'b0;                                                               
	      axis_tlast_delay <= 1'b0;                                                                
	    end                                                                                        
	  else                                                                                         
	    begin                                                                                      
	      axis_tvalid_delay <= axis_tvalid;                                                        
	      axis_tlast_delay <= axis_tlast;                                                          
	    end                                                                                        
	end 
	end
	reg [31:0] i0_r = 32'b0;
	reg [31:0] o0_received_r;
	wire [31:0] i0;
	wire i0_valid;
	wire i0_received;
	reg i0_valid_r = 1'b0;
	wire [31:0] o0;
	wire o0_valid;
	wire o0_received;
	reg  o0_valid_r = 1'b0;
	assign i0 = i0_r;
	assign i0_valid = i0_valid_r;
	assign o0_received = o0_received_r;
 
    assign bm_done = (outputs_counter_incr == samples);
	reg[1:0] send = 2'b00;
 
	bondmachine bm(.clk(m00_axis_aclk),
	.reset(!m00_axis_aresetn),
	.i0(i0),
	.i0_valid(i0_valid),
	.i0_received(i0_received),
	.o0(o0),
	.o0_valid(o0_valid),
	.o0_received(o0_received)
	);
 
	reg [2:0] counter;
	reg [15:0] output_mutex = 1;
	reg [15:0] input_reader = 1;
	reg [15:0] input_reader_index = 0;
 
	always @( posedge m00_axis_aclk )                  
	begin        
        if (tx_done) begin
            outputs_counter <= 0;
            outputs_counter_incr <= 0;
			outputs_counter_pointer <= 0;           
			o0_received_r <= 32'b0;
            o0_valid_r <= 32'b0;
			read_pointer <= 1'b0;
			read_pointer_output  <= 1'b0;
			output_mutex <= 1;
			input_reader_index <= 0;
        end
        else begin
            if (writes_done && !bm_done) begin
				if (send == 2'b00) begin
					send <= 2'b01;
				end
				else if (send == 2'b01) begin
 
					
						case (read_state)
							32'd0: begin
								i0_r <= stream_data_fifo[outputs_counter+0];
								read_pointer <= read_pointer + 1;	
								//i0_valid_r <= 1'b1;
								read_state <= 32'd0;
							end
						endcase
 
						if (read_pointer >= (bminputs)) begin
							read_state <= 32'd0;
							read_pointer <= 0;
							send <= 2'b10;
							i0_valid_r <= 1'b1;
						end
					

					
				end
				else if (send == 2'b10) begin
					 if (
					i0_received
					) begin
						i0_valid_r <= 1'b0;
 
						send <= 2'b11;
					end
				end
				else if (send == 2'b11) begin

					
					if (precision == 32) begin
						case (out_read_state)
							32'd0: begin
								if ( o0_valid && !o0_received_r) begin
									o0_valid_r <= 1'b1;
									o0_received_r <= 1'b1;
									output_stream_data_fifo[read_pointer_output] <= o0;
									read_pointer_output <= read_pointer_output + 1;
									out_read_state <= 32'd0;
								end
							end
						endcase
					end
					
					
					


					if (
				o0_valid_r
 
			 ) begin
				o0_valid_r <= 1'b0;
 
			end
			else if(
				!o0_valid && o0_received_r
			) 
			begin
					o0_received_r <= 1'b0;
					outputs_counter_pointer <= outputs_counter_pointer + bmoutputs;
					outputs_counter_incr <= outputs_counter_incr + 1;
 
					if (precision == 32) begin
						outputs_counter <= bminputs*(outputs_counter_incr+1);
					end else if (precision == 16) begin
						outputs_counter <= (bminputs*(outputs_counter_incr+1)) / 2;
					end else if (precision == 8) begin
						outputs_counter <= (bminputs*(outputs_counter_incr+1)) / 4;
					end
					send <= 2'b01;
			end
			end
			end
	    end
	end
 
	assign tx_en = m00_axis_tready && axis_tvalid;  
 
	reg [10:0] maxfifoloopcounteroutput = 0;
 
	always @( posedge m00_axis_aclk )                  
    begin        
       if (tx_done) begin
		maxfifoloopcounteroutput <= 0;
        stream_output_counter <= 0;
        tx_done <= 1'b0;
       end
 
      if(!m00_axis_aresetn)                            
        begin   
          stream_data_out <= 1;                      
        end                                          
      else if (tx_en)
        begin
           if (stream_output_counter <= (NUMBER_OF_OUTPUTS/(maxfifoloop+1)) - 1) begin
              if (precision == 32) begin
                    stream_data_out <= output_stream_data_fifo[stream_output_counter];
              end
              else if (precision == 16) begin
					stream_data_out[15:0] <= output_stream_data_fifo[stream_output_counter][15:0];
					stream_data_out[31:16] <= output_stream_data_fifo[stream_output_counter][31:16];
              end
			  else if (precision == 8) begin
					stream_data_out[7:0] <= output_stream_data_fifo[stream_output_counter][7:0];
					stream_data_out[15:8] <= output_stream_data_fifo[stream_output_counter][15:8];
					stream_data_out[23:16] <= output_stream_data_fifo[stream_output_counter][23:16];
					stream_data_out[31:24] <= output_stream_data_fifo[stream_output_counter][31:24];
              end
              stream_output_counter <= stream_output_counter + 1;
              if (stream_output_counter == (NUMBER_OF_OUTPUTS/(maxfifoloop+1)) - 1) begin
                    tx_done <= 1'b1;
              end
          end
        end                                          
    end
 
endmodule
 
module bondmachine(clk, reset, i0, i0_valid, i0_received, o0, o0_valid, o0_received);

	input clk, reset;
	input [31:0] i0;
	input i0_valid;
	output i0_received;
	//--------------Output Ports-----------------------
	output [31:0] o0;
	output o0_valid;
	input o0_received;



	wire [31:0] p0o0;
	wire p0o0_valid;
	wire p0o0_received;
	wire p1i1_received;
	wire [31:0] p1o0;
	wire p1o0_valid;
	wire p1o0_received;
	wire o0_received;
	wire [31:0] i0;
	wire i0_valid;
	wire i0_received;
	wire p0i0_received;
	wire p1i0_received;


	//Instantiation of the Processors and Shared Objects
	a0 a0_inst(clk, reset, i0, i0_valid, p0i0_received, p0o0, p0o0_valid, p0o0_received);
	a1 a1_inst(clk, reset, i0, i0_valid, p1i0_received, p0o0, p0o0_valid, p1i1_received, p1o0, p1o0_valid, p1o0_received);

	assign o0 = p1o0;
	assign o0_valid = p1o0_valid;

	assign p0o0_received = p1i1_received;
	assign p1o0_received = o0_received;
	assign i0_received = ( 1'b1 
		& (p0i0_received) 
		& (p1i0_received) 
		);

endmodule
`timescale 1ns/1ps
module p0rom(input [1:0] rom_bus, output [3:0] rom_value);
	reg [3:0] _rom [0:3];
	initial
	begin
	_rom[0] = 4'b0000;
	_rom[1] = 4'b1000;
	_rom[2] = 4'b0100;
	end
	assign rom_value = _rom[rom_bus];
endmodule
`timescale 1ns/1ps
module p0(clock_signal, reset_signal, rom_bus, rom_value, i0, i0_valid, i0_received, o0, o0_valid, o0_received);

	input clock_signal;
	input reset_signal;
	output  [1:0] rom_bus;
	input  [3:0] rom_value;

	input [31:0] i0;
	input i0_valid;
	output i0_received;
	output [31:0] o0;
	output o0_valid;
	input o0_received;

			// Opcodes in the instructions, length according the number of the selected.
	localparam	I2RW=2'b00,          // Sync input to register
			J=2'b01,          // Jump to a program location
			R2OWA=2'b10;          // Register to output

	localparam	R0=1'b0,		// Registers in the intructions
			R1=1'b1;
	localparam			I0=1'b0;
	localparam			O0=1'b0;
	reg [31:0] _auxo0;

	reg [31:0] _ram [0:0];		// Internal processor RAM

	(* KEEP = "TRUE" *) reg [1:0] _pc;		// Program counter

	// The number of registers are 2^R, two letters and an underscore as identifier , maximum R=8 and 265 rigisters
	(* KEEP = "TRUE" *) reg [31:0] _r0;
	(* KEEP = "TRUE" *) reg [31:0] _r1;

	wire [3:0] current_instruction;
	assign current_instruction=rom_value;


	reg i0_recv;

	always @(posedge clock_signal, posedge reset_signal)
	begin
		if (reset_signal)
		begin
			i0_recv <= #1 1'b0;
		end
		else
		begin
			case(current_instruction[3:2])
				I2RW: begin
					case (current_instruction[0])
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
			case(current_instruction[3:2])
				R2OWA: begin
					case (current_instruction[0])
					O0 : begin
						if (waitsm == 1'b1) o0_val <= 1'b1;
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
			_r0 <= #1 32'h0;
			_r1 <= #1 32'h0;
		end
		else begin
			// ha placeholder
			$display("Program Counter:%d", _pc);
			$display("Instruction:%b", rom_value);
			$display("Registers r0:%b r1:%b ", _r0, _r1);
				case(current_instruction[3:2])
					I2RW: begin
						case (current_instruction[1])
						R0 : begin
							case (current_instruction[0])
							I0 : begin
								if (i0_valid)
								begin
									_r0 <= #1 i0;
									_pc <= #1 _pc + 1'b1;
									$display("I2RW R0 I0");
								end
							end
							endcase
						end
						R1 : begin
							case (current_instruction[0])
							I0 : begin
								if (i0_valid)
								begin
									_r1 <= #1 i0;
									_pc <= #1 _pc + 1'b1;
									$display("I2RW R1 I0");
								end
							end
							endcase
						end
						endcase
					end
					J: begin
						_pc <= #1 current_instruction[1:0];
						$display("J ", current_instruction[1:0]);
					end
					R2OWA: begin
						case (current_instruction[1])
						R0 : begin
							case (current_instruction[0])
							O0 : begin
								if (waitsm == 1'b0) begin
									if (!o0_received) begin
										waitsm <= 1'b1;
									end
								end else begin
									_auxo0 <= #1 _r0;
									if (o0_received) begin
										_pc <= #1 _pc + 1'b1;
										waitsm <= 1'b0;
									end
								end
								$display("R2OWA R0 O0");
							end
							endcase
						end
						R1 : begin
							case (current_instruction[0])
							O0 : begin
								if (waitsm == 1'b0) begin
									if (!o0_received) begin
										waitsm <= 1'b1;
									end
								end else begin
									_auxo0 <= #1 _r1;
									if (o0_received) begin
										_pc <= #1 _pc + 1'b1;
										waitsm <= 1'b0;
									end
								end
								$display("R2OWA R1 O0");
							end
							endcase
						end
						endcase
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
	assign i0_received = i0_recv;
	assign o0 = _auxo0;
	assign o0_valid = o0_val;
endmodule
`timescale 1ns/1ps
module p1rom(input [1:0] rom_bus, output [3:0] rom_value);
	reg [3:0] _rom [0:3];
	initial
	begin
	_rom[0] = 4'b1000;
	_rom[1] = 4'b1001;
	_rom[2] = 4'b0100;
	_rom[3] = 4'b0000;
	end
	assign rom_value = _rom[rom_bus];
endmodule
`timescale 1ns/1ps
module p1(clock_signal, reset_signal, rom_bus, rom_value, i0, i0_valid, i0_received, i1, i1_valid, i1_received, o0, o0_valid, o0_received);

	input clock_signal;
	input reset_signal;
	output  [1:0] rom_bus;
	input  [3:0] rom_value;

	input [31:0] i0;
	input i0_valid;
	output i0_received;
	input [31:0] i1;
	input i1_valid;
	output i1_received;
	output [31:0] o0;
	output o0_valid;
	input o0_received;

			// Opcodes in the instructions, length according the number of the selected.
	localparam	J=2'b00,          // Jump to a program location
			R2OWA=2'b01,          // Register to output
			SICV3=2'b10;          // Wait for an input change via valid and increments a register

	localparam	R0=1'b0,		// Registers in the intructions
			R1=1'b1;
	localparam			I0=1'b0,
			I1=1'b1;
	localparam			O0=1'b0;
	reg [31:0] _auxo0;

	reg [31:0] _ram [0:0];		// Internal processor RAM

	(* KEEP = "TRUE" *) reg [1:0] _pc;		// Program counter

	// The number of registers are 2^R, two letters and an underscore as identifier , maximum R=8 and 265 rigisters
	(* KEEP = "TRUE" *) reg [31:0] _r0;
	(* KEEP = "TRUE" *) reg [31:0] _r1;

	wire [3:0] current_instruction;
	assign current_instruction=rom_value;


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
			case(current_instruction[3:2])
				R2OWA: begin
					case (current_instruction[0])
					O0 : begin
						if (waitsm == 1'b1) o0_val <= 1'b1;
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

	// Signals for sicv3 instruction
	reg sicv3_state;
	localparam SICV3IDLE = 1'b0;
	localparam SICV3WAIT = 1'b1;

	initial begin
		sicv3_state = SICV3IDLE;
	end

	reg i0_recv;
	reg i1_recv;

	always @(posedge clock_signal, posedge reset_signal)
	begin
		if (reset_signal)
		begin
			i0_recv <= #1 1'b0;
		end
		else
		begin
			case(current_instruction[3:2])
				SICV3: begin
					case (current_instruction[0])
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
	always @(posedge clock_signal, posedge reset_signal)
	begin
		if (reset_signal)
		begin
			i1_recv <= #1 1'b0;
		end
		else
		begin
			case(current_instruction[3:2])
				SICV3: begin
					case (current_instruction[0])
					I1 : begin
						if (i1_valid)
						begin
							i1_recv <= #1 1'b1;
						end else begin
							i1_recv <= #1 1'b0;
						end
					end
					default: begin
						if (!i1_valid)
						begin
							i1_recv <= #1 1'b0;
						end
					end
					endcase
				end
				default: begin
					if (!i1_valid)
					begin
						i1_recv <= #1 1'b0;
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
			_r0 <= #1 32'h0;
			_r1 <= #1 32'h0;
		end
		else begin
			// ha placeholder
			$display("Program Counter:%d", _pc);
			$display("Instruction:%b", rom_value);
			$display("Registers r0:%b r1:%b ", _r0, _r1);
				case(current_instruction[3:2])
					J: begin
						_pc <= #1 current_instruction[1:0];
						$display("J ", current_instruction[1:0]);
					end
					R2OWA: begin
						case (current_instruction[1])
						R0 : begin
							case (current_instruction[0])
							O0 : begin
								if (waitsm == 1'b0) begin
									if (!o0_received) begin
										waitsm <= 1'b1;
									end
								end else begin
									_auxo0 <= #1 _r0;
									if (o0_received) begin
										_pc <= #1 _pc + 1'b1;
										waitsm <= 1'b0;
									end
								end
								$display("R2OWA R0 O0");
							end
							endcase
						end
						R1 : begin
							case (current_instruction[0])
							O0 : begin
								if (waitsm == 1'b0) begin
									if (!o0_received) begin
										waitsm <= 1'b1;
									end
								end else begin
									_auxo0 <= #1 _r1;
									if (o0_received) begin
										_pc <= #1 _pc + 1'b1;
										waitsm <= 1'b0;
									end
								end
								$display("R2OWA R1 O0");
							end
							endcase
						end
						endcase
					end
					SICV3: begin
						case (current_instruction[1])
						R0 : begin
							case (current_instruction[0])
							I0 : begin
								if (i0_valid) begin
									if (sicv3_state == SICV3IDLE) begin
										_r0 <= #1 32'd0;
										sicv3_state <= SICV3WAIT;
									end else begin
										sicv3_state <= SICV3IDLE;
									end
									_pc <= #1 _pc + 1'b1;
									$display("SICV3 R0 I0");
								end else begin
									if (sicv3_state == SICV3WAIT) begin
										_r0 <= #1 _r0 + 1;
									end
								end
							end
							I1 : begin
								if (i1_valid) begin
									if (sicv3_state == SICV3IDLE) begin
										_r0 <= #1 32'd0;
										sicv3_state <= SICV3WAIT;
									end else begin
										sicv3_state <= SICV3IDLE;
									end
									_pc <= #1 _pc + 1'b1;
									$display("SICV3 R0 I1");
								end else begin
									if (sicv3_state == SICV3WAIT) begin
										_r0 <= #1 _r0 + 1;
									end
								end
							end
							endcase
						end
						R1 : begin
							case (current_instruction[0])
							I0 : begin
								if (i0_valid) begin
									if (sicv3_state == SICV3IDLE) begin
										_r1 <= #1 32'd0;
										sicv3_state <= SICV3WAIT;
									end else begin
										sicv3_state <= SICV3IDLE;
									end
									_pc <= #1 _pc + 1'b1;
									$display("SICV3 R1 I0");
								end else begin
									if (sicv3_state == SICV3WAIT) begin
										_r1 <= #1 _r1 + 1;
									end
								end
							end
							I1 : begin
								if (i1_valid) begin
									if (sicv3_state == SICV3IDLE) begin
										_r1 <= #1 32'd0;
										sicv3_state <= SICV3WAIT;
									end else begin
										sicv3_state <= SICV3IDLE;
									end
									_pc <= #1 _pc + 1'b1;
									$display("SICV3 R1 I1");
								end else begin
									if (sicv3_state == SICV3WAIT) begin
										_r1 <= #1 _r1 + 1;
									end
								end
							end
							endcase
						end
						endcase
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
	assign i0_received = i0_recv;
	assign i1_received = i1_recv;
	assign o0 = _auxo0;
	assign o0_valid = o0_val;
endmodule
