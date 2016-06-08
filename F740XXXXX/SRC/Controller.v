// Controller

module Controller ( opcode,
					funct,
					Reg_imm,
					Jump,
					Branch,
					Jal,
					Jr,
					MemtoReg,
					ALUOp,
					RegWrite,
					MemWrite
					);

	input  [5:0] opcode;
    input  [5:0] funct;

	output Reg_imm;
	output Jump;
	output Branch;
	output Jal;
	output Jr;
	output MemtoReg;
	output [3:0] ALUOp;
	output RegWrite;
	output MemWrite;
	
	reg Reg_imm;
	reg Jump;
	reg Branch;
	reg Jal;
	reg Jr;
	reg MemtoReg;
	reg [3:0] ALUOp;
	reg RegWrite;
	reg MemWrite;
	
	// Reg_imm;
	parameter Reg_data = 0,
			  imm_data = 1;
	
	// ALU
	parameter //op_nop = 0,
	          op_add = 1,
	          op_sub = 2,
	          op_and = 3,
	          op_or  = 4,
	          op_xor = 5,
	          op_nor = 6,
	          op_slt = 7,
	          op_sll = 8,
	          op_srl = 9,
	          op_beq = 10,
	          op_bne = 11;
			  
	// controller
	always@ (*) begin
		Reg_imm	 = Reg_data;
		Jump	 = 0;
		Branch	 = 0;
		Jal		 = 0;
		Jr		 = 0;
		MemtoReg = 0;
		ALUOp	 = 0;
		RegWrite = 0;
		MemWrite = 0;		
		case (opcode)
		6'b00_0000 : begin // R type
			case (funct)
			6'b10_0000 : begin 
				ALUOp	 =  op_add; /* write operation code */ 
				RegWrite =  1; /* write enable signal */ 
			end
			6'b10_0010 : begin 
				ALUOp	 =  op_sub; /* write operation code  */
				RegWrite =  1; /* write enable signal */
			end
			6'b10_0100 : begin 
				ALUOp	 =  op_and; /* write operation code */
			    RegWrite =  1; /* write enable signal */
			end
			6'b10_0101 : begin 
				ALUOp	 =  op_or;  /* write operation code  */
			    RegWrite =  1; /* write enable signal */
			end
			6'b10_0110 : begin 
				ALUOp	 =  op_xor;/* write operation code  */
			 	RegWrite =  1;/* write enable signal */
			end
			6'b10_0111 : begin 
				ALUOp	 =  op_nor; /* write operation code  */
			    RegWrite =  1; /* write enable signal */
			end
			6'b10_1010 : begin 
				Branch	 = 0;/* write enable signal */
				ALUOp	 = op_slt; /* write operation code  */
				RegWrite = 1;/* write enable signal */
			end
			6'b00_0000 : begin 
				ALUOp	 = op_sll;  /* write operation code  */
				RegWrite = 1; /* write enable signal */
			end
			6'b00_0010 : begin 
				ALUOp	 = op_srl; /* write operation code  */
			    RegWrite = 1; /* write enable signal */
			end
			6'b00_1000 : begin 
				Jr		 = 1; /* write enable signal */
			end		
			endcase // funct
		end
		// I type
		6'b00_1000 : begin // addi
			Reg_imm	 = imm_data;
			  ALUOp	 = op_add/* write operation code  */;
			  RegWrite = 1/* write enable signal */;
		end
		6'b00_1100 : begin // andi
			Reg_imm	 = imm_data;
		      ALUOp	 = op_and/* write operation code  */;
		      RegWrite = 1/* write enable signal */;
		end
		6'b00_1010 : begin // slti
			Reg_imm	 = imm_data;
		      ALUOp	 = op_slt/* write operation code  */;
		      RegWrite = 1/* write enable signal */;
		end
		6'b00_0100 : begin // beq
			  Branch = 1/* write branch signal */;
			  ALUOp	 = op_beq/* write operation code  */;
		end
		6'b00_0101 : begin // bne
			  Branch = 1/* write branch signal */;
          ALUOp	 = op_bne/* write operation code  */;
		end
		6'b10_0011 : begin // lw
			Reg_imm	 = imm_data;
			MemtoReg = 1;
			  ALUOp	 = op_add/* write operation code  */;
		      RegWrite = 1/* write enable signal */;
		end
		6'b10_1011 : begin // sw
			Reg_imm	 = imm_data;
		      ALUOp	 = op_add/* write operation code  */;
			MemWrite = 1;
		end
		// J Type
		6'b00_0010 : begin
           Jump = 1/* write jump enable signal */; 
	    end	
		6'b00_0011 : begin 
			 Jump = 1/* write jump enable signal */; 
			 Jal		 = 1/* write jal enable signal */;
			RegWrite = 1;		
		end
		endcase // opcode
	end
	
endmodule




