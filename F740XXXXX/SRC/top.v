// top
`include "PC.v"
`include "ADD.v"
`include "IF_ID.v"
`include "HDU.v"
`include "Controller.v"
`include "Registers.v"
`include "sign_extend.v"
`include "Mux2to1.v"
`include "ID_EX.v"
`include "Mux4to1.v"
`include "Jump_Ctrl.v"
`include "FU.v"
`include "ALU.v"
`include "EX_M.v"
`include "M_WB.v"

/* Module top.v */
/* ------------------------------------------------------------------------------------------- */
/* Please complete the missing wire declaration and the wire connection with the missing ports */
/* You can refer to the Pipeline CPU datapath  - hw3_pipelinecpu.jpg  ------------------------ */
/* ------------------------------------------------------------------------------------------- */



module top ( clk,
             rst,
			 // Instruction Memory
			 IM_Address,
             Instruction,
			 // Data Memory
			 DM_Address,
			 DM_enable,
			 DM_Write_Data,
			 DM_Read_Data);

	parameter data_size = 32;
	parameter mem_size = 16;	
	parameter pc_size = 18;
	
	input  clk, rst;
	
	// Instruction Memory
	output [mem_size-1:0] IM_Address;	
	input  [data_size-1:0] Instruction;
	
	// Data Memory
	output [mem_size-1:0] DM_Address;
	output DM_enable;
	output [data_size-1:0] DM_Write_Data;	
    input  [data_size-1:0] DM_Read_Data;
	
	// Wire Declaration part-----------------------------------*/
	// PC
	wire [pc_size-1:0] PCout;	 
	wire [pc_size-1:0] PC_add4;
	
	// IF_ID pipe---------------------------------
	wire [pc_size-1:0]   ID_PC;
	wire [data_size-1:0] ID_ir;
	
	// Hazard Detection Unit
	/*please declare the wire used for Hazard Detection Unit here*/
	/*-----------------------------------------------------*/
	
	  		wire PCWrite;
	  		wire IF_IDWrite;
	  		wire IF_Flush;
	  		wire ID_Flush;
	
			//input section //
			wire [4:0] ID_Rs;
			wire [4:0] ID_Rt;
			//wire [4:0] EX_WR_out;
			//wire EX_MemtoReg;
			//wire [1:0] EX_JumpOP;
			//end input section//
	
	
	/*-----------------------------------------------------*/
	wire Branch_Flush;
	wire Load_wait;
	
	// Controller
	wire [5:0] opcode;
	wire [5:0] funct;
	wire Reg_imm;
	wire Jump;
	wire Branch;
	wire Jal;
	wire Jr;
	wire MemtoReg;
	wire [3:0] ALUOp;
	wire RegWrite;
	wire MemWrite;
	
	// Registers
	wire [4:0] Rd;
	wire [4:0] Rs;
	wire [4:0] Rt;
	wire [data_size-1:0] Rs_data;
	wire [data_size-1:0] Rt_data;
	wire [4:0] shamt;
	
	// sign_extend	
	wire [15:0] imm;
	wire [data_size-1:0] se_imm;	
	
	// ID Mux part
	wire [4:0] Rd_Rt_out;
	wire [4:0] WR_out;
	
	// ID_EX 
	/*please declare the wire used for ID_EX pipe here*/	
	/*Tip: There are control signal part and data part*/
	/*-----------------------------------------------------*/
	
	// WB
	  		wire EX_MemtoReg;
	  		wire EX_RegWrite;
	// M
	  		wire EX_MemWrite;
	  		wire EX_Jal;
	// EX
	  		wire EX_Reg_imm;
	  		wire EX_Jump;
	  		wire EX_Branch;
	  		wire EX_Jr;		
	// pipe
	  		wire [pc_size-1:0] EX_PC;
	  		wire [3:0] EX_ALUOp;
	  		wire [4:0] EX_shamt;
	  		wire [data_size-1:0] EX_Rs_data;
	  		wire [data_size-1:0] EX_Rt_data;
	  		wire [data_size-1:0] EX_se_imm;
	  		wire [4:0] EX_WR_out;
	  		wire [4:0] EX_Rs;
	  		wire [4:0] EX_Rt;



			// input section //
			//wire ID_Flush;
	
			// WB
			wire ID_MemtoReg;
			wire ID_RegWrite;
			// M
			wire ID_MemWrite;
			wire ID_Jal;
			// EX
			wire ID_Reg_imm;
			wire ID_Jump;
			wire ID_Branch;
			wire ID_Jr;		
			// pipe
    	    //wire [pc_size-1:0] ID_PC;
    		wire [3:0] ID_ALUOp;
    		wire [4:0] ID_shamt;
    		wire [data_size-1:0] ID_Rs_data;
    		wire [data_size-1:0] ID_Rt_data;
    		wire [data_size-1:0] ID_se_imm;
    		wire [4:0] ID_WR_out;
    		//wire [4:0] ID_Rs;
    		//wire [4:0] ID_Rt;
			// end input section //
	
	
	
	/*-----------------------------------------------------*/
	//Jump Part
	wire [pc_size-1:0] BranchAddr;
	wire [pc_size-1:0] JumpAddr;
	wire [1:0] EX_JumpOP;
	wire [pc_size-1:0] PCin;
	
    		// input section //
			//wire Branch;
    		wire Zero;
    		//wire Jr;
    		//wire Jump;
			wire [1:0] JumpOP;
			// end input section //


	// Forwarding Unit part
	/*please declare the wire used for forwarding unit here*/
	/*-----------------------------------------------------*/	
      		wire enF1;
      		wire enF2;
     		wire sF1;
     		wire sF2;

			//input section //
			//wire [4:0] EX_Rs;
    		//wire [4:0] EX_Rt;
    		//wire M_RegWrite;
    		//wire [4:0] M_WR_out;
    		//wire WB_RegWrite;
    		//wire [4:0] WB_WR_out;

	
	/*-----------------------------------------------------*/
	wire [data_size-1:0] sF1_data;
	wire [data_size-1:0] sF2_data;
	wire [data_size-1:0] enF1_data;
	wire [data_size-1:0] enF2_data;
	
	// ALU part
	wire [data_size-1:0] scr2;	
	wire [data_size-1:0] EX_ALU_result;
	wire EX_Zero;

			//input section//
			//wire [3:0] ALUOp;
			wire [data_size-1:0] scr1;
			//wire [data_size-1:0] scr2;
			//wire [4:0] shamt;



	
	// PCplus4 adder used for Jal
	/*please declare the wire used for PCplus4 Adder (at EX stage)*/
	/*-----------------------------------------------------*/
	
			//wire [16-1:0] PC_add4;
	
	
	/*-----------------------------------------------------*/
	// EX_M
	/*please declare the wire used for EX_M pipe*/
	/*Tip: There are control signal part and data part*/
	/*-----------------------------------------------------*/
	
	// WB
	  		wire M_MemtoReg;	
	  		wire M_RegWrite;	
	// M	
	 		wire M_MemWrite;	
	 		wire M_Jal;		
	// pipe		  
			wire [data_size-1:0] M_ALU_result;
			wire [data_size-1:0] M_Rt_data;
			wire [pc_size-1:0] M_PCplus8;
	 		wire [4:0] M_WR_out;
	

			// input section //
			// WB		  
			//wire EX_MemtoReg;
    		//wire EX_RegWrite;
    		// M
   			//wire EX_MemWrite;
			//wire EX_Jal;
			// pipe		  
			//wire [data_size-1:0] EX_ALU_result;
    		//wire [data_size-1:0] EX_Rt_data;
    		wire [pc_size-1:0] EX_PCplus8;

    		//wire [4:0] EX_WR_out;


			// end input section //
	
	
	/*-----------------------------------------------------*/
	// M Jal part
	wire [data_size-1:0] M_WD_out;
	
	// M_WB
	/*please declare the wire used for M_WB pipe*/
	/*Tip: There are control signal part and data part*/
	/*-----------------------------------------------------*/
	
	// WB
	  		wire WB_MemtoReg;
	  		wire WB_RegWrite;
	// pipe
      		wire [data_size-1:0] WB_DM_Read_Data;
      		wire [data_size-1:0] WB_WD_out;
      		wire [4:0] WB_WR_out;

			wire [data_size-1:0] M_PCplus8_32bit;
			assign M_PCplus8_32bit =  32'h0000 || M_PCplus8;

			// input section //

			// WB
    		//wire M_MemtoReg;	
    		//wire M_RegWrite;	
			// pipe
    		wire [data_size-1:0] M_DM_Read_Data;
    		//wire [data_size-1:0] M_WD_out;
    		//wire [4:0] M_WR_out;

			// end input section //  
	
	
	/*-----------------------------------------------------*/
	
	// WD or DM Read data mux out
	wire [data_size-1:0] WB_Final_WD_out;
	
	
	// Wire Connect Part
	/* -------------------------------------------------------------------------------------------*/ 
	/* Below here, you are asked to complete the wire connection with the wire you declared above.*/
	/* Determine which wire should be filled in the "please fill here" area. ---------------------*/
	/* -------------------------------------------------------------------------------------------*/
	
		// PC
	assign IM_Address = PCout[pc_size-1:2];	
	
		// Controller
	assign opcode		 = Instruction[31:26];/*please fill here*/
	assign funct		 = Instruction[5:0];/*please fill here*/
	
		// Registers
	assign Rd			 = Instruction[15:11];/*please fill here*/
	assign Rs			 = Instruction[25:21];/*please fill here*/
	assign Rt			 = Instruction[20:16];/*please fill here*/
	
		// sign_extend
	assign imm			 = Instruction[15:0];/*please fill here*/
	
		// shamt to ID_EX pipe
	assign shamt		 = Instruction[10:6];/*please fill here*/
	
		//Jump Part
	assign JumpAddr		 = {Instruction[15:0],2'b0};/*please fill here*/
	
		// Data Memory
	assign DM_Address	 = M_ALU_result[17:2];/*please fill here*/
	assign DM_enable	 = MemWrite;/*please fill here*/
	assign DM_Write_Data = Rt_data;/*please fill here*/
	
	
	
	// IF
	/*-----------------------------------------------------------*/	
	// PC
	PC PC1 ( 
	.clk(clk), 
	.rst(rst),
	.PCWrite(PCWrite),
	.PCin(PCin), 
	.PCout(PCout)
	);
	
	ADD#(pc_size) ADD_Plus4 ( 
	.A(PCout),
	.B(18'd4),
	.Cout(PC_add4)
	);
	
	// IF_ID pipe
	/*-----------------------------------------------------------*/	
	IF_ID IF_ID1 ( 
	.clk(clk),
	.rst(rst),
	// input
	.IF_IDWrite(IF_IDWrite/*please fill here*/),
	.IF_Flush(IF_Flush/*please fill here*/),
	.IF_PC(PC_add4),
	.IF_ir(Instruction),
	// output
	.ID_PC(ID_PC),
	.ID_ir(ID_ir)
	);
	
	// ID
	/*-----------------------------------------------------------*/	
	// Hazard Detection Unit
	HDU HDU1 ( 
	// input
	.ID_Rs(ID_Rs/*please fill here*/),
	.ID_Rt(ID_Rt/*please fill here*/),
	.EX_WR_out(EX_WR_out/*please fill here*/),
	.EX_MemtoReg(EX_MemtoReg/*please fill here*/),
	.EX_JumpOP(EX_JumpOP),
	// output
	.PCWrite(PCWrite/*please fill here*/),			 
	.IF_IDWrite(IF_IDWrite/*please fill here*/),
	.IF_Flush(IF_Flush/*please fill here*/),
	.ID_Flush(ID_Flush/*please fill here*/),
	.Branch_Flush(Branch_Flush),
	.Load_wait(Load_wait)
	);
	
	// Controller
	Controller Controller1 ( 
	.opcode(opcode),
	.funct(funct),
	.Reg_imm(Reg_imm),
	.Jump(Jump),
	.Branch(Branch),
	.Jal(Jal),
	.Jr(Jr),
	.MemtoReg(MemtoReg),
	.ALUOp(ALUOp),
	.RegWrite(RegWrite),
	.MemWrite(MemWrite)
	);
	
	// Registers
	Registers Registers1 ( 
	.clk(clk), 
	.rst(rst),
	.Read_addr_1(Rs),
	.Read_addr_2(Rt),
	.Read_data_1(Rs_data),
	.Read_data_2(Rt_data),
	.RegWrite(RegWrite/*please fill here*/),
	.Write_addr(WB_WR_out/*please fill here*/),
	.Write_data(M_WD_out/*please fill here*/)
	);
	
	// sign_extend	
	sign_extend sign_extend1 ( 
	.in(imm),
	.out(se_imm)
	);
	
	// ID Mux part
	// Mux - select Rd or Rt
	Mux2to1#(5) Rd_Rt ( 
	.I0(Rt/*please fill here*/),
	.I1(Rd/*please fill here*/),
	.S(Reg_imm),
	.out(Rd_Rt_out)
	);
	
	// Mux - select $ra(5'd31, of jal instrction) or Rd_Rt mux out
	Mux2to1#(5) WR ( 
	.I0(Rd_Rt_out),
	.I1(5'd31),
	.S(RegWrite/*please fill here*/),
	.out(WR_out)
	);
	
	// ID_EX pipe
	/*-----------------------------------------------------------*/	
	ID_EX ID_EX1 ( 
	.clk(clk), 
	.rst(rst),
    // input 
	.ID_Flush(ID_Flush/*please fill here*/),
	// WB
	.ID_MemtoReg(ID_MemtoReg/*please fill here*/),
	.ID_RegWrite(ID_RegWrite/*please fill here*/),
	// M
	.ID_MemWrite(ID_MemtoReg/*please fill here*/),
	.ID_Jal(ID_Jal/*please fill here*/),
	// EX
	.ID_Reg_imm(ID_Reg_imm/*please fill here*/),
	.ID_Jump(ID_Jump/*please fill here*/),
	.ID_Branch(ID_Branch/*please fill here*/),
	.ID_Jr(ID_Jr/*please fill here*/),			   
	// pipe
	.ID_PC(ID_PC/*please fill here*/),
	.ID_ALUOp(ID_ALUOp/*please fill here*/),
	.ID_shamt(ID_shamt/*please fill here*/),
	.ID_Rs_data(ID_Rs_data/*please fill here*/),
	.ID_Rt_data(ID_Rt_data/*please fill here*/),
	.ID_se_imm(ID_se_imm/*please fill here*/),
	.ID_WR_out(ID_WR_out/*please fill here*/),
	.ID_Rs(Rs),
	.ID_Rt(Rt),
	// output
	// WB
	.EX_MemtoReg(EX_MemtoReg/*please fill here*/),
	.EX_RegWrite(EX_RegWrite/*please fill here*/),
	// M
	.EX_MemWrite(EX_MemWrite/*please fill here*/),
	.EX_Jal(EX_Jal/*please fill here*/),
	// EX
	.EX_Reg_imm(EX_Reg_imm/*please fill here*/),
	.EX_Jump(EX_Jump/*please fill here*/),
	.EX_Branch(EX_Branch/*please fill here*/),
	.EX_Jr(EX_Jr/*please fill here*/),
	// pipe
	.EX_PC(EX_PC/*please fill here*/),
	.EX_ALUOp(EX_ALUOp/*please fill here*/),
	.EX_shamt(EX_shamt/*please fill here*/),
	.EX_Rs_data(EX_Rs_data/*please fill here*/),
	.EX_Rt_data(EX_Rt_data/*please fill here*/),
	.EX_se_imm(EX_se_imm/*please fill here*/),
	.EX_WR_out(EX_WR_out/*please fill here*/),
	.EX_Rs(EX_Rs/*please fill here*/),
	.EX_Rt(EX_Rt/*please fill here*/)		   			   
	);
	// EX
	/*-----------------------------------------------------------*/	
	// Jump Part
	// Adder - Branch address adder
	ADD#(pc_size) ADD_Branch ( 
	.A(EX_PC/*please fill here*/),
	.B(se_imm[17:0]/*please fill here*/),
	.Cout(BranchAddr)
	);
	
	// Mux - select the next PC source (PC+4, or Branch addr, or jr's $Rs, or JumpAddr)
	Mux4to1#(pc_size) PC_Mux (
	.I0(PC_add4),
	.I1(BranchAddr),
	.I2(enF1_data[17:0]/*please fill here*/), //How could $Rs fit in 18-bit PC ? same as line 629
	.I3(JumpAddr),
	.S(EX_JumpOP),
	.out(PCin)
	);
	
	//Jump control
	Jump_Ctrl Jump_Ctrl1 (
	.Branch(Branch/*please fill here*/),
    .Zero(Zero/*please fill here*/),
    .Jr(Jr/*please fill here*/),
    .Jump(Jump/*please fill here*/),
    .JumpOP(EX_JumpOP)
	);
	
	// Forwarding Unit part
	FU FU1 ( 
	// input 
	.EX_Rs(EX_Rs/*please fill here*/),
    .EX_Rt(EX_Rt/*please fill here*/),
	.M_RegWrite(M_RegWrite/*please fill here*/),
	.M_WR_out(M_WR_out/*please fill here*/),
	.WB_RegWrite(WB_RegWrite/*please fill here*/),
	.WB_WR_out(WB_WR_out/*please fill here*/),
	// output
	.enF1(enF1/*please fill here*/),
	.enF2(enF2/*please fill here*/),
	.sF1(sF1/*please fill here*/),
	.sF2(sF2/*please fill here*/)	
	);
	
	// Mux - select forward data from M or WB (the Rs part)
	Mux2to1#(data_size) sF1_Mux ( 
	.I0(EX_ALU_result/*please fill here*/),
	.I1(M_ALU_result/*please fill here*/),
	.S(sF1/*please fill here*/),
	.out(sF1_data)
	);
	
	// Mux - select forward data from M or WB (the Rt part)
	Mux2to1#(data_size) sF2_Mux ( 
	.I0(EX_ALU_result/*please fill here*/),
	.I1(M_ALU_result/*please fill here*/),
	.S(sF2/*please fill here*/),
	.out(sF2_data)
	);
	
	// Mux - select origin Rs or the forward data (the Rs part)
	Mux2to1#(data_size) enF1_Mux ( 
	.I0(EX_Rs_data/*please fill here*/),
	.I1(sF1_data),
	.S(enF1/*please fill here*/),
	.out(enF1_data)
	);
	
	// Mux - select origin Rt or the forward data (the Rt part)
	Mux2to1#(data_size) enF2_Mux ( 
	.I0(EX_Rt_data/*please fill here*/),
	.I1(sF2_data),
	.S(enF2/*please fill here*/),
	.out(enF2_data)
	);
	
	// ALU part
	// Mux - select Rt or imm (the Rt part)
	Mux2to1#(data_size) Rt_imm (
	.I0(enF2_data),
	.I1(se_imm/*please fill here*/),
	.S(Reg_imm/*please fill here*/),
	.out(scr2)
	);	
	
	ALU ALU1 ( 
	.ALUOp(ALUOp/*please fill here*/),
	.scr1(scr1/*please fill here*/),
	.scr2(scr2),
	.shamt(shamt/*please fill here*/),
	.ALU_result(EX_ALU_result),
	.Zero(EX_Zero)
	);
	
	// PCplus4 used for Jal
	ADD#(pc_size) ADD_Plus4_2 ( 
	.A(EX_PC[17:0]/*please fill here*/),
	.B(18'd4),
	.Cout(EX_PCplus8/*please fill here*/)
	);
	
	// EX_M pipe
	/*-----------------------------------------------------------*/	
	EX_M EX_M1 ( 
	.clk(clk),
	.rst(rst),
	// input 
	// WB
	.EX_MemtoReg(EX_MemtoReg/*please fill here*/),
	.EX_RegWrite(EX_RegWrite/*please fill here*/),
	// M
	.EX_MemWrite(EX_MemtoReg/*please fill here*/),
	.EX_Jal(EX_Jal/*please fill here*/),
	// pipe
	.EX_ALU_result(EX_ALU_result/*please fill here*/),
	.EX_Rt_data(EX_Rt_data/*please fill here*/),
	.EX_PCplus8(EX_PCplus8/*please fill here*/),
	.EX_WR_out(EX_WR_out/*please fill here*/),
	// output
	// WB
	.M_MemtoReg(M_MemtoReg/*please fill here*/),
	.M_RegWrite(M_RegWrite/*please fill here*/),
	// M
	.M_MemWrite(M_MemWrite/*please fill here*/),
	.M_Jal(M_Jal/*please fill here*/),
	// pipe
	.M_ALU_result(M_ALU_result/*please fill here*/),
	.M_Rt_data(M_Rt_data/*please fill here*/),
	.M_PCplus8(M_PCplus8/*please fill here*/),
	.M_WR_out(M_WR_out/*please fill here*/)			  		  			  
	);
	
	// M
	/*-----------------------------------------------------------*/	
	// M Jal part
	// Mux - select Jal or ALU result
	Mux2to1#(data_size) Jal_RD_Select (
	.I0(M_ALU_result/*please fill here*/),
	.I1(M_PCplus8_32bit/*please fill here*/),  //Same problem as line 500
	.S(M_Jal/*please fill here*/),
	.out(M_WD_out)
	);	
	
	// M_WB pipe
	/*-----------------------------------------------------------*/	
	M_WB M_WB1 ( 
	.clk(clk),
    .rst(rst),
	// input 
	// WB
	.M_MemtoReg(M_MemtoReg/*please fill here*/),
	.M_RegWrite(M_RegWrite/*please fill here*/),
	// pipe
	.M_DM_Read_Data(M_DM_Read_Data/*please fill here*/),
	.M_WD_out(M_WD_out/*please fill here*/),
	.M_WR_out(M_WR_out/*please fill here*/),
	// output
	// WB
	.WB_MemtoReg(WB_MemtoReg/*please fill here*/),
	.WB_RegWrite(WB_RegWrite/*please fill here*/),
	// pipe
	.WB_DM_Read_Data(WB_DM_Read_Data/*please fill here*/),
	.WB_WD_out(WB_WD_out/*please fill here*/),
    .WB_WR_out(WB_WR_out/*please fill here*/)
	);
	
	// WB
	/*-----------------------------------------------------------*/	
	// Mux - select the WD or DM Read data
	Mux2to1#(data_size) DM_RD_Select (
	.I0(WB_WD_out/*please fill here*/),
	.I1(WB_DM_Read_Data/*please fill here*/),
	.S(WB_MemtoReg/*please fill here*/),
	.out(WB_Final_WD_out)
	);
	
endmodule


























