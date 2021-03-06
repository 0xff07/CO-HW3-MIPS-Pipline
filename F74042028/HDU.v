// Hazard Detection Unit

module HDU ( // input
			 ID_Rs,
             ID_Rt,
			 EX_WR_out,
			 EX_MemtoReg,
			 EX_JumpOP,
			 // output
			 PCWrite,			 
			 IF_IDWrite,
			 IF_Flush,
			 ID_Flush,
			 Branch_Flush,
			 Load_wait
			 );
	
	parameter bit_size = 32;
	
	input [4:0] ID_Rs;
	input [4:0] ID_Rt;
	input [4:0] EX_WR_out;
	input EX_MemtoReg;
	input [1:0] EX_JumpOP;
	
	output PCWrite;
	output IF_IDWrite;
	output IF_Flush;
	output ID_Flush;
	output Branch_Flush;
	output Load_wait;
	reg PCWrite;
	reg IF_IDWrite;
	reg IF_Flush;
	reg ID_Flush;
	reg Branch_Flush;
	reg Load_wait;
	
	always @(*) begin
		//default signal value
		PCWrite		 = 1;
		IF_IDWrite	 = 1;
		IF_Flush	 = 0;
		ID_Flush	 = 0;
		Branch_Flush = 0;
		Load_wait	 = 0;
		
		// Branch
		/*if ( write condition - Branch occurs ) begin*/
			/* write the signal change */
			/* ----------------------- */
			
			if(EX_JumpOP == 1) begin
			
			  IF_IDWrite = 0; //Since no branch prediction, flush 3 cycle.
			  IF_Flush = 1;
			  ID_Flush = 1;
			end
			
			
			/* ----------------------- */
		//end
		
		// Load
		/*if ( write condition - Load word data hazard occurs ) begin*/
			/* write the signal change */
			/* ----------------------- */
			if(EX_MemtoReg == 1)
			begin
			  if(ID_Rs == EX_WR_out || ID_Rt == EX_WR_out)
			  begin
			    PCWrite = 0; //don't write PC
			    IF_IDWrite = 0; //Halt data from IF_ID register//
			    ID_Flush = 1; //bubble, I.E. set all control signal to 0//
				//How to determine if it's a R-Type instruction ?//
			  end
			end
			
			/* ----------------------- */
		//end
	end
	
endmodule