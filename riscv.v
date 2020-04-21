//			4 STAGED PIPELINED RISC-V WITH DATA HAZARD CONTROL VERILOG IMPLEMENTATION
//				Group Members: Muhammad Hamza (173088), Abdul Hadi Zahid (174442)

/*Instruction Memory is loaded from a file named iMem.txt at line 122 while dMemory is loaded from a file named dMem.txt at line 538.
Both of these files with example code are attached with the verilog file.
By default both imem.txt and dmem.txt file located at E:\ location*/

/* Location of reg file, imem file and dmem file in below code
[31:0]registers[0:31] located at riscv.registerMem.registers
imem located at riscv.iMem.ram
dmem located at riscv.dMem.ram*/

module riscv( input clk, input async_reset);		//top level module
	
	wire[31:0] aluOut, aluOutAP, aluOutAAP, ImmData, ImmDataAP, A, AAP, A_out, B, BAP, B_out, BAAP, ALUB, dmemOut, dmemOutAP, dataW, imem, imemAP;
	wire storeEn, storeEnC, storeEnAP, storeEnAAP, loadEn, loadEnC, loadEnAP, loadEnAAP, loadEnAAAP, regWrite, regWriteC, regWriteAP, regWriteAAP, regWriteAAAP, ImmSelect, memRead, dataBSelect, dataBSelectC, dataBSelectAP, SubEn, SubEnC, SubEnAP, pcHazard, controlHazardMuxSelect;
	wire[6:0] pcCount;		
	wire[4:0] controlOp, addrWAP, addrWAAP, addrWAAAP, addrRs1AP, addrRs2AP;
	wire[3:0] AluOp, AluOpAP;
	wire[2:0] loadSel, loadSelC, loadSelAP, loadSelAAP, loadSelAAAP;
	wire[1:0] forwardA, forwardB;
		
	pcCounter pc(pcCount, clk, async_reset, pcHazard, pcCount);
	iMem i(pcCount, clk, imem);
	pipelineIF IF(imem, clk, async_reset, pcHazard, imemAP);
	
	hazardDetection hazard(loadEnAP, addrWAP, {imemAP[19:15]} ,{imemAP[24:20]}, pcHazard, controlHazardMuxSelect);
	registerMem regMem({imemAP[19:15]} ,{imemAP[24:20]} ,addrWAAAP ,clk, regWriteAAAP, dataW, A, B);
	controlLogic control({imemAP[6:0]}, controlOp ,storeEn, loadEn, regWrite, dataBSelect, ImmSelect, memRead);
	functionLogic funct({imemAP[14:12]}, {imemAP[31:25]}, controlOp, SubEn, AluOp, loadSel);
	ImmGenerator immGen({imemAP[31:7]}, storeEn, ImmSelect, ImmData);
	controlHazardMux hazardMux(controlHazardMuxSelect, SubEn, storeEn, loadEn, regWrite, dataBSelect, loadSel, SubEnC, storeEnC, loadEnC, regWriteC, dataBSelectC, loadSelC);
	pipelineID ID(A, ImmData, B, AluOp, SubEnC, storeEnC, loadEnC, regWriteC, dataBSelectC, {imemAP[11:7]}, {imemAP[19:15]} ,{imemAP[24:20]}, loadSelC, clk, async_reset, AAP, ImmDataAP, BAP, AluOpAP, SubEnAP, storeEnAP, loadEnAP, regWriteAP, dataBSelectAP, addrWAP, addrRs1AP, addrRs2AP, loadSelAP);
	
	dataForward forward(addrRs1AP, addrRs2AP, addrWAAP, addrWAAAP, regWriteAAP, regWriteAAAP, forwardA, forwardB);
	forwardSelect fwdSel(AAP, BAP, dataW, aluOutAP, forwardA, forwardB, A_out, B_out);
	ALUSrc src(B_out, ImmDataAP, dataBSelectAP, ALUB);
	ALU alu(A_out, ALUB , AluOpAP , SubEnAP , aluOut);
	pipelineIE IE(aluOut, B_out, loadEnAP, regWriteAP, storeEnAP, addrWAP, loadSelAP, clk, async_reset, aluOutAP, BAAP, loadEnAAP, regWriteAAP, storeEnAAP, addrWAAP, loadSelAAP);
	
	dMem d(BAAP, aluOutAP[6:0], storeEnAAP, clk, dmemOut);
	pipelineMem Mem(aluOutAP, dmemOut, loadEnAAP, regWriteAAP, addrWAAP, loadSelAAP, clk, async_reset, aluOutAAP, dmemOutAP, loadEnAAAP, regWriteAAAP, addrWAAAP, loadSelAAAP);
	
	aluDataSelect dataMux(aluOutAAP, dmemOutAP, loadEnAAAP, loadSelAAAP, dataW);
		
endmodule

module riscvtb(); //test bench

		reg clk;
		reg async_reset;
				
		initial 
		begin
		
		//example for clock signal, 
		#0 async_reset = 0; clk=0; #1 async_reset=1;		//async must be given first time
		#1 clk = ~clk ;#1 clk = ~clk;
		#1 clk = ~clk ;#1 clk = ~clk;
		#1 clk = ~clk ;#1 clk = ~clk;
		#1 clk = ~clk ;#1 clk = ~clk;
		#1 clk = ~clk ;#1 clk = ~clk;
		#1 clk = ~clk ;#1 clk = ~clk;
		#1 clk = ~clk ;#1 clk = ~clk;
		#1 clk = ~clk ;#1 clk = ~clk;
		#1 clk = ~clk ;#1 clk = ~clk;
		#1 clk = ~clk ;#1 clk = ~clk;
		#1 clk = ~clk ;#1 clk = ~clk;
		#1 clk = ~clk ;#1 clk = ~clk;
		#1 clk = ~clk ;#1 clk = ~clk;
		#1 clk = ~clk ;#1 clk = ~clk;
		#1 clk = ~clk ;#1 clk = ~clk;
		#1 clk = ~clk ;#1 clk = ~clk;
		#1 clk = ~clk ;#1 clk = ~clk;
		#1 clk = ~clk ;#1 clk = ~clk;
		#1 clk = ~clk ;#1 clk = ~clk;
		#1 clk = ~clk ;#1 clk = ~clk;
		#1 clk = ~clk ;#1 clk = ~clk;
		#1 clk = ~clk ;#1 clk = ~clk;
		#1 clk = ~clk ;#1 clk = ~clk;
		#1 clk = ~clk ;#1 clk = ~clk;
		#1 clk = ~clk ;#1 clk = ~clk;
		#1 clk = ~clk ;#1 clk = ~clk;
		#1 clk = ~clk ;#1 clk = ~clk;
		#1 clk = ~clk ;#1 clk = ~clk;
		#1 clk = ~clk ;#1 clk = ~clk;
		#1 clk = ~clk ;#1 clk = ~clk;
		#1 clk = ~clk ;#1 clk = ~clk;
		#1 clk = ~clk ;#1 clk = ~clk;
		
		end
		
		riscv tb(clk,async_reset);

endmodule

module pcCounter(
						input[6:0] prevPcCount, input clk, async_reset, pcHazard,
						output reg [6:0] nextPcCount);

	
	always@(posedge clk or negedge async_reset)
	begin
		if(async_reset == 1'b0)
			nextPcCount <= 7'b1111111;
		else if (pcHazard == 1'b1)
			nextPcCount <= prevPcCount ;
		else
			nextPcCount <= prevPcCount + 7'b0000001;
	end
	
endmodule

module iMem (
					input [6:0] addrRam, input clk,
					output reg [31:0] memOut);

	reg [31:0] ram[0:127];
					
	initial
	begin
		$readmemh("E:\imem.txt", ram);
	end
	
	always @ (*)
	begin
		memOut <= ram[addrRam];
	end 
	
endmodule

module pipelineIF(
						input[31:0] imem, input clk, async_reset, pcHazard,
						output reg [31:0] imemAP);
	
	always@(posedge clk or negedge async_reset)
	begin
	if(async_reset == 1'b0)
			imemAP <= 32'h00000000;
	else if (pcHazard ==1'b0)
			imemAP <= imem;
	else
			imemAP <=imemAP;
	end
						
endmodule

module hazardDetection(
								input loadEnAP, input[4:0]addrWAP, addrRs1, addrRs2,
								output reg pcHazard, controlHazardMuxSelect);

	always@(*)
	begin
	if(loadEnAP == 1'b1 && (addrWAP == addrRs1 || addrWAP == addrRs2))
		begin
		pcHazard = 1'b1;
		controlHazardMuxSelect = 1'b1;
		end
	else
		begin
		pcHazard = 1'b0;
		controlHazardMuxSelect = 1'b0;
		end
	end
								
endmodule

module registerMem(
						 input[4:0] addrA, addrB, addrW,
						 input clk, writeEn, input[31:0] dataW,
						 output[31:0] A, B);
						 
	
	reg[31:0] registers[31:0];
	assign A = registers[addrA];
	assign B = registers[addrB];

	initial
		registers[0] = 32'h00000000;
	always @(negedge clk)
	begin
		if(writeEn == 1 && addrW != 5'b00000)
			registers[addrW] <= dataW;
end			
			 
endmodule

module ImmGenerator(
							input[24:0] Imm25bit, input storeEn, ImmSelect,
							output reg [31:0] ImmData);

	
	reg[11:0] temp;

	always@(*)
	begin
	if(storeEn==0)
		temp = {Imm25bit[24:18],Imm25bit[17:13]};
	else
		temp = {Imm25bit[24:18],Imm25bit[4:0]};
	if(ImmSelect==0)
		ImmData = {{20{1'b0}},temp[11:0]};
	else
	begin
		if({temp[11]}==1'b0)
			ImmData = {{20{1'b0}},temp[11:0]};
		else
			ImmData = {{20{1'b1}},temp[11:0]};
	end
end
endmodule


module controlLogic(
							input[6:0] opCode,
							output reg [4:0] controlOp,
							output reg storeEn, loadEn, regWrite, dataBSelect, ImmSelect, memRead);
							
	always@(*)
	begin
	if(opCode == 7'b0010011)		//Type I
	begin
		storeEn = 1'b0;
		regWrite = 1'b1;
		dataBSelect = 1'b1;
		ImmSelect = 1'b1;
		controlOp = 5'b00001;
		loadEn = 1'b0;
		memRead = 1'b0;
	end
	else if(opCode == 7'b0100011)		//Type S
	begin
		storeEn = 1'b1;
		dataBSelect = 1'b1;
		controlOp = 5'b00010;
		ImmSelect = 1'b0;
		loadEn = 1'b0;
		memRead = 1'b0;
		regWrite = 1'b0;
	end
	else if(opCode == 7'b0000011)		//Load
	begin
		loadEn = 1'b1;
		memRead = 1'b1;
		regWrite = 1'b1;
		dataBSelect = 1'b1;
		controlOp = 5'b00100;
		storeEn = 1'b0;
		ImmSelect = 1'b0;
	end
	else if(opCode == 7'b0110011)		//Type R
	begin
		loadEn = 1'b0;
		memRead = 1'b0;
		regWrite = 1'b1;
		controlOp = 5'b01000;
		dataBSelect = 1'b0;
		storeEn = 1'b0;
		ImmSelect = 1'b0;
	end
end
endmodule

module functionLogic(
							input[2:0] AluFunct, input[6:0] functCode, input[4:0] controlOp,
							output reg SubEn, output reg [3:0] AluOp, output reg[2:0] loadSel);							
	initial begin
		SubEn = 1'b0;
		loadSel =3'b000;
	end	
	
	always@(*)
	begin
		if ((controlOp == 5'b01000 || controlOp == 5'b00001) && AluFunct == 3'b000 && functCode == 7'b0000000)	//Add (Type R/I)
		begin
			AluOp = 4'b0000;
			SubEn = 1'b0;		
		end
		else if (AluFunct == 3'b000 && functCode == 7'b0100000)	//Sub
		begin
			AluOp = 4'b0000;
			SubEn = 1'b1;		
		end
		else if (controlOp == 5'b00010 || controlOp == 5'b00100)	//Add (Load/Store)
		begin
			AluOp = 4'b0000;
			SubEn = 1'b0;		
			if (AluFunct == 3'b010)	//LW
				loadSel = 3'b000;
			else if (AluFunct == 3'b001)	//LH
				loadSel = 3'b001;	
			else if (AluFunct == 3'b000)	//LB
				loadSel = 3'b010;
			else if (AluFunct == 3'b101)	//LHU
				loadSel = 3'b011;
			else if (AluFunct == 3'b100)	//LBU
				loadSel = 3'b100;
		end
		else if (AluFunct == 3'b110)	//OR
			AluOp = 4'b0001;
		else if (AluFunct == 3'b111)	//AND
			AluOp = 4'b0010;
		else if (AluFunct == 3'b100 && functCode == 7'b0000000)	//XOR
			AluOp = 4'b0011;
		else if (AluFunct == 3'b001 && functCode == 7'b0000000)	//SLL
			AluOp = 4'b0100;
		else if (AluFunct == 3'b101 && functCode == 7'b0000000)	//SRL
			AluOp = 4'b0101;	
		else if (AluFunct == 3'b101 && functCode == 7'b0100000)	//SRA
			AluOp = 4'b0110;
		else if (AluFunct == 3'b010)	//SLT
			AluOp = 4'b0111;
		else if (AluFunct == 3'b011)	//SLTU
			AluOp = 4'b0111;
		
end
endmodule

module controlHazardMux(
								input controlHazardMuxSelect, SubEn, storeEn, loadEn, regWrite, dataBSelect, input[2:0] loadSel,
								output reg SubEnC, storeEnC, loadEnC, regWriteC, dataBSelectC, output reg[2:0] loadSelC);
								
	always@(*)
	begin
		if(controlHazardMuxSelect ==1'b1)
			begin
			SubEnC = 1'b0;
			storeEnC = 1'b0;
			loadEnC = 1'b0;
			regWriteC = 1'b0;
			dataBSelectC = 1'b0;
			loadSelC = 3'b000;
			end
		else
			begin
			SubEnC = SubEn;
			storeEnC = storeEn;
			loadEnC = loadEn;
			regWriteC = regWrite;
			dataBSelectC = dataBSelect;
			loadSelC = loadSel;
			end
	end
endmodule

module pipelineID(
						input[31:0] A, ImmData, B, input[3:0] AluOp, input SubEn, storeEn, loadEn, regWrite, dataBSelect, input[4:0] addrW, addrRs1, addrRs2, input[2:0] loadSel, input clk, async_reset,
					   output reg[31:0] AAP, ImmDataAP, BAP, output reg[3:0] AluOpAP, output reg SubEnAP, storeEnAP, loadEnAP, regWriteAP, dataBSelectAP, output reg[4:0]addrWAP, addrRs1AP, addrRs2AP, output reg[2:0] loadSelAP);
	
	always@(posedge clk or negedge async_reset)
	begin
	if(async_reset == 1'b0)
		begin
			AAP <= 32'h00000000;
			ImmDataAP <= 32'h00000000;
			BAP <= 32'h00000000;
			AluOpAP <= 4'b0000;
			SubEnAP <= 1'b0;
			storeEnAP <= 1'b0;
			loadEnAP <= 1'b0;
			regWriteAP <= 1'b0;
			addrWAP <=5'b00000;
			loadSelAP <=3'b000;
			dataBSelectAP <=1'b0;
			addrRs1AP = 5'b00000;
			addrRs2AP = 5'b00000;
		end
	else
		begin
			AAP <= A;
			ImmDataAP <= ImmData;
			BAP <= B;
			AluOpAP <= AluOp;
			SubEnAP <= SubEn;
			storeEnAP <= storeEn;
			loadEnAP <= loadEn;
			regWriteAP <= regWrite;
			addrWAP <= addrW;
			loadSelAP <= loadSel;
			dataBSelectAP <=dataBSelect;
			addrRs1AP = addrRs1;
			addrRs2AP = addrRs2;
		end
	end
endmodule

module dataForward(
						input[4:0] addrRs1, addrRs2, addrWAAP, addrWAAAP, input regWriteAAP, regWriteAAAP,
							output reg[1:0] forwardA, forwardB);
		
	always@(*)
	begin
		if(regWriteAAP == 1'b1 && addrWAAP != 5'b00000 && addrWAAP == addrRs1)
			forwardA = 2'b10;
		else if(regWriteAAAP == 1'b1 && addrWAAAP != 5'b00000 && addrWAAAP == addrRs1)
			forwardA = 2'b01;
		else
			forwardA = 2'b00;
		if(regWriteAAP == 1'b1 && addrWAAP != 5'b00000 && addrWAAP == addrRs2)	
			forwardB = 2'b10;
		else if(regWriteAAAP == 1'b1 && addrWAAAP != 5'b00000 && addrWAAAP == addrRs2)
			forwardB = 2'b01;
		else
			forwardB = 2'b00;
	end					

endmodule

module forwardSelect(
							input[31:0] AAP, BAP, dataW, aluOutAP, input[1:0] forwardA, forwardB,
							output reg[31:0] A_out, B_out);
	always@(*)
	begin
		if(forwardA == 2'b00)
			A_out = AAP;
		else if(forwardA == 2'b01)
			A_out = dataW;
		else if(forwardA == 2'b10)
			A_out = aluOutAP;
		else
			A_out = AAP;
			
		if(forwardB == 2'b00)
			B_out = BAP;
		else if(forwardB == 2'b01)
			B_out = dataW;
		else if(forwardB == 2'b10)
			B_out = aluOutAP;
		else
			B_out = BAP;
	end

endmodule

module ALUSrc(
					input[31:0] B_out, ImmData, input dataBSelect,
					output reg[31:0] ALUB);

	always@(*)
	begin
		if(dataBSelect==1)
			ALUB = ImmData;
		else
			ALUB = B_out;
	end
					
endmodule

module ALU (
				input[31:0] A, B, input[3:0]AluOp, input SubEn, 
				output reg [31:0] aluOut);
				
	
	always@(*)
	begin
	case(AluOp)
        4'b0000: // ADD
		  begin
		  if(SubEn==0)
				aluOut = A + B;
		  else
				aluOut = A - B;
		  end
		  4'b0001: //  OR
           aluOut = A | B;
        4'b0010: //  AND 
           aluOut = A & B;
        4'b0011: //  XOR 
           aluOut = A ^ B;
		  4'b0100: // SLL
           aluOut = A << B;
        4'b0101: // SRL
           aluOut = A >> B;
        4'b0110: // SRA
           aluOut = {A[7], A[7:1]};
		  4'b0111:// SLT
		  begin
				if($signed(A) < $signed(B))
					aluOut = 32'h00000001 ;
				else
					aluOut = 32'h00000000 ;
		  end
        4'b1000:// SLTU
		  begin
				if(A < B)
					aluOut = 32'h00000001 ;
				else
					aluOut = 32'h00000000 ;
		  end
        4'b1001: // XNOR
           aluOut = ~(A ^ B);
        default:
				aluOut = A + B;
        endcase
    end
endmodule

module pipelineIE(
						input[31:0] aluOut, BAP, input loadEnAP, regWriteAP, storeEnAP, input[4:0] addrWAP, input[2:0] loadSelAP, input clk, async_reset,
						output reg [31:0] aluOutAP, BAAP, output reg loadEnAAP, regWriteAAP, storeEnAAP, output reg[4:0] addrWAAP, output reg[2:0] loadSelAAP);

	
	
	always@(posedge clk or negedge async_reset)
	begin
	if(async_reset == 1'b0)
		begin
			aluOutAP <= 32'h00000000;
			BAAP <= 32'h00000000;
			loadEnAAP <= 1'b0;
			regWriteAAP <= 1'b0;
			storeEnAAP <= 1'b0;
			loadSelAAP <= 3'b000;
			addrWAAP <= 5'b00000;
		end
	else
		begin
			aluOutAP <= aluOut;
			BAAP <= BAP;
			loadEnAAP <= loadEnAP;
			regWriteAAP <= regWriteAP;
			storeEnAAP <= storeEnAP;
			loadSelAAP <= loadSelAP;
			addrWAAP <= addrWAP;
		end
	end
						
endmodule

module dMem (
					input [31:0] dataIn, input [6:0] addrRam, input storeEn, clk,
					output reg [31:0] memOut);
	
	reg [31:0] ram[127:0];
	
	initial
	begin
		$readmemh("E:\dmem.txt", ram);
	end


	always @ (posedge clk)
	begin
		if (storeEn)
			ram[addrRam] <= dataIn;
	end 
	
	always @(*)
	begin
		memOut <= ram[addrRam];
	end
	
endmodule

module pipelineMem(
				input[31:0] aluOutAP, dmemOut, input loadEnAAP, regWriteAAP, input[4:0] addrWAAP, input[2:0] loadSelAAP, input clk, async_reset,
				output reg[31:0] aluOutAAP, dmemOutAP, output reg loadEnAAAP, regWriteAAAP, output reg[4:0] addrWAAAP, output reg[2:0] loadSelAAAP);

	always@(posedge clk or negedge async_reset)
	begin
	if(async_reset == 1'b0)
		begin
			aluOutAAP <= 32'h00000000;
			dmemOutAP <= 32'h00000000;
			loadEnAAAP <= 1'b0;
			regWriteAAAP <= 1'b0;
			addrWAAAP <= 5'b00000;
			loadSelAAAP <= 3'b000;
		end
	else
		begin
			aluOutAAP <= aluOutAP;
			dmemOutAP <= dmemOut;
			loadEnAAAP <= loadEnAAP;
			regWriteAAAP <= regWriteAAP;
			addrWAAAP <= addrWAAP;
			loadSelAAAP <= loadSelAAP;
		end
	end		
endmodule

module aluDataSelect(
							input[31:0] aluOut, input[31:0] dmemOut, input loadEn, input[2:0] loadSel,
							output reg [31:0] dataW);

	always@(*)
	begin
		if(loadEn == 0)
			dataW = aluOut;
		else
			begin
			if(loadSel==3'b000)
				dataW = dmemOut;
			else if (loadSel ==3'b001)
				begin
				if(dmemOut[15] == 1'b0)
					dataW =  {{16{1'b0}},dmemOut[15:0]};
				else
					dataW =  {{16{1'b1}},dmemOut[15:0]};
				end
			else if (loadSel == 3'b010)
				begin
				if(dmemOut[7] == 1'b0)
					dataW =  {{24{1'b0}},dmemOut[7:0]};
				else
					dataW =  {{24{1'b1}},dmemOut[7:0]};
				end
			else if (loadSel == 3'b011)
				dataW =  {{16{1'b0}},dmemOut[15:0]};
			else if (loadSel == 3'b100)
				dataW =  {{24{1'b0}},dmemOut[7:0]};
			else
				dataW = dmemOut;
			end
	end
						
endmodule
