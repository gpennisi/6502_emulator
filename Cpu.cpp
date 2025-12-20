#include "Cpu.h"
#include "Bus.h"
#include <stdexcept>
#include <string>

Cpu::Cpu(Bus& b) : bus(b)
{
	
}


uint8_t Cpu::fetchByte()
{
	const uint8_t fetch = bus.read(PC);
	PC++;
	return fetch;
}
uint8_t Cpu::pull()
{
	S++;
	return bus.read(0x100 + S);
}
uint16_t Cpu::pullWord() {
	uint16_t ll = pull();
	uint16_t hh = pull();
	return (hh << 8) | ll;
}
void Cpu::push(const uint8_t& byte)
{
	bus.write(byte, 0x100 + S);
	S--;
}
void Cpu::pushWord(const uint16_t& word)
{
	push(uint8_t((word & 0xFF00) >> 8));
	push(uint8_t(word));
}


void Cpu::reset()
{
	// reset all registers
	A = 0x0;
	X = 0x0;
	Y = 0x0;
	// stack pointer addresses $0100-$01FF
	S = 0xFF;
	status_reg = 0x24; // *** REVISE
	// read the two 16-bit addresses that return
	// the two bytes that point to the address in memory
	// where the Program Counter starts from
	// FFFC is the starting point by design
	// it's far from the program memory start (0000)
	// and close to the program memory end (FFFF)
	// this area is known as the Reset Vector
	const uint8_t low_byte = bus.read(0xFFFC);
	const uint8_t high_byte = bus.read(0xFFFD);
	PC = (high_byte << 8) | low_byte;
}

void Cpu::cycle()
{
	// read the opcode at PC
	uint8_t opcode = bus.read(PC);
	// increment for next instruction
	PC++;
	uint16_t address;

	// EXEC INSTRUCTION

}
	

// Address Modes
void Cpu::acc() { isAccumulatorMode = true; }
void Cpu::abs()
{
	uint8_t ll = fetchByte();
	uint8_t hh = fetchByte();
	// shift high_byte and OR low_byte with it
	currentAddress = ((hh << 8) | ll);
}
void Cpu::absX() { abs(); currentAddress += X; }
void Cpu::absY() { abs(); currentAddress += Y; }
void Cpu::imm() { currentAddress = fetchByte(); }
void Cpu::impl() {};
void Cpu::rel() { currentAddress = PC + int8_t(fetchByte());}
void Cpu::ind()
{
	uint8_t ll = fetchByte();
	uint8_t hh = fetchByte();

	//LLHH address
	// shift hh by 8 zeros and mask out with ll
	uint16_t indirectAddress = ((uint16_t)hh << 8) | ll;
	// construct indirect address by 8bit lookup shifted and masked by lookup+1
	uint8_t indirect_ll = bus.read(indirectAddress);
	// this is to address the JUMP indirect error bug where 
	// when the high-hendian is a page boundary i.e. $00FF
	uint8_t indirect_hh = bus.read(
		(indirectAddress & 0xFF00) |
		((indirectAddress + 1) & 0x00FF)
	);

	currentAddress = ((uint16_t)indirect_hh << 8) | indirect_ll;

}
void Cpu::Xind()
{
	uint16_t pointer = (fetchByte() + X) & 0xFF;
	uint8_t	ll = bus.read(pointer);
	uint8_t hh = bus.read((pointer + 1) & 0xFF);

	currentAddress = (hh << 8) | ll;
}
void Cpu::indY()
{ 
	uint8_t zeroPageAddress = fetchByte();
	uint8_t ll = bus.read(zeroPageAddress);
	uint8_t hh = bus.read((zeroPageAddress+1) & 0xFF);
	uint16_t indirectAddress = ((uint16_t)hh << 8) | ll;
	uint16_t effectiveAddress = indirectAddress + Y;

	// Check for page boundary crossing 
	if ((indirectAddress & 0xFF00) != (effectiveAddress & 0xFF00)) {
		cycleCounter++;
	}
	currentAddress = indirectAddress + Y;
}
void Cpu::zpg()
{
	// from $00 to $FF. the leading bits are always 00
	// it can only therefore address 2^8=256-bits of mem
	// advantage: read 8 bits instead of 16 to get 8 bits value, one less cycle
	// get the page position
	currentAddress = fetchByte();
}
void Cpu::zpgX() { Cpu::zpg(); currentAddress += X; }
void Cpu::zpgY() { Cpu::zpg(); currentAddress += Y; }


// Operation Codes
void Cpu::ADC()
{
	uint8_t addend = bus.read(currentAddress);
	uint16_t sum = uint16_t(A) + uint16_t(addend) + uint16_t(getFlag(C));
	// check if two addeds have same signand the result is of opposite sign (i.e. - + (-) = +)
	updateFlag(V, ( (A ^ uint8_t(sum)) & ~(A ^ addend)) & 0x80 );
	A = sum & 0xFF;
	updateFlag(N, A & 0x80);
	updateFlag(Z, A == 0);
	updateFlag(C, sum > 0xFF);

}

void Cpu::AND()
{
	A &= bus.read(currentAddress);
	updateFlag(N, A & 0x80);
	updateFlag(Z, A == 0);
}

void Cpu::ASL()
{
	uint8_t value = isAccumulatorMode ? A : bus.read(currentAddress);
	uint8_t shift_value = (uint8_t)(value << 1);
	if(isAccumulatorMode)
	{
		A = shift_value;
		isAccumulatorMode = false;
	}
	else { bus.write(shift_value, currentAddress); }
	
	updateFlag(N, shift_value & 0x80);
	updateFlag(Z, shift_value == 0);
	updateFlag(C, value & 0x80);
}

void Cpu::BCC() { if (!getFlag(C)) { PC = currentAddress; } }
void Cpu::BCS() { if (getFlag(C)) { PC = currentAddress; } }
void Cpu::BEQ() { if (getFlag(Z)) { PC = currentAddress; } }
void Cpu::BIT()
{
	uint8_t value = bus.read(currentAddress);
	updateFlag(N, value & 0x80);
	updateFlag(V, value & 0x40);
	updateFlag(Z, (value & A) == 0);
}
void Cpu::BMI() { if (getFlag(N)) { PC = currentAddress; } }
void Cpu::BNE() { if (!getFlag(Z)) { PC = currentAddress; } }
void Cpu::BPL() { if (!getFlag(N)) { PC = currentAddress; } }

void Cpu::BRK()
{
	pushWord(PC + 2);
	setFlag(I);
	// TO FINISH BRK FLAG
}

void Cpu::JMP() { PC = bus.read(currentAddress); }
void Cpu::JSR()
{
	uint16_t origPC = PC;
	PC = bus.read(currentAddress);
	pushWord(origPC - 1);
}
void Cpu::LDA()
{
	A = bus.read(currentAddress);
	updateFlag(N, A & 0x80);
	updateFlag(Z, A == 0);
}	
void Cpu::LDX()
{
	X = bus.read(currentAddress);
	updateFlag(N, X & 0x80);
	updateFlag(Z, X == 0);
}
void Cpu::LDY()
{
	Y = bus.read(currentAddress);
	updateFlag(N, Y & 0x80);
	updateFlag(Z, Y == 0);
}
void Cpu::NOP() {}
void Cpu::PHA() { push(A); }
void Cpu::PHP() { push(status_reg | 0x20 | 0x10); }
void Cpu::PLA() 
{ 
	A = pull(); 
	updateFlag(N, A & 0x80);
	updateFlag(Z, A == 0);
}
void Cpu::PLP() 
{ 
	status_reg = pull() | (status_reg & 0x20) | (status_reg & 0x10); 
}
void Cpu::ROL()
{
	uint8_t value = isAccumulatorMode ? A : bus.read(currentAddress);
	uint8_t rot_value = (value >> 1) | (getFlag(C) ? 0x80 : 0x00);

	if (isAccumulatorMode)
	{
		A = rot_value;
		isAccumulatorMode = false;
	}
	else { bus.write(rot_value, currentAddress); }
	updateFlag(N, rot_value & 0x80);
	updateFlag(Z, rot_value == 0);
	updateFlag(C, value & 0x01);
}
void Cpu::ROR()
{
	uint8_t value = isAccumulatorMode ? A : bus.read(currentAddress);
	uint8_t rot_value = (value >> 1) | uint8_t(Cpu::getFlag(C) << 7);

	if (isAccumulatorMode)
	{
		A = rot_value;
		isAccumulatorMode = false;
	}
	else { bus.write(rot_value, currentAddress); }
	updateFlag(N, rot_value & 0x80);
	updateFlag(Z, rot_value == 0);
	updateFlag(C, value & 0x80);
}
void Cpu::STA() { bus.write(A, currentAddress); }
void Cpu::STX() { bus.write(X, currentAddress); }
void Cpu::STY() { bus.write(Y, currentAddress); }
void Cpu::TAX()
{
	X = A;
	updateFlag(N, X & 0x80);
	updateFlag(Z, X == 0);
}
void Cpu::TAY()
{
	Y = A;
	updateFlag(N, Y & 0x80);
	updateFlag(Z, Y == 0);
}
void Cpu::TSX() { S = X; }
void Cpu::TXA()
{
	A = X;
	updateFlag(N, X & 0x80);
	updateFlag(Z, X == 0);
}
void Cpu::TXS() { X = S; }
void Cpu::TYA()
{
	A = Y;
	updateFlag(N, Y & 0x80);
	updateFlag(Z, Y == 0);
}


// Penalty Functions
void Cpu::crossBound() 
{
	/*
	// If the high byte changed, we crossed a page boundary
	if ((address & 0xFF00) != (final_address & 0xFF00)) {
		cycleCounter++;
	}
	*/
};
void Cpu::branch() {};
void Cpu::sameBound() {};
void Cpu::np() {};


void Cpu::initInstructions()
{
	// https://www.masswerk.at/6502/6502_instruction_set.html#modes
	//     hex      memonic	  addressing    opc	 cycles  penalty
	/* -------------------------------------------------
	ADC
		Add Memory to Accumulator with Carry

		A + M + C->A, C
		NZCIDV
		+++--+
		addressing		assembler		opc	bytes	cycles
		immediate		ADC #oper		69	2		2
		zeropage		ADC oper		65	2		3
		zeropage, X		ADC oper, X		75	2		4
		absolute		ADC oper		6D	3		4
		absolute, X		ADC oper, X		7D	3		4 *
		absolute, Y		ADC oper, Y		79	3		4 *
		(indirect, X)	ADC(oper, X)	61	2		6
		(indirect), Y	ADC(oper), Y	71	2		5 *
	*/
	lookup[0x69] = { "ADC",  &Cpu::imm,  &Cpu::ADC, 2, &Cpu::np };
	lookup[0x65] = { "ADC",  &Cpu::zpg,  &Cpu::ADC, 3, &Cpu::np };
	lookup[0x75] = { "ADC",  &Cpu::zpgX, &Cpu::ADC, 4, &Cpu::np };
	lookup[0x6D] = { "ADC",  &Cpu::abs,  &Cpu::ADC, 4, &Cpu::np };
	lookup[0x7D] = { "ADC",  &Cpu::absX, &Cpu::ADC, 4, &Cpu::crossBound };
	lookup[0x79] = { "ADC",  &Cpu::absY, &Cpu::ADC, 4, &Cpu::crossBound };
	lookup[0x61] = { "ADC",  &Cpu::Xind, &Cpu::ADC, 6, &Cpu::np };
	lookup[0x71] = { "ADC",  &Cpu::indY, &Cpu::ADC, 5, &Cpu::crossBound };
	/* -------------------------------------------------
	AND
		AND Memory with Accumulator

		A AND M->A
		NZCIDV
		++----
		addressing		assembler		opc	bytes	cycles
		immediate		AND	#oper		29	2		2
		zeropage		AND	oper		25	2		3
		zeropage, X		AND	oper, X		35	2		4
		absolute		AND	oper		2D	3		4
		absolute, X		AND	oper, X		3D	3		4 *
		absolute, Y		AND	oper, Y		39	3		4 *
		(indirect, X)	AND(oper, X)	21	2		6
		(indirect), Y	AND(oper), Y	31	2		5 *

	*/
	lookup[0x29] = { "AND",  &Cpu::imm,  &Cpu::AND, 2, &Cpu::np };
	lookup[0x25] = { "AND",  &Cpu::zpg,  &Cpu::AND, 3, &Cpu::np };
	lookup[0x35] = { "AND",  &Cpu::zpgX, &Cpu::AND, 4, &Cpu::np };
	lookup[0x2D] = { "AND",  &Cpu::abs,  &Cpu::AND, 4, &Cpu::np };
	lookup[0x3D] = { "AND",  &Cpu::absX, &Cpu::AND, 4, &Cpu::crossBound };
	lookup[0x39] = { "AND",  &Cpu::absY, &Cpu::AND, 4, &Cpu::crossBound };
	lookup[0x21] = { "AND",  &Cpu::Xind, &Cpu::AND, 6, &Cpu::np };
	lookup[0x31] = { "AND",  &Cpu::indY, &Cpu::AND, 5, &Cpu::crossBound };
	/* -------------------------------------------------
	ASL
		Shift Left One Bit(Memory or Accumulator)

		C < -[76543210] < -0
		NZCIDV
		+++---
		addressing	assembler		opc	bytes	cycles
		accumulator	ASL A			0A	1		2
		zeropage	ASL oper		06	2		5
		zeropage, X	ASL oper, X		16	2		6
		absolute	ASL oper		0E	3		6
		absolute, X	ASL oper, X		1E	3		7
	*/
	lookup[0x0A] = { "ASL",  &Cpu::acc,  &Cpu::ASL, 2, &Cpu::np };
	lookup[0x06] = { "ASL",  &Cpu::zpg,  &Cpu::ASL, 5, &Cpu::np };
	lookup[0x16] = { "ASL",  &Cpu::zpgX, &Cpu::ASL, 6, &Cpu::np };
	lookup[0x0E] = { "ASL",  &Cpu::abs,  &Cpu::ASL, 6, &Cpu::np };
	lookup[0x1E] = { "ASL",  &Cpu::absX, &Cpu::ASL, 7, &Cpu::np };
	/* -------------------------------------------------
	BCC
		Branch on Carry Clear

		branch on C = 0
		NZCIDV
		------
		addressing	assembler	opc	bytes	cycles
		relative	BCC oper	90	2		2 * *

	*/
	lookup[0x90] = { "BCC",  &Cpu::rel,  &Cpu::BCC, 2, &Cpu::branch};
	/* -------------------------------------------------
	BCS
		Branch on Carry Set

		branch on C = 1
		NZCIDV
		------
		addressing	assembler	opc	bytes	cycles
		relative	BCS oper	B0	2		2 * *
	*/
	lookup[0xB0] = { "BCS",  &Cpu::rel,  &Cpu::BCS, 2, &Cpu::branch };
	/* -------------------------------------------------
	BEQ
		Branch on Result Zero

		branch on Z = 1
		NZCIDV
		------
		addressing	assembler	opc	bytes	cycles
		relative	BEQ oper	F0	2		2 * *
	*/
	lookup[0xF0] = { "BEQ",  &Cpu::rel,  &Cpu::BEQ, 2, &Cpu::branch };
	/* -------------------------------------------------
	BIT
		Test Bits in Memory with Accumulator

		bits 7 and 6 of operand are transfered to bit 7 and 6 of SR(N, V);
		the zero - flag is set according to the result of the operand AND
		the accumulator(set, if the result is zero, unset otherwise).
		This allows a quick check of a few bits at once without affecting
		any of the registers, other than the status register (SR).

		A AND M->Z, M7->N, M6->V
		NZCIDV
		M7+---M6
		addressing	assembler	opc	bytes	cycles
		zeropage	BIT oper	24	2		3
		absolute	BIT oper	2C	3		4
	*/
	lookup[0x24] = { "BIT",  &Cpu::zpg,  &Cpu::BIT, 3, &Cpu::np };
	lookup[0x2C] = { "BIT",  &Cpu::abs,  &Cpu::BIT, 4, &Cpu::np };
	/* -------------------------------------------------
	BMI
		Branch on Result Minus

		branch on N = 1
		NZCIDV
		------
		addressing	assembler	opc	bytes	cycles
		relative	BMI oper	30	2		2 * *
	*/
	lookup[0x30] = { "BMI",  &Cpu::rel,  &Cpu::BMI, 2, &Cpu::branch };
	/* -------------------------------------------------
	BNE
		Branch on Result not Zero

		branch on Z = 0
		NZCIDV
		------
		addressing	assembler	opc	bytes	cycles
		relative	BNE oper	D0	2		2 * *
	*/
	lookup[0xD0] = { "BNE",  &Cpu::rel,  &Cpu::BNE, 2, &Cpu::branch };
	/* -------------------------------------------------
	BPL
		Branch on Result Plus

		branch on N = 0
		NZCIDV
		------
		addressing	assembler	opc	bytes	cycles
		relative	BPL oper	10	2		2 * *
	*/
	lookup[0x10] = { "BPL",  &Cpu::rel,  &Cpu::BPL, 2, &Cpu::branch };
	/* -------------------------------------------------
	BRK
		Force Break

		BRK initiates a software interrupt similar to a hardware
		interrupt(IRQ).The return address pushed to the stack is
		PC + 2, providing an extra byte of spacing for a break mark
		(identifying a reason for the break.)
		The status register will be pushed to the stack with the break
		flag set to 1. However, when retrieved during RTI or by a PLP
		instruction, the break flag will be ignored.
		The interrupt disable flag is not set automatically.
		
		interrupt,
		push PC + 2, push SR
		NZCIDV
		---1--
		addressing	assembler	opc	bytes	cycles
		implied		BRK			00	1		7
	*/
	lookup[0x00] = { "BRK",  &Cpu::impl,  &Cpu::BRK, 7, &Cpu::np };
	/* -------------------------------------------------
	JMP
		Jump to New Location

		operand 1st byte->PCL
		operand 2nd byte->PCH
		NZCIDV
		------
		addressing	assembler	opc	bytes	cycles
		absolute	JMP oper	4C	3		3
		indirect	JMP(oper)	6C	3		5 * **
	*/
	lookup[0x4C] = { "JMP",  &Cpu::abs,  &Cpu::JMP, 3, &Cpu::np };
	lookup[0x6C] = { "JMP",  &Cpu::ind,  &Cpu::JMP, 5, &Cpu::sameBound };
	/* -------------------------------------------------
	JSR
		Jump to New Location Saving Return Address

		push(PC + 2),
		operand 1st byte->PCL
		operand 2nd byte->PCH
		NZCIDV
		------
		addressing	assembler	opc	bytes	cycles
		absolute	JSR oper	20	3		6

	*/
	lookup[0x20] = { "JSR",  &Cpu::abs,  &Cpu::JSR, 6, &Cpu::np };
	/* -------------------------------------------------
	LDA
		Load Accumulator with Memory

		M->A
		NZCIDV
		++----
		addressing		assembler			opc	bytes	cycles
		immediate		LDA #oper			A9	2		2
		zeropage		LDA oper			A5	2		3
		zeropage, X		LDA oper, X			B5	2		4
		absolute		LDA oper			AD	3		4
		absolute, X		LDA oper, X			BD	3		4 *
		absolute, Y		LDA oper, Y			B9	3		4 *
		(indirect, X)	LDA(oper, X)		A1	2		6
		(indirect), Y	LDA(oper), Y		B1	2		5 *
	*/
	lookup[0xA9] = { "LDA",  &Cpu::imm,  &Cpu::LDA, 2, &Cpu::np };
	lookup[0xA5] = { "LDA",  &Cpu::zpg,  &Cpu::LDA, 3, &Cpu::np };
	lookup[0xB5] = { "LDA",  &Cpu::zpgX, &Cpu::LDA, 4, &Cpu::np };
	lookup[0xAD] = { "LDA",  &Cpu::abs,  &Cpu::LDA, 4, &Cpu::np };
	lookup[0xBD] = { "LDA",  &Cpu::absX, &Cpu::LDA, 4, &Cpu::crossBound };
	lookup[0xB9] = { "LDA",  &Cpu::absY, &Cpu::LDA, 4, &Cpu::crossBound };
	lookup[0xA1] = { "LDA",  &Cpu::Xind, &Cpu::LDA, 6, &Cpu::np };
	lookup[0xB1] = { "LDA",  &Cpu::indY, &Cpu::LDA, 5, &Cpu::crossBound };
	/* -------------------------------------------------
	LDX
		Load Index X with Memory

		M->X
		NZCIDV
		++----
		addressing	assembler	opc	bytes	cycles
		immediate	LDX #oper	A2	2		2
		zeropage	LDX oper	A6	2		3
		zeropage, Y	LDX oper, Y	B6	2		4
		absolute	LDX oper	AE	3		4
		absolute, Y	LDX oper, Y	BE	3		4 *
	*/
	lookup[0xA2] = { "LDX",  &Cpu::imm,  &Cpu::LDX, 2, &Cpu::np };
	lookup[0xA6] = { "LDX",  &Cpu::zpg,  &Cpu::LDX, 3, &Cpu::np };
	lookup[0xB6] = { "LDX",  &Cpu::zpgY, &Cpu::LDX, 4, &Cpu::np };
	lookup[0xAE] = { "LDX",  &Cpu::abs,  &Cpu::LDX, 4, &Cpu::np };
	lookup[0xBE] = { "LDX",  &Cpu::absY, &Cpu::LDX, 4, &Cpu::crossBound };
	/* -------------------------------------------------
		LDY
		Load Index Y with Memory

		M->Y
		NZCIDV
		++----
		addressing	assembler	opc	bytes	cycles
		immediate	LDY #oper	A0	2		2
		zeropage	LDY oper	A4	2		3
		zeropage, X	LDY oper, X	B4	2		4
		absolute	LDY oper	AC	3		4
		absolute, X	LDY oper, X	BC	3		4 *
	*/
	lookup[0xA0] = { "LDY",  &Cpu::imm,  &Cpu::LDY, 2, &Cpu::np };
	lookup[0xA4] = { "LDY",  &Cpu::zpg,  &Cpu::LDY, 3, &Cpu::np };
	lookup[0xB4] = { "LDY",  &Cpu::zpgX, &Cpu::LDY, 4, &Cpu::np };
	lookup[0xAC] = { "LDY",  &Cpu::abs,  &Cpu::LDY, 4, &Cpu::np };
	lookup[0xBC] = { "LDY",  &Cpu::absX, &Cpu::LDY, 4, &Cpu::crossBound };
	/* -------------------------------------------------
	NOP
		No Operation

		-- -
		NZCIDV
		------
		addressing	assembler	opc		bytes	cycles
		implied		NOP			EA		1		2
	*/
	lookup[0xEA] = { "NOP",  &Cpu::impl, &Cpu::NOP, 2, &Cpu::np };
	/* -------------------------------------------------
	PHA
		Push Accumulator on Stack

		push A
		NZCIDV
		------
		addressing	assembler	opc	bytes	cycles
		implied		PHA			48	1		3
	*/
	lookup[0x48] = { "PHA",  &Cpu::impl, &Cpu::PHA, 3, &Cpu::np };
	/* -------------------------------------------------
	PHP
		Push Processor Status on Stack

		The status register will be pushed with the break
		flag and bit 5 set to 1.

		push SR
		NZCIDV
		------
		addressing	assembler	opc	bytes	cycles
		implied		PHP			08	1		3
	*/
	lookup[0x08] = { "PHP",  &Cpu::impl, &Cpu::PHP, 3, &Cpu::np };
	/* -------------------------------------------------
	PLA
		Pull Accumulator from Stack

		pull A
		NZCIDV
		++----
		addressing	assembler	opc	bytes	cycles
		implied		PLA			68	1		4
	*/
	lookup[0x68] = { "PLA",  &Cpu::impl, &Cpu::PLA, 4, &Cpu::np };
	/* -------------------------------------------------
	PLP
		Pull Processor Status from Stack

		The status register will be pulled with the break
		flag and bit 5 ignored.

		pull SR
		NZCIDV
		from stack
		addressing	assembler	opc	bytes	cycles
		implied		PLP			28	1		4
	*/
	lookup[0x28] = { "PLP",  &Cpu::impl, &Cpu::PLP, 4, &Cpu::np };
	/* -------------------------------------------------
	ROL
		Rotate One Bit Left(Memory or Accumulator)

		C < -[76543210] < -C
		NZCIDV
		+ ++---
		addressing	assembler	opc	bytes	cycles
		accumulator	ROL A		2A	1		2
		zeropage	ROL oper	26	2		5
		zeropage, X	ROL oper, X	36	2		6
		absolute	ROL oper	2E	3		6
		absolute, X	ROL oper, X	3E	3		7
	*/
	lookup[0x2A] = { "ROL",  &Cpu::acc,  &Cpu::ROL, 2, &Cpu::np };
	lookup[0x26] = { "ROL",  &Cpu::zpg,  &Cpu::ROL, 5, &Cpu::np };
	lookup[0x36] = { "ROL",  &Cpu::zpgX, &Cpu::ROL, 6, &Cpu::np };
	lookup[0x2E] = { "ROL",  &Cpu::abs,  &Cpu::ROL, 6, &Cpu::np };
	lookup[0x3E] = { "ROL",  &Cpu::absX, &Cpu::ROL, 7, &Cpu::np };
	/* -------------------------------------------------
	ROR
		Rotate One Bit Right(Memory or Accumulator)

		C ->[76543210]->C
		NZCIDV
		+ ++---
		addressing	assembler	opc	bytes	cycles
		accumulator	ROR A		6A	1		2
		zeropage	ROR oper	66	2		5
		zeropage, X	ROR oper, X	76	2		6
		absolute	ROR oper	6E	3		6
		absolute, X	ROR oper, X	7E	3		7
	*/
	lookup[0x6A] = { "ROR",  &Cpu::acc,  &Cpu::ROR, 2, &Cpu::np };
	lookup[0x66] = { "ROR",  &Cpu::zpg,  &Cpu::ROR, 5, &Cpu::np };
	lookup[0x76] = { "ROR",  &Cpu::zpgX, &Cpu::ROR, 6, &Cpu::np };
	lookup[0x6E] = { "ROR",  &Cpu::abs,  &Cpu::ROR, 6, &Cpu::np };
	lookup[0x7E] = { "ROR",  &Cpu::absX, &Cpu::ROR, 7, &Cpu::np };
	/* -------------------------------------------------
	STA
		Store Accumulator in Memory

		A->M
		NZCIDV
		------
		addressing		assembler		opc	 bytes	cycles
		zeropage		STA oper		85	 2		3
		zeropage, X		STA oper, X		95	 2		4
		absolute		STA oper		8D	 3		4
		absolute, X		STA oper, X		9D	 3		5
		absolute, Y		STA oper, Y		99	 3		5
		(indirect, X)	STA(oper, X)	81	 2		6
		(indirect), Y	STA(oper), Y	91	 2		6
	*/
	lookup[0x85] = { "STA",  &Cpu::zpg,  &Cpu::STA, 3, &Cpu::np };
	lookup[0x95] = { "STA",  &Cpu::zpgX, &Cpu::STA, 4, &Cpu::np };
	lookup[0x8D] = { "STA",  &Cpu::abs,  &Cpu::STA, 4, &Cpu::np };
	lookup[0x9D] = { "STA",  &Cpu::absX, &Cpu::STA, 5, &Cpu::np };
	lookup[0x99] = { "STA",  &Cpu::absY, &Cpu::STA, 5, &Cpu::np };
	lookup[0x81] = { "STA",  &Cpu::Xind, &Cpu::STA, 6, &Cpu::np };
	lookup[0x91] = { "STA",  &Cpu::indY, &Cpu::STA, 6, &Cpu::np };
	/* -------------------------------------------------
	STX
		Store Index X in Memory

		X->M
		NZCIDV
		------
		addressing	assembler	opc	bytes	cycles
		zeropage	STX oper	86	2		3
		zeropage, Y	STX oper, Y	96	2		4
		absolute	STX oper	8E	3		4
	*/
	lookup[0x86] = { "STX",  &Cpu::zpg,  &Cpu::LDX, 3, &Cpu::np };
	lookup[0x96] = { "STX",  &Cpu::zpgY, &Cpu::LDX, 4, &Cpu::np };
	lookup[0x8E] = { "STX",  &Cpu::abs,  &Cpu::LDX, 4, &Cpu::np };
	/* -------------------------------------------------
	STY
		Sore Index Y in Memory

		Y->M
		NZCIDV
		------
		addressing	assembler	opc	bytes	cycles
		zeropage	STY oper	84	2	3
		zeropage, X	STY oper, X	94	2	4
		absolute	STY oper	8C	3	4
	*/
	lookup[0x84] = { "STY",  &Cpu::zpg,  &Cpu::LDX, 3, &Cpu::np };
	lookup[0x94] = { "STY",  &Cpu::zpgX, &Cpu::LDX, 4, &Cpu::np };
	lookup[0x8C] = { "STY",  &Cpu::abs,  &Cpu::LDX, 4, &Cpu::np };
	/* -------------------------------------------------
	TAX
		Transfer Accumulator to Index X

		A->X
		NZCIDV
		++----
		addressing	assembler	opc	bytes	cycles
		implied		TAX			AA	1		2
	*/
	lookup[0xAA] = { "TAX",  &Cpu::impl, &Cpu::TAX, 2, &Cpu::np };
	/* -------------------------------------------------
	TAY
		Transfer Accumulator to Index Y

		A->Y
		NZCIDV
		++----
		addressing	assembler	opc	bytes	cycles
		implied		TAY			A8	1		2
	*/
	lookup[0xA8] = { "TAY",  &Cpu::impl, &Cpu::TAY, 2, &Cpu::np };
	/* -------------------------------------------------
	TSX
		Transfer Stack Pointer to Index X

		SP->X
		NZCIDV
		++----
		addressing	assembler	opc	bytes	cycles
		implied		TSX			BA	1		2
	*/
	lookup[0x8C] = { "TSX",  &Cpu::impl, &Cpu::TSX, 2, &Cpu::np };
	/* -------------------------------------------------
	TXA
		Transfer Index X to Accumulator

		X->A
		NZCIDV
		++----
		addressing	assembler	opc	bytes	cycles
		implied		TXA			8A	1		2
	*/
	lookup[0x8A] = { "TXA",  &Cpu::impl, &Cpu::TXA, 2, &Cpu::np };
	/* -------------------------------------------------
	TXS
		Transfer Index X to Stack Register

		X->SP
		NZCIDV
		------
		addressing	assembler	opc	bytes	cycles
		implied		TXS			9A	1		2
	*/
	lookup[0x9A] = { "TXS",  &Cpu::impl, &Cpu::TXS, 2, &Cpu::np };
	/* -------------------------------------------------
	TYA
		Transfer Index Y to Accumulator

		Y->A
		NZCIDV
		++----
		addressing	assembler	opc	bytes	cycles
		implied		TYA			98	1		2
	*/
	lookup[0x98] = { "TYA",  &Cpu::impl, &Cpu::TYA, 2, &Cpu::np };

}



void Cpu::execInstruction(Instruction instruction)
{
	(this->*instruction.addr)();
	(this->*instruction.opc)();
}