//Cpu.cpp

#include "Cpu.h"
#include "Bus.h"
#include <stdexcept>
#include <iterator>


// declare lookup
Cpu::Instruction Cpu::lookup[0x100];

// core functions
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
	S = 0xFD;
	SR = 0x24; 
	isAccumulatorMode = false;
	cycleCounter = 7;
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
	// fetch 
	uint8_t opc = fetchByte();
	// decode 
	Instruction inst = lookup[opc];
	// execute 
	execInstruction(inst);
}
void Cpu::execInstruction(Instruction instruction)
{
	if (instruction.inst == "XXX") 
	{
		std::cerr << "[CPU Error] Illegal Instruction not supported " << std::endl;
		exit(1);
	}
	penalty = 0;
	cycleCounter += instruction.cycles;

	(this->*instruction.addr)();
	(this->*instruction.opc)();
	if (instruction.penalty) cycleCounter += penalty;
}
void Cpu::irq() 
{
	if (!getFlag(I)) {
		interrupt(0xFFFE, false);
		cycleCounter += 7;
	}
}
void Cpu::nmi() 
{
	interrupt(0xFFFA, false);
	cycleCounter += 7;
}

std::string Cpu::disassembleInstruction(uint32_t addr)
{
	uint8_t value = bus.read(uint16_t(addr));
	Instruction instruction = lookup[value];

	std::stringstream ss;
	ss << std::hex << std::uppercase << std::setfill('0');

	// i.e. "$C000 : LDA "
	ss << "$" << std::setw(4) << addr << " : " << "[$" << int(value) << "] "
		<< instruction.inst << " ";


	addr++; // next operand


	auto addrMode = instruction.addr;

	if (addrMode == &Cpu::imm)
	{
		uint8_t value = bus.read((uint16_t)addr);
		ss << "#$" << std::setw(2) << (int)value;
	}
	else if (addrMode == &Cpu::zpg)
	{
		uint8_t lo = bus.read((uint16_t)addr);
		ss << "$" << std::setw(2) << (int)lo;
	}
	else if (addrMode == &Cpu::zpgX)
	{
		uint8_t lo = bus.read((uint16_t)addr);
		ss << "$" << std::setw(2) << (int)lo << ",X";
	}
	else if (addrMode == &Cpu::zpgY)
	{
		uint8_t lo = bus.read((uint16_t)addr);
		ss << "$" << std::setw(2) << (int)lo << ",Y";
	}
	else if (addrMode == &Cpu::abs)
	{
		uint8_t lo = bus.read((uint16_t)addr); addr++;
		uint8_t hi = bus.read((uint16_t)addr);
		ss << "$" << std::setw(4) << ((hi << 8) | lo);
	}
	else if (addrMode == &Cpu::absX)
	{
		uint8_t lo = bus.read((uint16_t)addr); addr++;
		uint8_t hi = bus.read((uint16_t)addr);
		ss << "$" << std::setw(4) << ((hi << 8) | lo) << ",X";
	}
	else if (addrMode == &Cpu::absY)
	{
		uint8_t lo = bus.read((uint16_t)addr); addr++;
		uint8_t hi = bus.read((uint16_t)addr);
		ss << "$" << std::setw(4) << ((hi << 8) | lo) << ",Y";
	}
	else if (addrMode == &Cpu::ind)
	{
		uint8_t lo = bus.read((uint16_t)addr); addr++;
		uint8_t hi = bus.read((uint16_t)addr);
		ss << "($" << std::setw(4) << ((hi << 8) | lo) << ")";
	}
	else if (addrMode == &Cpu::Xind)
	{
		uint8_t lo = bus.read((uint16_t)addr);
		ss << "($" << std::setw(2) << (int)lo << ",X)";
	}
	else if (addrMode == &Cpu::indY)
	{
		uint8_t lo = bus.read((uint16_t)addr);
		ss << "($" << std::setw(2) << (int)lo << "),Y";
	}
	else if (addrMode == &Cpu::rel)
	{
		uint8_t value = bus.read((uint16_t)addr);
		// Calculate the absolute address of the branch target
		// addr + 1 (because PC would be at next instruction) + signed offset
		uint16_t target = (uint16_t)(addr + 1 + (int8_t)value);
		ss << "$" << std::setw(2) << (int)value << " [$" << std::setw(4) << target << "]";
	}

	// IMP and ACC modes require no extra printing

	return ss.str();
}


// helper functions
uint16_t Cpu::arithmetic(const uint16_t& value, std::function<uint16_t(uint16_t)> operation)
{
	uint16_t result = operation(value);
	updateFlag(N, uint8_t(result) & 0x80);
	updateFlag(Z, uint8_t(result) == 0);
	return result;
}
void Cpu::compare(uint8_t reg, const uint8_t& value)
{
	// https://www.nesdev.org/obelisk-6502-guide/reference.html#CMP
	updateFlag(N, uint8_t(reg - value) & 0x80);
	updateFlag(Z, reg == value);
	updateFlag(C, reg >= value);
}
void Cpu::load(uint8_t& reg, const uint8_t& value)
{
	reg = value;
	updateFlag(N, reg & 0x80);
	updateFlag(Z, reg == 0);
}
void Cpu::logic(std::function<void(uint8_t&)> operation)
{
	operation(A);
	updateFlag(N, A & 0x80);
	updateFlag(Z, A == 0);
}
uint8_t Cpu::shift(const uint8_t& mask, std::function<uint8_t(uint8_t)> operation)
{
	uint8_t value = isAccumulatorMode ? A : bus.read(currentAddress);
	updateFlag(C, value & mask);

	uint8_t result = (uint8_t)arithmetic(value, [&](uint16_t v) {
		return (uint16_t)operation((uint8_t)v);
		});

	if (isAccumulatorMode) { A = result; isAccumulatorMode = false; }
	else { bus.write(result, currentAddress); }
	return result;
}
void Cpu::absoluteIndexed(uint8_t& reg)
{
	abs();
	const uint16_t finalAddress = currentAddress + reg;
	crossBound(currentAddress, finalAddress);
	currentAddress = finalAddress;
}
void Cpu::interrupt(uint16_t address, bool isSoftware)
{
	pushWord(PC);
	uint8_t statusToPush = SR | 0x20;
	if (isSoftware) {
		statusToPush |= 0x10;
	}
	else {
		statusToPush &= ~0x10;
	}

	push(statusToPush);

	setFlag(I);

	uint8_t ll = bus.read(address);
	uint8_t hh = bus.read(address + 1);
	PC = (hh << 8) | ll;
}

// Penalty Functions
void Cpu::crossBound(const uint16_t& baseAddress, const uint16_t& finalAddress )
{
	// If the high byte changed, we crossed a page boundary
	penalty = (baseAddress & 0xFF00) != (finalAddress & 0xFF00);
};
void Cpu::branch()
{
	// branch taken: +1 cycle
	penalty = 1;

	// page boundary crossed: +1 additional cycle
	if ((PC & 0xFF00) != (currentAddress & 0xFF00)) {
		penalty++;
	}
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
void Cpu::absX() { absoluteIndexed(X); }
void Cpu::absY() { absoluteIndexed(Y); }
void Cpu::imm()
{  
	currentAddress = PC;
	PC++;
}
void Cpu::impl() {};
void Cpu::rel() 
{ 
	// fetch the offset and increment PC first
	int8_t offset = fetchByte();
	// add to the incremented PC
	currentAddress = PC + offset;
	branch();
}
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
	zpg();
	uint8_t ll = bus.read(currentAddress);
	uint8_t hh = bus.read((currentAddress + 1) & 0xFF);

	uint16_t base = ((uint16_t)hh << 8) | ll;
	currentAddress = base + Y;

	crossBound(base, currentAddress);
}
void Cpu::zpg()
{
	// from $00 to $FF. the leading bits are always 00
	// it can only therefore address 2^8=256-bits of mem
	// advantage: read 8 bits instead of 16 to get 8 bits value, one less cycle
	// get the page position
	currentAddress = fetchByte();
}
void Cpu::zpgX() { Cpu::zpg(); currentAddress = (currentAddress + X) & 0xFF; }
void Cpu::zpgY() { Cpu::zpg(); currentAddress = (currentAddress + Y) & 0xFF; }

void Cpu::ADC() 
{ 
	uint8_t addend = bus.read(currentAddress);
	uint16_t result = arithmetic(
		A, 
		[addend = addend, C = getFlag(C)](uint16_t v) {
			return uint16_t(v) + uint16_t(addend) + uint16_t(C);
		});
	updateFlag(V, ((A ^ uint8_t(result)) & ~(A ^ addend)) & 0x80);
	updateFlag(C, result > 0xFF);
	A = (uint8_t)result;
}
void Cpu::AND() 
{ 
	logic([value = bus.read(currentAddress)](uint8_t& A) { A &= value; }); 
}
void Cpu::ASL() { shift(0x80, [](uint8_t v) { return v << 1; }); }

void Cpu::BCC()
{
	if (!getFlag(C)) { PC = currentAddress; }
	else { penalty = 0; }
}
void Cpu::BCS()
{
	if (getFlag(C)) { PC = currentAddress; }
	else { penalty = 0; }
}
void Cpu::BEQ()
{
	if (getFlag(Z)) { PC = currentAddress; }
	else { penalty = 0; }
}
void Cpu::BIT()
{
	uint8_t value = bus.read(currentAddress);
	updateFlag(N, value & 0x80);
	updateFlag(V, value & 0x40);
	updateFlag(Z, (value & A) == 0);
}
void Cpu::BMI()
{
	if (getFlag(N)) { PC = currentAddress; }
	else { penalty = 0; }
}
void Cpu::BNE()
{
	if (!getFlag(Z)) { PC = currentAddress; }
	else { penalty = 0; }
}
void Cpu::BPL()
{
	if (!getFlag(N)) { PC = currentAddress; }
	else { penalty = 0; }
}
void Cpu::BRK() {
	// https://www.nesdev.org/the%20'B'%20flag%20&%20BRK%20instruction.txt
	PC++; // BRK is a 2-byte instruction; skip the padding byte
	interrupt(0xFFFE, true);
}
void Cpu::BVC()
{
	if (!getFlag(V)) { PC = currentAddress; }
	else { penalty = 0; }
}
void Cpu::BVS()
{
	if (getFlag(V)) { PC = currentAddress; }
	else { penalty = 0; }
}

void Cpu::CLC() { clearFlag(C); }
void Cpu::CLD() { clearFlag(D); }
void Cpu::CLI() { clearFlag(I); }
void Cpu::CLV() { clearFlag(V); }
void Cpu::CMP() 
{ 
	compare(A, bus.read(currentAddress)); 
}
void Cpu::CPX() { compare(X, bus.read(currentAddress)); }
void Cpu::CPY() { compare(Y, bus.read(currentAddress)); }

void Cpu::DEC()
{
	bus.write( 
		(uint8_t)arithmetic(bus.read(currentAddress), [](uint16_t v){ return v - 1; }),
		currentAddress
	);
}
void Cpu::DEX() { X = (uint8_t)arithmetic(X, [](uint16_t v) { return v - 1; }); }
void Cpu::DEY() { Y = (uint8_t)arithmetic(Y, [](uint16_t v) { return v - 1; }); }

void Cpu::EOR() { 
	logic([value = bus.read(currentAddress)](uint8_t& A) { A ^= value; }); 
}

void Cpu::INC()
{
	bus.write(
		(uint8_t)arithmetic(bus.read(currentAddress), [](uint16_t v) { return v + 1; }),
		currentAddress
	);
}
void Cpu::INX() { X = (uint8_t)arithmetic(X, [](uint16_t v) { return v + 1; }); }
void Cpu::INY() { Y = (uint8_t)arithmetic(Y, [](uint16_t v) { return v + 1; }); }

void Cpu::JMP() { PC = currentAddress; }
void Cpu::JSR()
{
	pushWord(PC - 1);
	PC = currentAddress;
}

void Cpu::LDA() 
{ 
	load(A, bus.read(currentAddress)); 
}
void Cpu::LDX() 
{ 
	load(X, bus.read(currentAddress)); 
}
void Cpu::LDY() 
{ 
	load(Y, bus.read(currentAddress)); 
}
void Cpu::LSR() { shift(0x01, [](uint8_t v) { return v >> 1; }); }

void Cpu::ORA() 
{ 
	logic([value = bus.read(currentAddress)](uint8_t& A) { A |= value; }); 
}

void Cpu::NOP() {}

void Cpu::PHA() { push(A); }
void Cpu::PHP() { push(SR | 0x20 | 0x10); }
void Cpu::PLA() { load(A, pull()); }
void Cpu::PLP()
{
	uint8_t pulled = pull();
	SR = (pulled & 0xCF) | 0x20;  // Clear bit 4, set bit 5
}

void Cpu::ROL() {
	shift(0x80, [this](uint8_t v) {
		return (v << 1) | (getFlag(C) ? 0x01 : 0x00);
		});
}
void Cpu::ROR() {
	shift(0x01, [this](uint8_t v) {
		return (v >> 1) | (getFlag(C) ? 0x80 : 0x00);
		});
}
void Cpu::RTI() 
{
	// pull ignoring bit 4 and 5 11001111
	SR = pull() & 0xCF;
	/*
	from documentation:
	'The status register is pulled with the break flag
	and bit 5 ignored.Then PC is pulled from the stack.'

	but it seems in reallife bit (0x20) is always 1 and ignored during a pull.
	might need revsion
	*/
	PC = pullWord();
}
void Cpu::RTS()
{
	PC = pullWord();
	PC++;
}

void Cpu::SBC() {
	uint8_t subtraend = bus.read(currentAddress);
	uint16_t result = arithmetic(
		A,
		[subtraend, C = getFlag(C)](uint16_t v) {
			return uint16_t(v) - uint16_t(subtraend) - (C ? 0 : 1);
		});

	updateFlag(V, ((A ^ subtraend) & (A ^ uint8_t(result))) & 0x80);
	updateFlag(C, result < 0x100);
	A = (uint8_t)result;
}
void Cpu::SEC() { setFlag(C); }
void Cpu::SED() { setFlag(D); }
void Cpu::SEI() { setFlag(I); }
void Cpu::STA() { bus.write(A, currentAddress); }
void Cpu::STX() { bus.write(X, currentAddress); }
void Cpu::STY() { bus.write(Y, currentAddress); }

void Cpu::TAX() { load(X, A); }
void Cpu::TAY() { load(Y, A); }
void Cpu::TSX() { load(X, S); }
void Cpu::TXA() { load(A, X); }
void Cpu::TXS() { S = X; }
void Cpu::TYA() { load(A, Y); }

//Illegal opcs
void Cpu::ALR()
{
	A &= bus.read(currentAddress);
	updateFlag(C, A & 0x01);
	A >>= 1;
	updateFlag(N, A & 0x80);
	updateFlag(Z, A == 0);
}
void Cpu::ANC()
{
	AND();
	updateFlag(C, A & 0x80);
}
void Cpu::ANE()
{
	logic([value = X & bus.read(currentAddress)](uint8_t& A) { A &= value; });
}
void Cpu::ARR()
{
	A = ( (A & bus.read(currentAddress))  >> 1) | 
		(getFlag(C) ? 0x80 : 0x00);

	updateFlag(N, A & 0x80);
	updateFlag(Z, A == 0);
	updateFlag(C, A & 0x40);
	updateFlag(V, ((A >> 6) ^ (A >> 5)) & 1);
}

void Cpu::DCP()
{
	uint8_t value = bus.read(currentAddress) - 1;
	bus.write(value, currentAddress);
	compare(A, value);
}

void Cpu::ISC()
{
	uint8_t value = bus.read(currentAddress) + 1;
	bus.write(value, currentAddress);

	uint16_t result = arithmetic(A, [value, C = getFlag(C)](uint16_t v) {
		return uint16_t(v) - uint16_t(value) - (C ? 0 : 1);
		});
	updateFlag(V, ((A ^ value) & (A ^ uint8_t(result))) & 0x80);
	updateFlag(C, result < 0x100);
	A = uint8_t(result);
}
void Cpu::LAS()
{
	logic([value = bus.read(currentAddress) & S](uint8_t& A) { A &= value; });
	X = S = A;
}
void Cpu::LAX()
{
	LDA();
	TAX();
}
void Cpu::LXA()
{
	logic([value = bus.read(currentAddress) ](uint8_t& A) { A &= value; });
	X = A;
}

void Cpu::RLA()
{
	uint8_t value = shift(0x80, [this](uint8_t v) {
		return (v << 1) | (getFlag(C) ? 0x01 : 0x00);
		});
	logic( [value = value](uint8_t& A) { A &= value; } );

}
void Cpu::RRA()
{
	uint8_t value = shift(0x01, [this](uint8_t v) {
		return (v >> 1) | (getFlag(C) ? 0x80 : 0x00);
		});
	logic([value = value](uint8_t& A) { A &= value; });
}

void Cpu::SAX() { bus.write(A & X, currentAddress); }
void Cpu::SBX()
{
	uint8_t value = bus.read(currentAddress);
	X &= A;
	uint8_t result = X - value;
	updateFlag(C, X >= value);
	updateFlag(Z, result == 0);
	updateFlag(N, result & 0x80);
	X = result;
}
void Cpu::SHA()
{
	bus.write( 
		A & X & (uint8_t(currentAddress >> 8) + 1),
		currentAddress
	);
}
void Cpu::SHX()
{
	bus.write(
		X & (uint8_t(currentAddress >> 8) + 1),
		currentAddress
	);
}
void Cpu::SHY()
{
	bus.write(
		Y & (uint8_t(currentAddress >> 8) + 1),
		currentAddress
	);
}
void Cpu::SLO()
{
	uint8_t value = shift(0x80, [](uint8_t v) { return v << 1; });
	logic( [value = value ] (uint8_t& A)  { A |= value; } );
}
void Cpu::SRE()
{
	uint8_t value = shift(0x01, [](uint8_t v) { return v >> 1; });
	logic([value = value](uint8_t& A) { A |= value; });
}

void Cpu::TAS()
{
	push(A & X);
	SHA();
}

void Cpu::JAM()
{
	PC--;
}


void Cpu::initInstructions()
{
	// tmp fill up of entries for missing insturctions
	for (int i = 0; i < std::size(lookup); i++) {
		lookup[i] = { "XXX", &Cpu::impl, &Cpu::NOP, 2, false };
	}

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
	lookup[0x69] = { "ADC",  &Cpu::imm,  &Cpu::ADC, 2, false };
	lookup[0x65] = { "ADC",  &Cpu::zpg,  &Cpu::ADC, 3, false };
	lookup[0x75] = { "ADC",  &Cpu::zpgX, &Cpu::ADC, 4, false };
	lookup[0x6D] = { "ADC",  &Cpu::abs,  &Cpu::ADC, 4, false };
	lookup[0x7D] = { "ADC",  &Cpu::absX, &Cpu::ADC, 4, true  };
	lookup[0x79] = { "ADC",  &Cpu::absY, &Cpu::ADC, 4, true  };
	lookup[0x61] = { "ADC",  &Cpu::Xind, &Cpu::ADC, 6, false };
	lookup[0x71] = { "ADC",  &Cpu::indY, &Cpu::ADC, 5, true  };
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
	lookup[0x29] = { "AND",  &Cpu::imm,  &Cpu::AND, 2, false };
	lookup[0x25] = { "AND",  &Cpu::zpg,  &Cpu::AND, 3, false };
	lookup[0x35] = { "AND",  &Cpu::zpgX, &Cpu::AND, 4, false };
	lookup[0x2D] = { "AND",  &Cpu::abs,  &Cpu::AND, 4, false };
	lookup[0x3D] = { "AND",  &Cpu::absX, &Cpu::AND, 4, true  };
	lookup[0x39] = { "AND",  &Cpu::absY, &Cpu::AND, 4, true  };
	lookup[0x21] = { "AND",  &Cpu::Xind, &Cpu::AND, 6, false };
	lookup[0x31] = { "AND",  &Cpu::indY, &Cpu::AND, 5, true  };
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
	lookup[0x0A] = { "ASL",  &Cpu::acc,  &Cpu::ASL, 2, false };
	lookup[0x06] = { "ASL",  &Cpu::zpg,  &Cpu::ASL, 5, false };
	lookup[0x16] = { "ASL",  &Cpu::zpgX, &Cpu::ASL, 6, false };
	lookup[0x0E] = { "ASL",  &Cpu::abs,  &Cpu::ASL, 6, false };
	lookup[0x1E] = { "ASL",  &Cpu::absX, &Cpu::ASL, 7, false };
	/* -------------------------------------------------
	BCC
		Branch on Carry Clear

		branch on C = 0
		NZCIDV
		------
		addressing	assembler	opc	bytes	cycles
		relative	BCC oper	90	2		2 * *

	*/
	lookup[0x90] = { "BCC",  &Cpu::rel,  &Cpu::BCC, 2, true  };
	/* -------------------------------------------------
	BCS
		Branch on Carry Set

		branch on C = 1
		NZCIDV
		------
		addressing	assembler	opc	bytes	cycles
		relative	BCS oper	B0	2		2 * *
	*/
	lookup[0xB0] = { "BCS",  &Cpu::rel,  &Cpu::BCS, 2, true  };
	/* -------------------------------------------------
	BEQ
		Branch on Result Zero

		branch on Z = 1
		NZCIDV
		------
		addressing	assembler	opc	bytes	cycles
		relative	BEQ oper	F0	2		2 * *
	*/
	lookup[0xF0] = { "BEQ",  &Cpu::rel,  &Cpu::BEQ, 2, true  };
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
	lookup[0x24] = { "BIT",  &Cpu::zpg,  &Cpu::BIT, 3, false };
	lookup[0x2C] = { "BIT",  &Cpu::abs,  &Cpu::BIT, 4, false };
	/* -------------------------------------------------
	BMI
		Branch on Result Minus

		branch on N = 1
		NZCIDV
		------
		addressing	assembler	opc	bytes	cycles
		relative	BMI oper	30	2		2 * *
	*/
	lookup[0x30] = { "BMI",  &Cpu::rel,  &Cpu::BMI, 2, true  };
	/* -------------------------------------------------
	BNE
		Branch on Result not Zero

		branch on Z = 0
		NZCIDV
		------
		addressing	assembler	opc	bytes	cycles
		relative	BNE oper	D0	2		2 * *
	*/
	lookup[0xD0] = { "BNE",  &Cpu::rel,  &Cpu::BNE, 2, true  };
	/* -------------------------------------------------
	BPL
		Branch on Result Plus

		branch on N = 0
		NZCIDV
		------
		addressing	assembler	opc	bytes	cycles
		relative	BPL oper	10	2		2 * *
	*/
	lookup[0x10] = { "BPL",  &Cpu::rel,  &Cpu::BPL, 2, true  };
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
	lookup[0x00] = { "BRK",  &Cpu::impl,  &Cpu::BRK, 7, false };
	/* -------------------------------------------------
	BVC
		Branch on Overflow Clear

		branch on V = 0
		NZCIDV
		------
		addressing	assembler	opc	bytes	cycles
		relative	BVC oper	50	2		2 * *
	*/
	lookup[0x50] = { "BVC",  &Cpu::rel,  &Cpu::BVC, 2, true  };
	/* -------------------------------------------------
	BVS
		Branch on Overflow Set

		branch on V = 1
		NZCIDV
		------
		addressing	assembler	opc	bytes	cycles
		relative	BVS oper	70	2		2 * *
	*/
	lookup[0x70] = { "BVS",  &Cpu::rel,  &Cpu::BVS, 2, true  };
	/* -------------------------------------------------
	CLC
		Clear Carry Flag

		0->C
		NZCIDV
		--0---
		addressing	assembler	opc	bytes	cycles
		implied		CLC			18	1		2
	*/
	lookup[0x18] = { "CLC",  &Cpu::impl,  &Cpu::CLC, 2, false };
	/* -------------------------------------------------
	CLD
		Clear Decimal Mode

		0->D
		NZCIDV
		----0-
		addressing	assembler	opc	bytes	cycles
		implied		CLD			D8	1		2
	*/
	lookup[0xD8] = { "CLD",  &Cpu::impl,  &Cpu::CLD, 2, false };
	/* -------------------------------------------------
	CLI
		Clear Interrupt Disable Bit

		0->I
		NZCIDV
		---0--
		addressing	assembler	opc	bytes	cycles
		implied		CLI			58	1		2
	*/
	lookup[0x58] = { "CLI",  &Cpu::impl,  &Cpu::CLI, 2, false };
	/* -------------------------------------------------
	CLV
		Clear Overflow Flag

		0->V
		NZCIDV
		-----0
		addressing	assembler	opc	bytes	cycles
		implied		CLV			B8	1		2
	*/
	lookup[0xB8] = { "CLV",  &Cpu::impl,  &Cpu::CLV, 2, false };
	/* -------------------------------------------------
	CMP
		Compare Memory with Accumulator

		A - M
		NZCIDV
		+++---
		addressing		assembler			opc	bytes	cycles
		immediate		CMP #oper			C9	2		2
		zeropage		CMP oper			C5	2		3
		zeropage, X		CMP oper, X			D5	2		4
		absolute		CMP oper			CD	3		4
		absolute, X		CMP oper, X			DD	3		4 *
		absolute, Y		CMP oper, Y			D9	3		4 *
		(indirect, X)	CMP(oper, X)		C1	2		6
		(indirect), Y	CMP(oper), Y		D1	2		5 *
	*/
	lookup[0xC9] = { "CMP",  &Cpu::imm,  &Cpu::CMP, 2, false };
	lookup[0xC5] = { "CMP",  &Cpu::zpg,  &Cpu::CMP, 3, false };
	lookup[0xD5] = { "CMP",  &Cpu::zpgX, &Cpu::CMP, 4, false };
	lookup[0xCD] = { "CMP",  &Cpu::abs,  &Cpu::CMP, 4, false };
	lookup[0xDD] = { "CMP",  &Cpu::absX, &Cpu::CMP, 4, true  };
	lookup[0xD9] = { "CMP",  &Cpu::absY, &Cpu::CMP, 4, true  };
	lookup[0xC1] = { "CMP",  &Cpu::Xind, &Cpu::CMP, 6, false };
	lookup[0xD1] = { "CMP",  &Cpu::indY, &Cpu::CMP, 5, true  };
	/* -------------------------------------------------
	CPX
		Compare Memory and Index X

		X - M
		NZCIDV
		+++---
		addressing	assembler	opc	bytes	cycles
		immediate	CPX #oper	E0	2		2
		zeropage	CPX oper	E4	2		3
		absolute	CPX oper	EC	3		4
	*/
	lookup[0xE0] = { "CPX",  &Cpu::imm,  &Cpu::CPX, 2, false };
	lookup[0xE4] = { "CPX",  &Cpu::zpg,  &Cpu::CPX, 3, false };
	lookup[0xEC] = { "CPX",  &Cpu::abs,  &Cpu::CPX, 4, false };
	/* -------------------------------------------------
	CPY
		Compare Memory and Index Y

		Y - M
		NZCIDV
		+++---
		addressing	assembler	opc	bytes	cycles
		immediate	CPY #oper	C0	2		2
		zeropage	CPY oper	C4	2		3
		absolute	CPY oper	CC	3		4
	*/
	lookup[0xC0] = { "CPY",  &Cpu::imm,  &Cpu::CPY, 2, false };
	lookup[0xC4] = { "CPY",  &Cpu::zpg,  &Cpu::CPY, 3, false };
	lookup[0xCC] = { "CPY",  &Cpu::abs,  &Cpu::CPY, 4, false };
	/* -------------------------------------------------
	DEC
		Decrement Memory by One

		M - 1->M
		NZCIDV
		++----
		addressing	assembler	opc	bytes	cycles
		zeropage	DEC oper	C6	2		5
		zeropage, X	DEC oper, X	D6	2		6
		absolute	DEC oper	CE	3		6
		absolute, X	DEC oper, X	DE	3		7
	*/
	lookup[0xC6] = { "DEC",  &Cpu::zpg,  &Cpu::DEC, 5, false };
	lookup[0xD6] = { "DEC",  &Cpu::zpgX, &Cpu::DEC, 6, false };
	lookup[0xCE] = { "DEC",  &Cpu::abs,  &Cpu::DEC, 6, false };
	lookup[0xDE] = { "DEC",  &Cpu::absX, &Cpu::DEC, 7, false };
	/* -------------------------------------------------
	DEX
		Decrement Index X by One

		X - 1->X
		NZCIDV
		++----
		addressing	assembler	opc	bytes	cycles
		implied		DEX			CA	1		2
	*/
	lookup[0xCA] = { "DEX",  &Cpu::impl,  &Cpu::DEX, 2, false };
	/* -------------------------------------------------
	DEY
		Decrement Index Y by One

		Y - 1->Y
		NZCIDV
		++----
		addressing	assembler	opc	bytes	cycles
		implied		DEY			88	1		2
	*/
	lookup[0x88] = { "DEY",  &Cpu::impl,  &Cpu::DEY, 2, false };
	/* -------------------------------------------------
	EOR
		Exclusive - OR Memory with Accumulator

		A EOR M->A
		NZCIDV
		++----
		addressing		assembler		opc	bytes	cycles
		immediate		EOR #oper		49	2		2
		zeropage		EOR oper		45	2		3
		zeropage, X		EOR oper, X		55	2		4
		absolute		EOR oper		4D	3		4
		absolute, X		EOR oper, X		5D	3		4 *
		absolute, Y		EOR oper, Y		59	3		4 *
		(indirect, X)	EOR(oper, X)	41	2		6
	*/
	lookup[0x49] = { "EOR",  &Cpu::imm,  &Cpu::EOR, 2, false };
	lookup[0x45] = { "EOR",  &Cpu::zpg,  &Cpu::EOR, 3, false };
	lookup[0x55] = { "EOR",  &Cpu::zpgX, &Cpu::EOR, 4, false };
	lookup[0x4D] = { "EOR",  &Cpu::abs,  &Cpu::EOR, 4, false };
	lookup[0x5D] = { "EOR",  &Cpu::absX, &Cpu::EOR, 4, true  };
	lookup[0x59] = { "EOR",  &Cpu::absY, &Cpu::EOR, 4, true  };
	lookup[0x41] = { "EOR",  &Cpu::Xind, &Cpu::EOR, 6, false };
	/* -------------------------------------------------
	INC
		Increment Memory by One

		M + 1->M
		NZCIDV
		++----
		addressing	assembler	opc	bytes	cycles
		zeropage	INC oper	E6	2		5
		zeropage, X	INC oper, X	F6	2		6
		absolute	INC oper	EE	3		6
		absolute, X	INC oper, X	FE	3		7
	*/
	lookup[0xE6] = { "INC",  &Cpu::zpg,  &Cpu::INC, 5, false };
	lookup[0xF6] = { "INC",  &Cpu::zpgX, &Cpu::INC, 6, false };
	lookup[0xEE] = { "INC",  &Cpu::abs,  &Cpu::INC, 6, false };;
	lookup[0xFE] = { "INC",  &Cpu::absX, &Cpu::INC, 7, false };
	/* -------------------------------------------------
	INX
		Increment Index X by One

		X + 1->X
		NZCIDV
		++----
		addressing	assembler	opc	bytes	cycles
		implied		INX			E8	1		2
	*/
	lookup[0xE8] = { "INX",  &Cpu::impl,  &Cpu::INX, 2, false };
	/* -------------------------------------------------
	INY
		Increment Index Y by One

		Y + 1->Y
		NZCIDV
		++----
		addressing	assembler	opc	bytes	cycles
		implied		INY			C8	1		2
	*/
	lookup[0xC8] = { "INY",  &Cpu::impl,  &Cpu::INY, 2, false };
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
	lookup[0x4C] = { "JMP",  &Cpu::abs,  &Cpu::JMP, 3, false };
	lookup[0x6C] = { "JMP",  &Cpu::ind,  &Cpu::JMP, 5, true };
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
	lookup[0x20] = { "JSR",  &Cpu::abs,  &Cpu::JSR, 6, false };
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
	lookup[0xA9] = { "LDA",  &Cpu::imm,  &Cpu::LDA, 2, false };
	lookup[0xA5] = { "LDA",  &Cpu::zpg,  &Cpu::LDA, 3, false };
	lookup[0xB5] = { "LDA",  &Cpu::zpgX, &Cpu::LDA, 4, false };
	lookup[0xAD] = { "LDA",  &Cpu::abs,  &Cpu::LDA, 4, false };
	lookup[0xBD] = { "LDA",  &Cpu::absX, &Cpu::LDA, 4, true  };
	lookup[0xB9] = { "LDA",  &Cpu::absY, &Cpu::LDA, 4, true  };
	lookup[0xA1] = { "LDA",  &Cpu::Xind, &Cpu::LDA, 6, false };
	lookup[0xB1] = { "LDA",  &Cpu::indY, &Cpu::LDA, 5, true  };
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
	lookup[0xA2] = { "LDX",  &Cpu::imm,  &Cpu::LDX, 2, false };
	lookup[0xA6] = { "LDX",  &Cpu::zpg,  &Cpu::LDX, 3, false };
	lookup[0xB6] = { "LDX",  &Cpu::zpgY, &Cpu::LDX, 4, false };
	lookup[0xAE] = { "LDX",  &Cpu::abs,  &Cpu::LDX, 4, false };
	lookup[0xBE] = { "LDX",  &Cpu::absY, &Cpu::LDX, 4, true  };
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
	lookup[0xA0] = { "LDY",  &Cpu::imm,  &Cpu::LDY, 2, false };
	lookup[0xA4] = { "LDY",  &Cpu::zpg,  &Cpu::LDY, 3, false };
	lookup[0xB4] = { "LDY",  &Cpu::zpgX, &Cpu::LDY, 4, false };
	lookup[0xAC] = { "LDY",  &Cpu::abs,  &Cpu::LDY, 4, false };
	lookup[0xBC] = { "LDY",  &Cpu::absX, &Cpu::LDY, 4, true  };
	/* -------------------------------------------------
	LSR
		Shift One Bit Right(Memory or Accumulator)

		0 ->[76543210]->C
		NZCIDV
		0++---
		addressing	assembler	opc	bytes	cycles
		accumulator	LSR A		4A	1		2
		zeropage	LSR oper	46	2		5
		zeropage, X	LSR oper, X	56	2		6
		absolute	LSR oper	4E	3		6
		absolute, X	LSR oper, X	5E	3		7
	*/
	lookup[0x4A] = { "LSR",  &Cpu::acc,  &Cpu::LSR, 2, false };
	lookup[0x46] = { "LSR",  &Cpu::zpg,  &Cpu::LSR, 5, false };
	lookup[0x56] = { "LSR",  &Cpu::zpgX, &Cpu::LSR, 6, false };
	lookup[0x4E] = { "LSR",  &Cpu::abs,  &Cpu::LSR, 6, false };
	lookup[0x5E] = { "LSR",  &Cpu::absX, &Cpu::LSR, 7, true  };
	/* -------------------------------------------------
	NOP
		No Operation

		-- -
		NZCIDV
		------
		addressing	assembler	opc		bytes	cycles
		implied		NOP			EA		1		2
	*/
	lookup[0xEA] = { "NOP",  &Cpu::impl, &Cpu::NOP, 2, false };
	/* -------------------------------------------------
	ORA
		OR Memory with Accumulator

		A OR M->A
		NZCIDV
		++----
		addressing		assembler		opc	bytes	cycles
		immediate		ORA #oper		09	2		2
		zeropage		ORA oper		05	2		3
		zeropage, X		ORA oper, X		15	2		4
		absolute		ORA oper		0D	3		4
		absolute, X		ORA oper, X		1D	3		4 *
		absolute, Y		ORA oper, Y		19	3		4 *
		(indirect, X)	ORA(oper, X)	01	2		6
		(indirect), Y	ORA(oper), Y	11	2		5 *
	*/
	lookup[0x09] = { "ORA",  &Cpu::imm,  &Cpu::ORA, 2, false };
	lookup[0x05] = { "ORA",  &Cpu::zpg,  &Cpu::ORA, 3, false };
	lookup[0x15] = { "ORA",  &Cpu::zpgX, &Cpu::ORA, 4, false };
	lookup[0x0D] = { "ORA",  &Cpu::abs,  &Cpu::ORA, 4, false };
	lookup[0x1D] = { "ORA",  &Cpu::absX, &Cpu::ORA, 4, true  };
	lookup[0x19] = { "ORA",  &Cpu::absY, &Cpu::ORA, 4, true };
	lookup[0x01] = { "ORA",  &Cpu::Xind, &Cpu::ORA, 6, false };
	lookup[0x11] = { "ORA",  &Cpu::indY, &Cpu::ORA, 5, true };
	/* -------------------------------------------------
	PHA
		Push Accumulator on Stack

		push A
		NZCIDV
		------
		addressing	assembler	opc	bytes	cycles
		implied		PHA			48	1		3
	*/
	lookup[0x48] = { "PHA",  &Cpu::impl, &Cpu::PHA, 3, false };
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
	lookup[0x08] = { "PHP",  &Cpu::impl, &Cpu::PHP, 3, false };
	/* -------------------------------------------------
	PLA
		Pull Accumulator from Stack

		pull A
		NZCIDV
		++----
		addressing	assembler	opc	bytes	cycles
		implied		PLA			68	1		4
	*/
	lookup[0x68] = { "PLA",  &Cpu::impl, &Cpu::PLA, 4, false };
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
	lookup[0x28] = { "PLP",  &Cpu::impl, &Cpu::PLP, 4, false };
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
	lookup[0x2A] = { "ROL",  &Cpu::acc,  &Cpu::ROL, 2, false };
	lookup[0x26] = { "ROL",  &Cpu::zpg,  &Cpu::ROL, 5, false };
	lookup[0x36] = { "ROL",  &Cpu::zpgX, &Cpu::ROL, 6, false };
	lookup[0x2E] = { "ROL",  &Cpu::abs,  &Cpu::ROL, 6, false };
	lookup[0x3E] = { "ROL",  &Cpu::absX, &Cpu::ROL, 7, false };
	/* -------------------------------------------------
	ROR
		Rotate One Bit Right(Memory or Accumulator)

		C ->[76543210]->C
		NZCIDV
		+++---
		addressing	assembler	opc	bytes	cycles
		accumulator	ROR A		6A	1		2
		zeropage	ROR oper	66	2		5
		zeropage, X	ROR oper, X	76	2		6
		absolute	ROR oper	6E	3		6
		absolute, X	ROR oper, X	7E	3		7
	*/
	lookup[0x6A] = { "ROR",  &Cpu::acc,  &Cpu::ROR, 2, false };
	lookup[0x66] = { "ROR",  &Cpu::zpg,  &Cpu::ROR, 5, false };
	lookup[0x76] = { "ROR",  &Cpu::zpgX, &Cpu::ROR, 6, false };;
	lookup[0x6E] = { "ROR",  &Cpu::abs,  &Cpu::ROR, 6, false };
	lookup[0x7E] = { "ROR",  &Cpu::absX, &Cpu::ROR, 7, false };
	/* -------------------------------------------------
	RTI
		Return from Interrupt

		The status register is pulled with the break flag
		and bit 5 ignored.Then PC is pulled from the stack.

		pull SR, pull PC
		NZCIDV
		from stack
		addressing	assembler	opc	bytes	cycles
		implied		RTI			40	1		6
	*/
	lookup[0x40] = { "RTI",  &Cpu::impl,  &Cpu::RTI, 6, false };
	/* -------------------------------------------------
	RTS
		Return from Subroutine

		pull PC, PC + 1->PC
		NZCIDV
		------
		addressing	assembler	opc	bytes	cycles
		implied		RTS			60	1		6
	*/
	lookup[0x60] = { "RTS",  &Cpu::impl,  &Cpu::RTS, 6, false };
	/* -------------------------------------------------
	SBC
		Subtract Memory from Accumulator with Borrow

		A - M - C->A
		NZCIDV
		+++--+
		addressing		assembler		opc	bytes	cycles
		immediate		SBC #oper		E9	2		2
		zeropage		SBC oper		E5	2		3
		zeropage, X		SBC oper, X		F5	2		4
		absolute		SBC oper		ED	3		4
		absolute, X		SBC oper, X		FD	3		4 *
		absolute, Y		SBC oper, Y		F9	3		4 *
		(indirect, X)	SBC(oper, X)	E1	2		6
		(indirect), Y	SBC(oper), Y	F1	2		5 *
	*/
	lookup[0xE9] = { "SBC",  &Cpu::imm,  &Cpu::SBC, 2, false };
	lookup[0xE5] = { "SBC",  &Cpu::zpg,  &Cpu::SBC, 3, false };
	lookup[0xF5] = { "SBC",  &Cpu::zpgX, &Cpu::SBC, 4, false };
	lookup[0xED] = { "SBC",  &Cpu::abs,  &Cpu::SBC, 4, false };
	lookup[0xFD] = { "SBC",  &Cpu::absX, &Cpu::SBC, 4, true  };
	lookup[0xF9] = { "SBC",  &Cpu::absY, &Cpu::SBC, 4, true  };
	lookup[0xE1] = { "SBC",  &Cpu::Xind, &Cpu::SBC, 6, false };
	lookup[0xF1] = { "SBC",  &Cpu::indY, &Cpu::SBC, 5, true  };
	/* -------------------------------------------------
	SEC
		Set Carry Flag

		1->C
		NZCIDV
		--1---
		addressing	assembler	opc	bytes	cycles
		implied		SEC			38	1		2
	*/
	lookup[0x38] = { "SEC",  &Cpu::impl,  &Cpu::SEC, 2, false };
	/* -------------------------------------------------
	SED
		Set Decimal Flag

		1->D
		NZCIDV
		----1-
		addressing	assembler	opc	bytes	cycles
		implied		SED			F8	1		2
	*/
	lookup[0xF8] = { "SED",  &Cpu::impl,  &Cpu::SED, 2, false };
	/* -------------------------------------------------
	SEI
		Set Interrupt Disable Status

		1->I
		NZCIDV
		---1--
		addressing	assembler	opc	bytes	cycles
		implied		SEI			78	1		2
	*/
	lookup[0x78] = { "SEI",  &Cpu::impl,  &Cpu::SEI, 2, false };
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
	lookup[0x85] = { "STA",  &Cpu::zpg,  &Cpu::STA, 3, false };
	lookup[0x95] = { "STA",  &Cpu::zpgX, &Cpu::STA, 4, false };
	lookup[0x8D] = { "STA",  &Cpu::abs,  &Cpu::STA, 4, false };
	lookup[0x9D] = { "STA",  &Cpu::absX, &Cpu::STA, 5, false };
	lookup[0x99] = { "STA",  &Cpu::absY, &Cpu::STA, 5, false };
	lookup[0x81] = { "STA",  &Cpu::Xind, &Cpu::STA, 6, false };
	lookup[0x91] = { "STA",  &Cpu::indY, &Cpu::STA, 6, false };
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
	lookup[0x86] = { "STX",  &Cpu::zpg,  &Cpu::STX, 3, false };
	lookup[0x96] = { "STX",  &Cpu::zpgY, &Cpu::STX, 4, false };
	lookup[0x8E] = { "STX",  &Cpu::abs,  &Cpu::STX, 4, false };
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
	lookup[0x84] = { "STY",  &Cpu::zpg,  &Cpu::STY, 3, false };
	lookup[0x94] = { "STY",  &Cpu::zpgX, &Cpu::STY, 4, false };
	lookup[0x8C] = { "STY",  &Cpu::abs,  &Cpu::STY, 4, false };
	/* -------------------------------------------------
	TAX
		Transfer Accumulator to Index X

		A->X
		NZCIDV
		++----
		addressing	assembler	opc	bytes	cycles
		implied		TAX			AA	1		2
	*/
	lookup[0xAA] = { "TAX",  &Cpu::impl, &Cpu::TAX, 2, false };
	/* -------------------------------------------------
	TAY
		Transfer Accumulator to Index Y

		A->Y
		NZCIDV
		++----
		addressing	assembler	opc	bytes	cycles
		implied		TAY			A8	1		2
	*/
	lookup[0xA8] = { "TAY",  &Cpu::impl, &Cpu::TAY, 2, false };
	/* -------------------------------------------------
	TSX
		Transfer Stack Pointer to Index X

		SP->X
		NZCIDV
		++----
		addressing	assembler	opc	bytes	cycles
		implied		TSX			BA	1		2
	*/
	lookup[0xBA] = { "TSX",  &Cpu::impl, &Cpu::TSX, 2, false };
	/* -------------------------------------------------
	TXA
		Transfer Index X to Accumulator

		X->A
		NZCIDV
		++----
		addressing	assembler	opc	bytes	cycles
		implied		TXA			8A	1		2
	*/
	lookup[0x8A] = { "TXA",  &Cpu::impl, &Cpu::TXA, 2, false };
	/* -------------------------------------------------
	TXS
		Transfer Index X to Stack Register

		X->SP
		NZCIDV
		------
		addressing	assembler	opc	bytes	cycles
		implied		TXS			9A	1		2
	*/
	lookup[0x9A] = { "TXS",  &Cpu::impl, &Cpu::TXS, 2, false };
	/* -------------------------------------------------
	TYA
		Transfer Index Y to Accumulator

		Y->A
		NZCIDV
		++----
		addressing	assembler	opc	bytes	cycles
		implied		TYA			98	1		2
	*/
	lookup[0x98] = { "TYA",  &Cpu::impl, &Cpu::TYA, 2, false };
	











	// ----------- Illegal Codes -----------
	/* -------------------------------------------------
	ALR (ASR)
	AND oper + LSR

	A AND oper, 0 -> [76543210] -> C

		NZCIDV
		+++---
	addressing	assembler	opc	bytes	cycles	
	immediate	ALR #oper	4B	2		2  
	*/
	lookup[0x4B] = { "ALR",  &Cpu::imm, &Cpu::ALR, 2, false };
	/* -------------------------------------------------
	ANC
		AND oper + set C as ASL

		A AND oper, bit(7)->C

		NZCIDV
		+++---
		addressing	assembler	opc	bytes	cycles
		immediate	ANC #oper	0B	2		2
		immediate	ANC #oper	2B	2		2
	*/
	lookup[0x0B] = { "ANC",  &Cpu::imm, &Cpu::ANC, 2, false };
	lookup[0x2B] = { "ANC",  &Cpu::imm, &Cpu::ANC, 2, false };
	/* -------------------------------------------------
	ANE(XAA)
		* OR X + AND oper

		Highly unstable, do not use.

		A base value in A is determined based on the contets of A and a constant, which may be 
		typically $00, $ff, $ee, etc.The value of this constant depends on temerature, 
		the chip series, and maybe other factors, as well.
		In order to eliminate these uncertaincies from the equation, 
		use either 0 as the operand or a value of $FF in the accumulator.

		(A OR CONST) AND X AND oper->A

		NZCIDV
		+++---
		addressing	assembler	opc	bytes	cycles
		immediate	ANE #oper	8B	2		2  
	*/
	lookup[0x8B] = { "ANE",  &Cpu::imm, &Cpu::ANE, 2, false };
	/* -------------------------------------------------
	ARR
		AND oper + ROR

		This operation involves the adder :
		V - flag is set according to(A AND oper) + oper
		The carry is not set, but bit 7 (sign)is exchanged with the carry

		A AND oper, C ->[76543210]->C

		NZCIDV
		+++--+
		addressing	assembler	opc	bytes	cycles
		immediate	ARR #oper	6B	2		2
	*/
	lookup[0x6B] = { "ARR",  &Cpu::imm, &Cpu::ARR, 2, false };
	/* -------------------------------------------------
	DCP(DCM)
		DEC oper + CMP oper

		M - 1->M, A - M

		Decrements the operand and then compares the result to the accumulator.

		NZCIDV
		+++---
		addressing		assembler		opc	bytes	cycles
		zeropage		DCP oper		C7	2		5
		zeropage, X		DCP oper, X		D7	2		6
		absolute		DCP oper		CF	3		6
		absolute, X		DCP oper, X		DF	3		7
		absolute, Y		DCP oper, Y		DB	3		7
		(indirect, X)	DCP(oper, X)	C3	2		8
		(indirect), Y	DCP(oper), Y	D3	2		8
	*/
	lookup[0xC7] = { "DCP",  &Cpu::zpg,  &Cpu::DCP, 5, false };
	lookup[0xD7] = { "DCP",  &Cpu::zpgX, &Cpu::DCP, 6, false };
	lookup[0xCF] = { "DCP",  &Cpu::abs,  &Cpu::DCP, 6, false };
	lookup[0xDF] = { "DCP",  &Cpu::absX, &Cpu::DCP, 7, false };
	lookup[0xDB] = { "DCP",  &Cpu::absY, &Cpu::DCP, 7, false };
	lookup[0xC3] = { "DCP",  &Cpu::Xind, &Cpu::DCP, 8, false };
	lookup[0xD3] = { "DCP",  &Cpu::indY, &Cpu::DCP, 8, false };
	/* -------------------------------------------------
	ISC(ISB, INS)
		INC oper + SBC oper

		M + 1->M, A - M - ~C->A

		NZCIDV
		+++--+
		addressing		assembler		opc	bytes	cycles
		zeropage		ISC oper		E7	2		5
		zeropage, X		ISC oper, X		F7	2		6
		absolute		ISC oper		EF	3		6
		absolute, X		ISC oper, X		FF	3		7
		absolute, Y		ISC oper, Y		FB	3		7
		(indirect, X)	ISC(oper, X)	E3	2		8
		(indirect), Y	ISC(oper), Y	F3	2		8
	*/
	lookup[0xE7] = { "ISC",  &Cpu::zpg,  &Cpu::ISC, 5, false };
	lookup[0xF7] = { "ISC",  &Cpu::zpgX, &Cpu::ISC, 6, false };
	lookup[0xEF] = { "ISC",  &Cpu::abs,  &Cpu::ISC, 6, false };
	lookup[0xFF] = { "ISC",  &Cpu::absX, &Cpu::ISC, 7, false };
	lookup[0xFB] = { "ISC",  &Cpu::absY, &Cpu::ISC, 7, false };
	lookup[0xE3] = { "ISC",  &Cpu::Xind, &Cpu::ISC, 8, false };
	lookup[0xF3] = { "ISC",  &Cpu::indY, &Cpu::ISC, 8, false };
	/* -------------------------------------------------
	LAS(LAR)
		LDA / TSX oper

		M AND SP->A, X, SP

		NZCIDV
		++----
		addressing		assembler		opc	bytes	cycles
		absolute, Y		LAS oper, Y		BB	3		4 *
	*/
	lookup[0xBB] = { "LAS",  &Cpu::absY, &Cpu::LAS, 4, true };
	/* -------------------------------------------------
	LAX
		LDA oper + LDX oper

		M->A->X

		NZCIDV
		++----
		addressing		assembler		opc	bytes	cycles
		zeropage		LAX oper		A7	2		3
		zeropage, Y		LAX oper, Y		B7	2		4
		absolute		LAX oper		AF	3		4
		absolute, Y		LAX oper, Y		BF	3		4 *
		(indirect, X)	LAX(oper, X)	A3	2		6
		(indirect), Y	LAX(oper), Y	B3	2		5 *
	*/
	lookup[0xA7] = { "LAX",  &Cpu::zpg,  &Cpu::LAX, 3, false };
	lookup[0xB7] = { "LAX",  &Cpu::zpgY, &Cpu::LAX, 4, false };
	lookup[0xAF] = { "LAX",  &Cpu::abs,  &Cpu::LAX, 4, false };
	lookup[0xBF] = { "LAX",  &Cpu::absY, &Cpu::LAX, 4, true };
	lookup[0xA3] = { "LAX",  &Cpu::Xind, &Cpu::LAX, 6, false };
	lookup[0xB3] = { "LAX",  &Cpu::indY, &Cpu::LAX, 5, true };
	/* -------------------------------------------------
	LXA(LAX immediate)
		Store* AND oper in A and X

		Highly unstable, involves a 'magic' constant, see ANE

		(A OR CONST) AND oper->A->X

		NZCIDV
		++----
		addressing	assembler	opc	bytes	cycles
		immediate	LXA #oper	AB	2		2
	*/
	lookup[0xAB] = { "LXA",  &Cpu::imm, &Cpu::LXA, 2, false };
	/* -------------------------------------------------
	RLA
		ROL oper + AND oper

		M = C < -[76543210] < -C, A AND M->A

		NZCIDV
		+++---
		addressing		assembler		opc		bytes	cycles
		zeropage		RLA oper		27		2		5
		zeropage, X		RLA oper, X		37		2		6
		absolute		RLA oper		2F		3		6
		absolute, X		RLA oper, X		3F		3		7
		absolute, Y		RLA oper, Y		3B		3		7
		(indirect, X)	RLA(oper, X)	23		2		8
		(indirect), Y	RLA(oper), Y	33		2		8
	*/
	lookup[0x27] = { "RLA",  &Cpu::zpg,  &Cpu::RLA, 5, false };
	lookup[0x37] = { "RLA",  &Cpu::zpgX, &Cpu::RLA, 6, false };
	lookup[0x2F] = { "RLA",  &Cpu::abs,  &Cpu::RLA, 6, false };
	lookup[0x3F] = { "RLA",  &Cpu::absX, &Cpu::RLA, 7, false };
	lookup[0x3B] = { "RLA",  &Cpu::absY, &Cpu::RLA, 7, false };
	lookup[0x23] = { "RLA",  &Cpu::Xind, &Cpu::RLA, 8, false };
	lookup[0x33] = { "RLA",  &Cpu::indY, &Cpu::RLA, 8, false };
	/* -------------------------------------------------
	RRA
		ROR oper + AND oper

		M = C < -[76543210] < -C, A AND M->A

		NZCIDV
		+++--+
		addressing		assembler		opc	bytes	cycles
		zeropage		RRA oper		67	2		5
		zeropage,X		RRA oper,X		77	2		6
		absolute		RRA oper		6F	3		6
		absolute,X		RRA oper,X		7F	3		7
		absolute,Y		RRA oper,Y		7B	3		7
		(indirect,X)	RRA (oper,X)	63	2		8
		(indirect),Y	RRA (oper),Y	73	2		8
	*/
	lookup[0x67] = { "RRA",  &Cpu::zpg,  &Cpu::RRA, 5, false };
	lookup[0x77] = { "RRA",  &Cpu::zpgX, &Cpu::RRA, 6, false };
	lookup[0x6F] = { "RRA",  &Cpu::abs,  &Cpu::RRA, 6, false };
	lookup[0x7F] = { "RRA",  &Cpu::absX, &Cpu::RRA, 7, false };
	lookup[0x7B] = { "RRA",  &Cpu::absY, &Cpu::RRA, 7, false };
	lookup[0x63] = { "RRA",  &Cpu::Xind, &Cpu::RRA, 8, false };
	lookup[0x73] = { "RRA",  &Cpu::indY, &Cpu::RRA, 8, false };
	/* -------------------------------------------------
	SAX(AXS, AAX)
		A and X are put on the bus at the same time(resulting 
		effectively in an AND operation) and stored in M

		A AND X->M

		NZCIDV
		------
		addressing		assembler		opc	bytes	cycles
		zeropage		SAX oper		87	2		3
		zeropage, Y		SAX oper, Y		97	2		4
		absolute		SAX oper		8F	3		4
		(indirect, X)	SAX(oper, X)	83	2		6
	*/
	lookup[0x87] = { "SAX",  &Cpu::zpg,  &Cpu::SAX, 3, false };
	lookup[0x97] = { "SAX",  &Cpu::zpgY, &Cpu::SAX, 4, false };
	lookup[0x8F] = { "SAX",  &Cpu::abs,  &Cpu::SAX, 4, false };
	lookup[0x83] = { "SAX",  &Cpu::Xind, &Cpu::SAX, 6, false };
	/* -------------------------------------------------
	SBX(AXS, SAX)
		CMP and DEX at once, sets flags like CMP

		(A AND X) - oper->X

		NZCIDV
		+++---
		addressing	assembler	opc	bytes	cycles
		immediate	SBX #oper	CB	2		2
	*/
	lookup[0xCB] = { "SBX",  &Cpu::imm,  &Cpu::SBX, 3, false };
	/* -------------------------------------------------
	SHA(AHX, AXA)
		Stores A AND X AND(high - byte of addr. + 1) at addr.

		unstable: sometimes 'AND (H+1)' is dropped, 
		page boundary crossings may not work(with the high - byte 
		of the value used as the high - byte of the address)

		A AND X AND(H + 1)->M

		NZCIDV
		------
		addressing		assembler		opc	bytes	cycles
		absolute, Y		SHA oper, Y		9F	3		5  
		(indirect), Y	SHA(oper), Y	93	2		6
	*/
	lookup[0x9F] = { "SHA",  &Cpu::absY,  &Cpu::SHA, 5, false };
	lookup[0x93] = { "SHA",  &Cpu::indY,  &Cpu::SHA, 6, false };
	/* -------------------------------------------------
	SHX(A11, SXA, XAS)
		Stores X AND(high - byte of addr. + 1) at addr.

		unstable: sometimes 'AND (H+1)' is dropped, 
		page boundary crossings may not work(with the high - byte 
		of the value used as the high - byte of the address)

		X AND(H + 1)->M

		NZCIDV
		------
		addressing	assembler	opc	bytes	cycles
		absolute, Y	SHX oper, Y	9E	3		5
	*/
	lookup[0x9E] = { "SHX",  &Cpu::absY,  &Cpu::SHX, 5, false };
	/* -------------------------------------------------
	SHY(A11, SYA, SAY)
		Stores Y AND(high - byte of addr. + 1) at addr.

		unstable: sometimes 'AND (H+1)' is dropped, 
		page boundary crossings may not work(with the high - byte 
		of the value used as the high - byte of the address)

		Y AND(H + 1)->M

		NZCIDV
		------
		addressing	assembler	opc	bytes	cycles
		absolute, X	SHY oper, X	9C	3		5
	*/
	lookup[0x9C] = { "SHY",  &Cpu::absX,  &Cpu::SHY, 5, false };
	/* -------------------------------------------------
	SLO(ASO)
		ASL oper + ORA oper

		M = C < -[76543210] < -0, A OR M->A

		NZCIDV
		+++---
		addressing		assembler		opc	bytes	cycles
		zeropage		SLO oper		07	2		5
		zeropage, X		SLO oper, X		17	2		6
		absolute		SLO oper		0F	3		6
		absolute, X		SLO oper, X		1F	3		7
		absolute, Y		SLO oper, Y		1B	3		7
		(indirect, X)	SLO(oper, X)	03	2		8
		(indirect), Y	SLO(oper), Y	13	2		8
	*/
	lookup[0x07] = { "SLO",  &Cpu::zpg,  &Cpu::SLO, 5, false };
	lookup[0x17] = { "SLO",  &Cpu::zpgX, &Cpu::SLO, 6, false };
	lookup[0x0F] = { "SLO",  &Cpu::abs,  &Cpu::SLO, 6, false };
	lookup[0x1F] = { "SLO",  &Cpu::absX, &Cpu::SLO, 7, false };
	lookup[0x1B] = { "SLO",  &Cpu::absY, &Cpu::SLO, 7, false };
	lookup[0x03] = { "SLO",  &Cpu::Xind, &Cpu::SLO, 8, false };
	lookup[0x13] = { "SLO",  &Cpu::indY, &Cpu::SLO, 8, false };
	/* -------------------------------------------------
	SRE(LSE)
		LSR oper + EOR oper

		M = 0 ->[76543210]->C, A EOR M->A

		NZCIDV
		+++---
		addressing		assembler		opc	bytes	cycles
		zeropage		SRE oper		47	2		5
		zeropage, X		SRE oper, X		57	2		6
		absolute		SRE oper		4F	3		6
		absolute, X		SRE oper, X		5F	3		7
		absolute, Y		SRE oper, Y		5B	3		7
		(indirect, X)	SRE(oper, X)	43	2		8
		(indirect), Y	SRE(oper), Y	53	2		8
	*/
	lookup[0x47] = { "SRE",  &Cpu::zpg,  &Cpu::SRE, 5, false };
	lookup[0x57] = { "SRE",  &Cpu::zpgX, &Cpu::SRE, 6, false };
	lookup[0x4F] = { "SRE",  &Cpu::abs,  &Cpu::SRE, 6, false };
	lookup[0x5F] = { "SRE",  &Cpu::absX, &Cpu::SRE, 7, false };
	lookup[0x5B] = { "SRE",  &Cpu::absY, &Cpu::SRE, 7, false };
	lookup[0x43] = { "SRE",  &Cpu::Xind, &Cpu::SRE, 8, false };
	lookup[0x53] = { "SRE",  &Cpu::indY, &Cpu::SRE, 8, false };
	/* -------------------------------------------------
	TAS(XAS, SHS)
		Puts A AND X in SP and stores A AND X AND(high - byte of addr. + 1) at addr.

		unstable: sometimes 'AND (H+1)' is dropped, 
		page boundary crossings may not work(with the high - byte 
		of the value used as the high - byte of the address)

		A AND X->SP, A AND X AND(H + 1)->M

		NZCIDV
		------
		addressing	assembler	opc	bytes	cycles
		absolute, Y	TAS oper, Y	9B	3		5
	*/
	lookup[0x9B] = { "TAS",  &Cpu::absY,  &Cpu::TAS, 5, false };
	/* -------------------------------------------------
	USBC(SBC)
		SBC oper + NOP

		effectively same as normal SBC immediate, instr.E9.

		A - M - ~C ->A

		NZCIDV
		+++--+
		addressing	assembler	opc	bytes	cycles
		immediate	USBC #oper	EB	2		2
	*/
	lookup[0xEB] = { "USBC",  &Cpu::imm,  &Cpu::SBC, 2, false };
	/* -------------------------------------------------
	NOPs(including DOP, TOP)
		Instructions effecting in 'no operations' in various address modes.
		Operands are ignored.

		NZCIDV
		-------
		opc	addressing	bytes	cycles
		1A	implied		1		2
		3A	implied		1		2
		5A	implied		1		2
		7A	implied		1		2
		DA	implied		1		2
		FA	implied		1		2
		80	immediate	2		2
		82	immediate	2		2
		89	immediate	2		2
		C2	immediate	2		2
		E2	immediate	2		2
		04	zeropage	2		3
		44	zeropage	2		3
		64	zeropage	2		3
		14	zeropage, X	2		4
		34	zeropage, X	2		4
		54	zeropage, X	2		4
		74	zeropage, X	2		4
		D4	zeropage, X	2		4
		F4	zeropage, X	2		4
		0C	absolute	3		4
		1C	absolute, X	3		4 *
		3C	absolute, X	3		4 *
		5C	absolute, X	3		4 *
		7C	absolute, X	3		4 *
		DC	absolute, X	3		4 *
		FC	absolute, X	3		4 *
	*/
	lookup[0x1A] = { "NOP", &Cpu::impl, &Cpu::NOP, 2, false };
	lookup[0x3A] = { "NOP", &Cpu::impl, &Cpu::NOP, 2, false };
	lookup[0x5A] = { "NOP", &Cpu::impl, &Cpu::NOP, 2, false };
	lookup[0x7A] = { "NOP", &Cpu::impl, &Cpu::NOP, 2, false };
	lookup[0xDA] = { "NOP", &Cpu::impl, &Cpu::NOP, 2, false };
	lookup[0xFA] = { "NOP", &Cpu::impl, &Cpu::NOP, 2, false };
	lookup[0x80] = { "NOP", &Cpu::imm,  &Cpu::NOP, 2, false };
	lookup[0x82] = { "NOP", &Cpu::imm,  &Cpu::NOP, 2, false };
	lookup[0x89] = { "NOP", &Cpu::imm,  &Cpu::NOP, 2, false };
	lookup[0xC2] = { "NOP", &Cpu::imm,  &Cpu::NOP, 2, false };
	lookup[0xE2] = { "NOP", &Cpu::imm,  &Cpu::NOP, 2, false };
	lookup[0x04] = { "NOP", &Cpu::zpg,  &Cpu::NOP, 3, false };
	lookup[0x44] = { "NOP", &Cpu::zpg,  &Cpu::NOP, 3, false };
	lookup[0x64] = { "NOP", &Cpu::zpg,  &Cpu::NOP, 3, false };
	lookup[0x14] = { "NOP", &Cpu::zpgX, &Cpu::NOP, 4, false };
	lookup[0x34] = { "NOP", &Cpu::zpgX, &Cpu::NOP, 4, false };
	lookup[0x54] = { "NOP", &Cpu::zpgX, &Cpu::NOP, 4, false };
	lookup[0x74] = { "NOP", &Cpu::zpgX, &Cpu::NOP, 4, false };
	lookup[0xD4] = { "NOP", &Cpu::zpgX, &Cpu::NOP, 4, false };
	lookup[0xF4] = { "NOP", &Cpu::zpgX, &Cpu::NOP, 4, false };
	lookup[0x0C] = { "NOP", &Cpu::abs,  &Cpu::NOP, 4, false };
	lookup[0x1C] = { "NOP", &Cpu::absX, &Cpu::NOP, 4, true };
	lookup[0x3C] = { "NOP", &Cpu::absX, &Cpu::NOP, 4, true };
	lookup[0x5C] = { "NOP", &Cpu::absX, &Cpu::NOP, 4, true };
	lookup[0x7C] = { "NOP", &Cpu::absX, &Cpu::NOP, 4, true };
	lookup[0xDC] = { "NOP", &Cpu::absX, &Cpu::NOP, 4, true };
	lookup[0xFC] = { "NOP", &Cpu::absX, &Cpu::NOP, 4, true };
	/* -------------------------------------------------
	JAM(KIL, HLT)
		These instructions freeze the CPU.

		The processor will be trapped infinitely in T1 phase with $FF on the data bus.— Reset required.

		Instruction codes : 02, 12, 22, 32, 42, 52, 62, 72, 92, B2, D2, F2
	*/
	lookup[0x02] = { "JAM", &Cpu::impl, &Cpu::JAM, 1, true };
	lookup[0x12] = { "JAM", &Cpu::impl, &Cpu::JAM, 1, true };
	lookup[0x22] = { "JAM", &Cpu::impl, &Cpu::JAM, 1, true };
	lookup[0x32] = { "JAM", &Cpu::impl, &Cpu::JAM, 1, true };
	lookup[0x42] = { "JAM", &Cpu::impl, &Cpu::JAM, 1, true };
	lookup[0x52] = { "JAM", &Cpu::impl, &Cpu::JAM, 1, true };
	lookup[0x62] = { "JAM", &Cpu::impl, &Cpu::JAM, 1, true };
	lookup[0x72] = { "JAM", &Cpu::impl, &Cpu::JAM, 1, true };
	lookup[0x92] = { "JAM", &Cpu::impl, &Cpu::JAM, 1, true };
	lookup[0xB2] = { "JAM", &Cpu::impl, &Cpu::JAM, 1, true };
	lookup[0xD2] = { "JAM", &Cpu::impl, &Cpu::JAM, 1, true };
	lookup[0xF2] = { "JAM", &Cpu::impl, &Cpu::JAM, 1, true };
}

