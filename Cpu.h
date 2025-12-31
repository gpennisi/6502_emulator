//Cpu.h
 
#pragma once
#include <stdint.h>
#include <functional>
#include <string>
#include "Bus.h"
#include <iostream>
#include <iomanip> // for hex printing
#include <sstream>

class Cpu
{
private:
	Bus& bus;

	int cycleCounter;
	uint8_t A;
	uint8_t X;
	uint8_t Y;
	uint8_t S;
	uint8_t SR;
	uint16_t PC;

	enum P
	{
		N = 0x80,
		V = 0x40,
		U = 0x20,
		B = 0x10,
		D = 0x08,
		I = 0x04,
		Z = 0x02,
		C = 0x01
	};

	uint16_t currentAddress;
	bool isAccumulatorMode = false;
	uint8_t penalty = 0;

public:
	Cpu(Bus& b) : bus(b) {};


	//sets the initial state
	void reset();
	//simulate one step of CPU operation
	void cycle();

	// pointer to instruction function
	using opcFunction = void (Cpu::*)();
	using addrFunction = void (Cpu::*)();

	// lookup table struct
	struct Instruction {
		std::string inst;
		addrFunction addr;
		opcFunction opc;
		int cycles;
		bool penalty;
	};
	// create a lookup table that can hold 151 + illegal options
	static Instruction lookup[0x100];
	void initInstructions();
	void execInstruction(Instruction inst);
	// endAddress is uint32_t to wrap around when 0xFFFF+1
	std::string disassembleInstruction(const uint32_t addr);

	// flag manipulation
	void setFlag(P flag) { SR = SR | flag; };
	void clearFlag(P flag) {SR = SR & (~flag); };
	bool getFlag(P flag) { return SR & flag; }
	void updateFlag(P flag, bool condition)
	{
		if (condition) setFlag(flag); else clearFlag(flag);
	}

	// register getters/setters
	const uint8_t getRegA() { return A; }
	const uint8_t getRegX() { return X; }
	const uint8_t getRegY() { return Y; }
	const uint8_t getRegS() { return S; }
	const uint8_t getSR() { return SR; }
	const uint16_t getPC() { return PC; }
	void setPC(const uint16_t& address) { PC = address; }


	std::string cpuState(bool flat = true)
	{
		std::string spacer = flat ? " " : "\n ";
		std::stringstream ss;
		// std::left << std::setw(40)
		ss << std::hex << std::setfill('0')
			<< "PC: 0x" << std::setw(4) << int(getPC()) << spacer
			<< "A: 0x" << std::setw(2) << int(getRegA()) << spacer
			<< "X: 0x" << std::setw(2) << int(getRegX()) << spacer
			<< "Y: 0x" << std::setw(2) << int(getRegY()) << spacer
			<< "S: 0x" << std::setw(2) << int(getRegS()) << spacer
			<< "NVUBDIZC" << spacer
			<< getFlag(N)
			<< getFlag(U)
			<< getFlag(B)
			<< getFlag(D)
			<< getFlag(I)
			<< getFlag(Z)
			<< getFlag(C) << spacer
			<< "Cycles: " << std::dec << cycleCounter;

		return ss.str();
	}

	// core functions
	uint8_t fetchByte();
	uint8_t pull();
	uint16_t pullWord();
	void push(const uint8_t& byte);
	void pushWord(const uint16_t& word);
	void irq();
	void nmi();

	// helper functions
	uint16_t arithmetic(const uint16_t& value, std::function<uint16_t(uint16_t)> operation);
	void compare(uint8_t reg, const uint8_t& value);
	void load(uint8_t& reg, const uint8_t& value);
	void logic(std::function<void(uint8_t&)> operation);
	uint8_t shift(const uint8_t& mask, std::function<uint8_t(uint8_t)> operation);
	void absoluteIndexed(uint8_t& reg);
	void interrupt(uint16_t address, bool isSoftware);

	// Penalty Functions
	void crossBound(const uint16_t& baseAddress, const uint16_t& finalAddress);
	void branch();

	// --- Address Modes ---
	void acc(), abs(), absX(), absY(),
		imm(), impl(), ind(), Xind(), indY(),
		rel(), zpg(), zpgX(), zpgY();

	// --- Opcodes ---(this->*instruction.opc)();
	// Load / Store	
	void LDA(), LDX(), LDY(), STA(), STX(), STY();
	// Arithmetic	
	void ADC(), SBC(), INC(), INX(), INY(), DEC(), DEX(), DEY();
	// Logic	
	void AND(), EOR(), ORA(), BIT();
	// Shift / Rotate	
	void ASL(), LSR(), ROL(), ROR();
	// Transfer	
	void TAX(), TAY(), TSX(), TXA(), TXS(), TYA();
	// Stack	
	void PHA(), PHP(), PLA(), PLP();
	// Control Flow	
	void JMP(), JSR(), RTS(), RTI();
	// Branching	
	void BCC(), BCS(), BEQ(), BMI(), BNE(), BPL(), BVC(), BVS();
	// Register Flags	
	void CLC(), CLD(), CLI(), CLV(), SEC(), SED(), SEI();
	// Comparison	
	void CMP(), CPX(), CPY();
	//  System	
	void BRK(), NOP();
	// Illegal
	void ALR(), ANC(), ANE(), ARR(), DCP(), ISC(),
		LAS(), LAX(), LXA(), RLA(), RRA(), SAX(), SBX(), SHA(),
		SHX(), SHY(), SLO(), SRE(), TAS(), JAM();
};


