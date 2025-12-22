#pragma once
#include <stdint.h>
#include <functional>
#include "Bus.h"
class Cpu
{
public:
	static enum P
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

private:
	int cycleCounter;
	uint8_t A;
	uint8_t X;
	uint8_t Y;
	uint8_t S;
	uint8_t status_reg;
	uint16_t PC;

	Bus& bus;

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
		uint8_t cycles;
		bool penalty;
	};
	// create a lookup table that can hold 151 + illegal options
	static Instruction lookup[0x100];
	void initInstructions();
	void execInstruction(Instruction inst);

	// flag manipulation
	void setFlag(P flag) { status_reg = status_reg | flag; };
	void clearFlag(P flag) {status_reg = status_reg & (~flag); };
	bool getFlag(P flag) { return status_reg & flag; }
	void updateFlag(P flag, bool condition)
	{
		if (condition) setFlag(flag); else clearFlag(flag);
	}

	// reads the programcounter and automatically increments it
	uint8_t fetchByte();
	uint8_t pull();
	uint16_t pullWord();
	void push(const uint8_t& byte);
	void pushWord(const uint16_t& word);

	// helper functions
	uint16_t arithmetic(const uint16_t& value, std::function<uint16_t(uint16_t)> operation);
	void compare(uint8_t reg, const uint8_t& value);
	void load(uint8_t& reg, const uint8_t& value);
	void logic(std::function<void(uint8_t&)> operation);
	void shift(const uint8_t& mask, std::function<uint8_t(uint8_t)> operation);
	void absoluteIndexed(uint8_t& reg);

	// Penalty Functions
	void crossBound(const uint16_t& baseAddress, const uint16_t& finalAddress),
		branch();

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
};


