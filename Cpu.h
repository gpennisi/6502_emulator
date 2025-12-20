#pragma once
#include <stdint.h>
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

public:
	Cpu(Bus& b) : bus(b) {};


	//sets the initial state
	void reset();
	//simulate one step of CPU operation
	void cycle();


	// pointer to instruction function
	using opcFunction = void (Cpu::*)();
	using addrFunction = void (Cpu::*)();
	using penaltyFunction = void (Cpu::*)();
	// lookup table struct
	struct Instruction {
		std::string inst;
		addrFunction addr;
		opcFunction opc;
		uint8_t cycles;
		penaltyFunction penalty;
	};
	// create a lookup table that can hold 151 + illegal options
	static Instruction lookup[0x100];
	void initInstructions();
	void execInstruction(Instruction inst);

	// given a mask (flag), perform OR with the mask to set the flag
	// i.e N = 10000000 (0x80)
	// i.e. status_flag = 00001000
	// N OR status_flag = 10001000
	void setFlag(P flag) { status_reg = status_reg | flag; };
	// given a mask (flag), perform AND with NOT flag
	// i.e N = 10000000 (0x80), ~N = 01111111
	// i.e. status_flag = 10001000
	// ~N AND status_flag = 00001000
	void clearFlag(P flag) {status_reg = status_reg & (~flag); };
	// check the flag status by masking with P::<flag>
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


	// Operation Codes
	void ADC(), AND(), ASL(), BCC(), BCS(), BEQ(), BIT(), BMI(), BNE(),
		BPL(), BRK(), BVC(), BVS(), CLC(), CLD(), CLI(), CLV(), 
		CMP(), CPX(), CPY(), DEC(), DEX(), DEI(), EOR(), INC(), INX(), INY(),
		JMP(), JSR(),
		LDA(), LDX(), LDY(), NOP(),
		PHA(), PHP(), PLA(), PLP(),
		ROL(), ROR(),
		STA(), STX(), STY(),
		TAX(), TAY(), TSX(), TXA(), TXS(), TYA();

	// Address Modes
	void acc(), abs(), absX(), absY(),
		imm(), impl(), ind(), Xind(), indY(),
		rel(), zpg(), zpgX(), zpgY();

	// Penalty Functions
	void crossBound(), branch(), sameBound(), np();

};

