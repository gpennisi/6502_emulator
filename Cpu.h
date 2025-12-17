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
		opcFunction addr;
		addrFunction opc;
		uint8_t cycles;
		penaltyFunction penalty;
	};
	// create a lookup table that can hold 151 + illegal options
	static Instruction lookup[0x100];
	void initInstructions();
	void execInstruction();

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

	void updateN(uint8_t& value);
	void updateZ(uint8_t& value);
	void updateV(uint8_t& value);
	void updateC(uint8_t& value);

	// reads the programcounter and automatically increments it
	uint8_t fetchByte();
	uint8_t pull();
	void push(const uint8_t& byte);
	void pushWord(const uint16_t& word);


	// Operation Codes
	void ADC(), JMP(), JSR(),
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

