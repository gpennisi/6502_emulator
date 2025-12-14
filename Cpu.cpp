#include "Cpu.h"
#include "Bus.h"
#include <stdexcept>
#include <string>

Cpu::Cpu(Bus& b) : bus(b)
{
	
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
	// DECODE stage opcodes:
	// https://www.masswerk.at/6502/6502_instruction_set.html#modes


	// IMPLEMENT A LOOKUP TABLE **REVISE
	switch(opcode)
	{ 

	// NOP
	case 0xEA:
		cycleCounter += 2;
		break;
	// JMP absolute 
	case 0x4C:
		PC = absoluteAddress(0x0);
		cycleCounter += 3;
		break;
	// JMP indirect 
	case 0x6C:
		PC = indirectAddress();
		break;
	// JSR absolute 
	case 0x20:
		uint16_t origPC = PC;
		PC = absoluteAddress(0x0);
		pushWord(origPC -1);
		cycleCounter += 3;
		break;


	// ADC immediate ** TO REVISE
	case 0x69:
		uint8_t addend = fetchByte();
		uint16_t sum = A + addend + getFlag(C);
		
		if (sum > 0xFF) setFlag(C); else clearFlag(C);
		
		updateZN(A);
		
		bool overflow = (~(uint16_t(A) ^ addend) & (uint16_t(A) ^ sum)) & 0x0080;
		if (overflow) setFlag(V); else clearFlag(V);

		A = sum & 0xFF;
		break;
		
	// LDA immediate 
	case 0xA9:
		A = fetchByte();
		updateZN(A);
		cycleCounter += 2;
		break;
	// LDA zeropage 
	case 0xA5:
		A = bus.read(zeroPageAddress(0x0));
		updateZN(A);
		cycleCounter += 3;
		break;
	// LDA zeropage,X
	case 0xB5:
		A = bus.read(zeroPageAddress(X));
		updateZN(A);
		cycleCounter += 4;
		break;
	// LDA absolute
	case 0xAD:
		A = bus.read(absoluteAddress(0x0));
		updateZN(A);
		cycleCounter += 4;
		break;
	// LDA absolute,X
	case 0xBD:
		A = bus.read(absoluteAddress(X));
		updateZN(A);
		cycleCounter += 4; // **add penalty
		break;
	// LDA absolute,Y
	case 0xB9:
		A = bus.read(absoluteAddress(Y));
		cycleCounter += 4; // **add penalty
		break;
	// LDA (indirect,X) - preIndexed
	case 0xA1:
		A = indexedIndirectAddress();
		updateZN(A);
		cycleCounter += 6;
		break;
	// LDA (indirect), Y - postIndexed
	case 0xB1:
		A = indirectIndexedAddress();
		updateZN(A);
		cycleCounter += 5; // **add penalty
		break;

	// LDX immediate 
	case 0xA2:
		X = fetchByte();
		updateZN(X);
		cycleCounter += 2;
		break;
	// LDX zeropage 
	case 0xA6:
		X = bus.read(zeroPageAddress(0x0));
		updateZN(X);
		cycleCounter += 3;
		break;
	// LDX zeropage,Y
	case 0xB6:
		X = bus.read(zeroPageAddress(Y));
		updateZN(X);
		cycleCounter += 4;
		break;
	// LDX absolute
	case 0xAE:
		X = bus.read(absoluteAddress(0x0));
		updateZN(X);
		cycleCounter += 4;
		break;
	// LDX absolute,Y
	case 0xBE:
		X = bus.read(absoluteAddress(Y));
		updateZN(X);  // **add penalty
		cycleCounter += 4;
		break;


	// LDY immediate 
	case 0xA0:
		Y = fetchByte();
		updateZN(Y);
		cycleCounter += 2;
		break;
	// LDY zeropage 
	case 0xA4:
		Y = bus.read(zeroPageAddress(0x0));
		updateZN(Y);
		cycleCounter += 3;
		break;
	// LDY zeropage,X
	case 0xB4:
		Y = bus.read(zeroPageAddress(X));
		updateZN(Y);
		cycleCounter += 4;
		break;
	// LDY absolute
	case 0xAC:
		Y = bus.read(absoluteAddress(0x0));
		updateZN(Y);
		cycleCounter += 4;
		break;
	// LDY absolute,X
	case 0xBC:
		Y = bus.read(absoluteAddress(X));
		updateZN(Y);  // **add penalty
		cycleCounter += 4;
		break;

	// STA zeropage 
	case 0x85:
		bus.write(A, zeroPageAddress(0x0));
		cycleCounter += 3;
		break;
	// STA zeropage,X
	case 0x95:
		bus.write(A, zeroPageAddress(X));
		cycleCounter += 4;
		break;
	// STA absolute
	case 0x8D:
		bus.write(A, absoluteAddress(0x0) );
		cycleCounter += 4;
		break;
	// STA absolute,X
	case 0x9D:
		bus.write(A, absoluteAddress(X));
		cycleCounter += 5;
		break;
	// STA absolute,Y
	case 0x99:
		bus.write(A, absoluteAddress(Y));
		cycleCounter += 5;
		break;
	// STA (indirect,X) - preIndexed
	case 0x81:
		bus.write(A, indexedIndirectAddress());
		cycleCounter += 6;
		break;
	// STA (indirect), Y - postIndexed
	case 0x91:
		bus.write(A, indirectIndexedAddress());
		cycleCounter += 6;
		break;

	// STX zeropage 
	case 0x86:
		bus.write(X, zeroPageAddress(0x0));
		cycleCounter += 3;
		break;
	// STX zeropage,Y
	case 0x96:
		bus.write(X, zeroPageAddress(Y));
		cycleCounter += 4;
		break;
	// STX absolute
	case 0x8E:
		bus.write(X, absoluteAddress(0x0));
		cycleCounter += 4;
		break;

	//  STY zeropage 
	case 0x8C:
		bus.write(Y, zeroPageAddress(0x0));
		cycleCounter += 3;
		break;
	// STY zeropage,X
	case 0x84:
		bus.write(Y, zeroPageAddress(X));
		cycleCounter += 4;
		break;
	// STY absolute
	case 0x94:
		bus.write(Y, absoluteAddress(0x0));
		cycleCounter += 4;
		break;
	
	// TAX
	case 0xAA:
		X = A;
		updateZN(X);
		break;
	// TAY
	case 0xA8:
		Y = A;
		updateZN(Y);
		break;
	// TXA
	case 0x8A:
		A = X;
		updateZN(A);
		break;
	// TYA
	case 0x98:
		A = Y;
		updateZN(A);
		break;
	// TXS
	case 0x9A:
		X = S;
		break;
	// TSX
	case 0xBA:
		S = X;
		break;

	// PHA
	case 0x48:
		push(A);
		cycleCounter += 3;
		break;
	// PHP - The status register will be pushed with the break flag and bit 5 set to 1
	case 0x08:
		push(status_reg | 0x20 | 0x10);
		cycleCounter += 3;
		break;
	// PLA
	case 0x68:
		A = pull();
		updateZN(A);
		cycleCounter += 4;
		break;
	// PLP - The status register will be pulled with the break flag and bit 5 ignored
	case 0x28:
		status_reg = pull() | (status_reg & 0x20) | (status_reg & 0x10);
		cycleCounter += 4;
		break;


	// ROL A
	case 0x2A:
		A = rotate(A, 'l');
		cycleCounter += 2;
		break;
	// ROL zeropage
	case 0x26:
		address = zeroPageAddress(0x0);
		bus.write( 
			Cpu::rotate(bus.read(address), 'l'), 
			address
		);
		cycleCounter += 5;
		break;
	// ROL zeropage, X
	case 0x36:
		address = zeroPageAddress(X);
		bus.write(
			Cpu::rotate(bus.read(address), 'l'),
			address
		);
		cycleCounter += 6;
		break;
	// ROL absolute 
	case 0x2E:
		address = absoluteAddress(0x0);
		bus.write(
			Cpu::rotate(bus.read(address), 'l'),
			address
		);
		cycleCounter += 6;
		break;
	// ROL absolute, X
	case 0x3E:
		address = absoluteAddress(X);
		bus.write(
			Cpu::rotate(bus.read(address), 'l'),
			address
		);
		cycleCounter += 7;
		break;

	// ROR A
	case 0x6A:
		A = rotate(A, 'r');
		cycleCounter += 2;
		break;
	// ROR zeropage
	case 0x66:
		address = zeroPageAddress(0x0);
		bus.write(
			Cpu::rotate(bus.read(address), 'r'),
			address
		);
		cycleCounter += 5;
		break;
	// ROR zeropage, X
	case 0x76:
		address = zeroPageAddress(X);
		bus.write(
			Cpu::rotate(bus.read(address), 'r'),
			address
		);
		cycleCounter += 6;
		break;
	// ROR absolute 
	case 0x6E:
		address = absoluteAddress(0x0);
		bus.write(
			Cpu::rotate(bus.read(address), 'r'),
			address
		);
		cycleCounter += 6;
		break;
	// ROR absolute, X
	case 0x7E:
		address = absoluteAddress(X);
		bus.write(
			Cpu::rotate(bus.read(address), 'r'),
			address
		);
		cycleCounter += 7;
		break;

	default:
		break;
	}

}
	
uint8_t Cpu::rotate(const uint8_t& value, const char& side)
{	
	uint8_t rot_value;
	if(side == 'l') { 
		rot_value = (value << 1) | uint8_t(Cpu::getFlag(C));
		// set C if MSB is 1, otherwise clear C
		if (value & 0x80) Cpu::setFlag(C);
		else Cpu::clearFlag(C);
	}
	else if (side == 'r') { 
		rot_value = (value >> 1) | uint8_t(Cpu::getFlag(C) << 7);
		// set C if LSB is 1, otherwise clear C
		if (value & 0x01) Cpu::setFlag(C);
		else Cpu::clearFlag(C);
	}

	if (rot_value == 0) setFlag(Z); else clearFlag(Z);
	if (rot_value & 0x80) setFlag(N); else clearFlag(N);

	return rot_value;
}


void Cpu::rotateZeroPage(
	const uint8_t& index_reg,
	const char& side)
{
	uint8_t zero_page_address = fetchByte() + index_reg;
	bus.write(Cpu::rotate(bus.read(zero_page_address), side), zero_page_address);

	cycleCounter += 5;
}


void Cpu::rotateAbsolute(
	const uint8_t& index_reg,
	const char& side)
{
	uint16_t address = absoluteAddress(index_reg);
	
	uint8_t data = bus.read(address);
	uint8_t result = rotate(data, side);
	bus.write(result, address);

	cycleCounter += 6;

}


uint16_t Cpu::absoluteAddress(const uint8_t& index_reg)
{
	uint8_t ll = fetchByte();
	uint8_t hh = fetchByte();
	// shift high_byte and OR low_byte with it
	uint16_t address = ((hh << 8) | ll);
	uint16_t final_address = address + index_reg;

	// If the high byte changed, we crossed a page boundary
	if ((address & 0xFF00) != (final_address & 0xFF00)) {
		cycleCounter++;
	}

	return final_address;
}


uint16_t Cpu::zeroPageAddress(const uint8_t& index_reg)
{
	// from $00 to $FF. the leading bits are always 00
	// it can only therefore address 2^8=256-bits of mem
	// advantage: read 8 bits instead of 16 to get 8 bits value, one less cycle
	// get the page position
	uint8_t zero_page_address = fetchByte() + index_reg;
	return zero_page_address;
}



uint16_t Cpu::indexedIndirectAddress()
{
	uint16_t pointer = (fetchByte() + X) & 0xFF;
	uint8_t	ll = bus.read(pointer);
	uint8_t hh = bus.read((pointer + 1) & 0xFF);

	return  (hh << 8) | ll;
}


uint16_t Cpu::indirectIndexedAddress()
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
	return indirectAddress + Y;
}


uint16_t Cpu::indirectAddress()
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

	return ((uint16_t)indirect_hh << 8) | indirect_ll;

}

uint8_t Cpu::pull()
{
	S++;
	return bus.read(0x100 + S);
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


void Cpu::updateZN(uint8_t value)
{
	if (value == 0) Cpu::setFlag(Z); else Cpu::clearFlag(Z);
	if (value & 0x80) Cpu::setFlag(N); else Cpu::clearFlag(N);
}


uint8_t Cpu::fetchByte()
{
	const uint8_t fetch = bus.read(PC);
	PC++;
	return fetch;
}
