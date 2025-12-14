#pragma once
#include <stdint.h>

class Bus
{
private:
	uint8_t ram[0x10000];
public:
	uint8_t read(const uint16_t address);
	void write(const uint8_t byte, const uint16_t address);


};

