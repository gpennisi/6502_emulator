//Bus.h
#pragma once
#include <stdint.h>
#include <cstring> // for memset
#include <vector>
#include <iostream>

class Bus
{
private:
    uint8_t ram[0x10000];
public:
    // constructor with non-garbage initialization
    Bus() { std::memset(ram, 0, sizeof(ram)); }

    const uint8_t read(const uint16_t address) { return ram[address]; };
    void write(const uint8_t byte, const uint16_t address) { ram[address] = byte; }

    // to load a series of instructions 
    void loadProgram(const std::vector<uint8_t>& program, uint16_t startAddress, bool debug = false) {
        uint16_t current = startAddress;
        for (uint8_t byte : program) {
            write(byte, current++);
            if (debug) std::cout << std::hex << (int)byte << std::endl;
        }
    }
    void printValueAtMemory(const uint16_t address)
    {
        std::cout << std::hex << (int)address << " : " 
            << (int)read(address) << std::endl;
    }

};