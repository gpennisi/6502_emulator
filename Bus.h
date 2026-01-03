#pragma once
#include <cstdint>  
#include <array>    
#include <vector>   
#include <iostream>
#include <iomanip>

class Bus
{
public:
    // 64KB Address Space
    static constexpr size_t MEM_SIZE = 64 * 1024;

private:
    // 2KB System RAM (std::array is a class, has .fill)
    std::array<uint8_t, 2048> cpuRam;

    // Cartridge ROM (std::vector is a class, has .resize)
    std::vector<uint8_t> cartridgeRom;

public:
    // constructor with non-garbage initialization
    Bus() { 
        // Clear internal RAM on startup
        cpuRam.fill(0);
        // Resize ROM to 64KB just to be safe for simple testing
        // (Real cartridges vary in size, but this covers the address space)
        cartridgeRom.resize(MEM_SIZE, 0);
    }

    const uint8_t read(const uint16_t address) const
    {
        if (address <= 0x1fff)
        {
            // address & 0x07ff to simulate ram only reading 11 pins
            // so that 00000000000 and 100000000000 actually are the same
            return cpuRam[address & 0x07ff];
        }
        else if (address <= 0x3fff)
        {
            // implement PPU logic
            return 0x00;
        }
        else
        {
            return cartridgeRom[address];
        }

    }
    void write(const uint8_t byte, const uint16_t address)
    {
        if (address <= 0x1fff)
        {
            cpuRam[address & 0x07ff] = byte;
        }
        else if (address <= 0x3fff)
        {
            // implement PPU logic
            0;
        }
        else
        {
            std::cerr << "[Memory Error] Cannot write on Read-only memory!" << std::endl;
        }

    }

    // to load a series of instructions (i.e. insert a cartridge)
    bool loadProgram(const std::vector<uint8_t>& program, uint16_t startAddress)
    {   
        const size_t programSize = program.size();
        if (uint32_t(startAddress + programSize) > MEM_SIZE)
        {
            std::cerr << "[Memory Error] Program does not fit in RAM!" << std::endl;
            std::cerr << "Program End: " << (startAddress + programSize)
                << " > RAM Size: " << MEM_SIZE << std::endl;
            return false;
        }

        for (size_t i = 0; i < programSize; i++)
        {
            // bypass the bus logic, we simulate the loading the cartridge before powering on 
            // the machine, otherwise you cannot write read-only memory
            cartridgeRom[startAddress + i] = program[i];
        }
        return true;
    }

    void printValueAtMemory(const uint16_t address)
    {
        // print as hex, i.e. 0x8000 = 0xFF
        std::cout << "0x" << std::hex << std::setw(4) << std::setfill('0') << int(address)
            << " : 0x" << std::setw(2) << int(read(address))
            << std::endl;
    }

};


