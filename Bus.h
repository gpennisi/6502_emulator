//Bus.h
#pragma once
#include <stdint.h>
#include <cstring> // for memset
#include <vector>
#include <iostream>
#include <iomanip> // for hex printing
#include <cstdlib> // exit()

class Bus
{
public:
    static constexpr size_t RAM_SIZE = 64 * 1024;
private:
    uint8_t ram[RAM_SIZE];
public:
    // constructor with non-garbage initialization
    Bus() { std::memset(ram, 0, RAM_SIZE); }

    const uint8_t read(const uint16_t address) const
    {
        return ram[address];
    }
    void write(const uint8_t byte, const uint16_t address)
    {
        if (address >= RAM_SIZE)
        {
            std::cerr << "[Memory Error] Write Out of Bounds: " << address << std::endl;
            return;
        }
        ram[address] = byte;
    }

    // to load a series of instructions 
    bool loadProgram(const std::vector<uint8_t>& program, uint16_t startAddress, bool debug = false)
    {
        if ((uint32_t)startAddress + program.size() > RAM_SIZE)
        {
            std::cerr << "[Memory Error] Program does not fit in RAM!" << std::endl;
            std::cerr << "Program End: " << (startAddress + program.size())
                << " > RAM Size: " << RAM_SIZE << std::endl;
            return false;
        }

        uint16_t current = startAddress;
        for (uint8_t byte : program) {
            // Usiamo 'this->write' così sfruttiamo il controllo bounds scritto sopra
            write(byte, current++);

            if (debug) {
                // Stampa hex pulita
                std::cout << "ADDR: " << std::hex << (current - 1)
                    << " DATA: " << int(byte) << std::endl;
            }
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