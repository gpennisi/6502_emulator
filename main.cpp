#include "Cpu.h"
#include <iostream>

// init hardware (outside so it's on heap)
Bus bus;
Cpu cpu(bus);

int main()
{
    // fillup instructions
    cpu.initInstructions();

    //  PROGRAM 1
    std::vector<uint8_t> program = {
        0xA9, 0x05,          // LDA #$05
        0x8D, 0x00, 0x01,    // STA $0100  
        0x8D, 0x10, 0x01,    // STA $0110
        0xAD, 0x00, 0x01,    // LDA $0100
        0x18,                // CLC
        0xE9, 0x01,          // SBC #$01
        0x8D, 0x00, 0x01,    // STA $0100
        0x18,                // CLC
        0x00                 // BRK  
    };

    // load program into mem
    bus.loadProgram(program, 0x0600, true);


    // reset vector (where PC will start from)
    bus.write(0x00, 0xFFFC);
    bus.write(0x06, 0xFFFD);

    // reset cpu
    cpu.reset();
  
    // execute program
    while (true) {
        cpu.cycle();

        //step by step debug
        //cpu.printCpuState();

        // exit when we hit break (for thsi test)
        if (bus.read(cpu.getPC()) == 0x00) break;

    }

    cpu.printCpuState();
    bus.printValueAtMemory(0x0100);
    bus.printValueAtMemory(0x0110);


    //  PROGRAM 2
    program = {
        0xA9, 0x20,          // LDA #$20
        0x8D, 0x11, 0X00,    // STA $0011 
        0x8D, 0x12, 0x00,    // STA $0012

        0xA2, 0x05,          // LDX #$05

        0xAD, 0x12, 0x00,    // LDA $0012
        0x18,                // CLC
        0x69, 0x01,          // ADC #$01
        0x8D, 0x12, 0x00,    // STA $0012
        0xCA,                // DEX

        0xD0, 0xF4,          // BNE ( 12 bytes from LDA $0012, -12 in two's compl is 0xF4)
        0xA0, 0x01,          // LDY #$01

        0x00                 // BRK  
    };

    // load program into mem
    bus.loadProgram(program, 0x0600, true);


    // reset vector (where PC will start from)
    bus.write(0x00, 0xFFFC);
    bus.write(0x06, 0xFFFD);

    // reset cpu
    cpu.reset();

    // execute program
    while (true) {
        cpu.cycle();

        //step by step debug
        //cpu.printCpuState();

        // exit when we hit break (for thsi test)
        if (bus.read(cpu.getPC()) == 0x00) break;

    }

    cpu.printCpuState();
    bus.printValueAtMemory(0x0011);
    bus.printValueAtMemory(0x0012);





    return 0;
}